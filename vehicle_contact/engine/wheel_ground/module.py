"""Per-wheel orchestration and wheel state.

The module is the wheel-ground boundary: it owns support queries, suspension
and tyre evaluation and exposes a step result the chassis integrator consumes.
It never moves the chassis; it only reports forces at contact points.

Support is never assumed from a query hit alone.  A probe is a candidate; the
finite suspension reach rule in :mod:`support` decides whether it becomes
support, and an unsupported wheel produces exactly zero force.
"""

from math import cos, sin

from .. import math3d as V
from .frames import ContactFrame, relative_velocity
from .support import SupportQuery
from .suspension import (SuspensionParameters, evaluate_suspension,
                         relax_airborne)
from .tire import (TireParameters, evaluate_tire, free_spin_omega,
                   relax_to_rolling, update_steer)


class Wheel:
    """One corner: geometry, suspension parameters and current contact state."""

    def __init__(self, name, anchor_body, radius, powered, steered):
        self.name = name
        # Attachment point in body-local coordinates (x forward, z lateral).
        self.anchor_body = anchor_body
        self.radius = radius
        self.powered = powered
        self.steered = steered

        self.suspension = SuspensionParameters()

        # Suspension state.
        self._compression = 0.0
        self.compression_vel = 0.0

        # Contact state, refreshed every step by an explicit shape query.
        self.grounded = False
        self.contact_point = None
        self.contact_normal = None
        self.contact_distance = None
        self.query_kind = None
        # Solved support geometry (world), published for the visible wheel:
        # the anchor and direction actually used by this step's query.
        self.center = None
        self.anchor_world = None
        self.support_dir = None

        # Spin bookkeeping (radians, positive = forward rolling).
        self.spin_ground = 0.0
        self.spin_air = 0.0
        self.omega = 0.0
        self.steer_angle = 0.0

        # Last force contributions (for the diagnostics UI).
        self.f_suspension = (0.0, 0.0, 0.0)
        self.f_tire = (0.0, 0.0, 0.0)
        self.slip = 0.0
        self.load = 0.0

    # Suspension fields stay directly readable/writable on the wheel so the
    # verified live-tuning path is unchanged; they are stored in one parameter
    # object shared with the suspension evaluator.
    @property
    def rest_length(self):
        return self.suspension.rest_length

    @rest_length.setter
    def rest_length(self, value):
        self.suspension.rest_length = value

    @property
    def max_travel(self):
        return self.suspension.max_travel

    @max_travel.setter
    def max_travel(self, value):
        self.suspension.max_travel = value

    @property
    def spring_k(self):
        return self.suspension.spring_k

    @spring_k.setter
    def spring_k(self, value):
        self.suspension.spring_k = value

    @property
    def tau(self):
        return self.suspension.tau

    @tau.setter
    def tau(self, value):
        self.suspension.tau = value

    @property
    def compression(self):
        return self._compression

    @compression.setter
    def compression(self, value):
        self._compression = value

    @property
    def spin_total(self):
        return self.spin_ground + self.spin_air

    def effective_damping(self, sprung_mass):
        return self.suspension.effective_damping(sprung_mass)


class WheelStep:
    """Force contribution of one wheel for a single step."""

    __slots__ = ("wheel", "grounded", "f_suspension", "f_tire",
                 "contact_offset")

    def __init__(self, wheel, grounded, f_suspension, f_tire, contact_offset):
        self.wheel = wheel
        self.grounded = grounded
        self.f_suspension = f_suspension
        self.f_tire = f_tire
        self.contact_offset = contact_offset


class WheelGroundModule:
    """Contact boundary for a set of wheels over one terrain."""

    def __init__(self, terrain, wheels, query_kind="sphere", tire=None,
                 steer_limit=0.0, query=None):
        self.terrain = terrain
        self.wheels = wheels
        self.query = query or SupportQuery(terrain, query_kind)
        self.tire = tire or TireParameters()
        self.steer_limit = steer_limit

    # --------------------------------------------------------- geometry
    def wheel_anchor_world(self, body, wheel):
        return V.add(body.position,
                     V.body_to_world(body.quat, wheel.anchor_body))

    def suspension_direction(self, body):
        return V.norm(V.body_to_world(body.quat, (0.0, -1.0, 0.0)))

    # ------------------------------------------------------------ step
    def per_wheel_step(self, body, control, dt):
        """Refresh every wheel's support and forces.

        Returns ``(steps, warnings)``; ``steps`` preserves wheel order so the
        chassis integrator reproduces the reference force accumulation order.
        """
        steps = []
        warnings = []
        susp_dir = self.suspension_direction(body)
        powered_count = sum(1 for w in self.wheels if w.powered)

        for w in self.wheels:
            w.query_kind = self.query.kind
            prev_grounded = w.grounded
            w.grounded = False
            w.contact_point = None
            w.contact_normal = None
            w.contact_distance = None

            anchor = self.wheel_anchor_world(body, w)
            w.anchor_world = anchor
            w.support_dir = susp_dir
            probe = self.query.probe(anchor, susp_dir,
                                     w.rest_length + w.max_travel, w.radius)

            if probe is None:
                self._unsupported(w, dt, prev_grounded, warnings, relax=True)
                w.center = V.add(anchor, V.scale(
                    susp_dir, w.rest_length + w.max_travel))
                self._air_spin(w, body, control, dt, powered_count)
                steps.append(WheelStep(w, False, (0.0, 0.0, 0.0),
                                       (0.0, 0.0, 0.0), None))
                continue

            distance = self.query.quantize_distance(probe.distance)
            if not self.query.within_reach(distance, w.rest_length,
                                           w.max_travel):
                # A query hit is a candidate only: one beyond the finite
                # suspension reach is not support.  No force is applied and
                # the previous surface is never retained.
                self._unsupported(w, dt, prev_grounded, warnings, relax=False)
                w.center = V.add(anchor, V.scale(
                    susp_dir, w.rest_length + w.max_travel))
                self._air_spin(w, body, control, dt, powered_count)
                steps.append(WheelStep(w, False, (0.0, 0.0, 0.0),
                                       (0.0, 0.0, 0.0), None))
                continue

            compression = w.suspension.support_compression(distance)
            w.grounded = True
            w.contact_point = probe.point
            w.contact_normal = probe.normal
            w.contact_distance = distance
            w.center = V.add(anchor, V.scale(susp_dir, distance))
            w.compression_vel = (compression - w.compression) / dt
            w.compression = compression
            if not prev_grounded:
                warnings.append("%s gained support" % w.name)
            if compression >= w.max_travel - 1e-4:
                warnings.append("%s suspension bottomed" % w.name)

            f_susp, f_tire, offset = self._ground_forces(body, w, control, dt,
                                                         powered_count)
            w.f_suspension, w.f_tire = f_susp, f_tire
            w.steer_deg = V.deg(w.steer_angle)
            steps.append(WheelStep(w, True, f_susp, f_tire, offset))

        return steps, warnings

    @staticmethod
    def _unsupported(w, dt, prev_grounded, warnings, relax):
        if relax:
            # Relax toward full extension on the spring time constant; the
            # query result (or lack of one) is never cached as support.
            new_compression, rate = relax_airborne(w.suspension, w.compression,
                                                   dt)
            w.compression_vel = rate
            w.compression = new_compression
        if prev_grounded:
            warnings.append("%s lost support (edge/air)" % w.name)
        w.f_suspension = (0.0, 0.0, 0.0)
        w.f_tire = (0.0, 0.0, 0.0)
        w.load = 0.0

    # --------------------------------------------------------- forces
    def _ground_forces(self, body, w, control, dt, powered_count):
        n = w.contact_normal
        sprung = body.mass / len(self.wheels)
        f_susp = evaluate_suspension(w.suspension, w.compression,
                                     w.compression_vel, n, sprung).force

        offset = V.sub(w.contact_point, body.position)
        v_contact = relative_velocity(body.velocity, body.angular_velocity,
                                      offset)

        target_steer = (control.get("steer", 0.0) * self.steer_limit
                        if w.steered else 0.0)
        w.steer_angle = update_steer(w.steer_angle, target_steer, dt)
        fwd = V.norm(V.body_to_world(
            body.quat, (cos(w.steer_angle), 0.0, sin(w.steer_angle))))
        frame = ContactFrame(n, fwd)

        load = max(V.dot(f_susp, n), 0.0)
        w.load = load
        _, v_long, v_lat = frame.decompose(v_contact)

        tres = evaluate_tire(
            self.tire, control, load, v_long, v_lat, frame.forward,
            frame.lateral, w.omega, w.radius, self.wheel_inertia(w, body),
            body.mass, w.powered, powered_count, dt)
        w.slip = tres.slip
        w.omega = tres.omega
        return f_susp, tres.force, offset

    def wheel_inertia(self, w, body):
        # Solid-disk approximation of one corner's unsprung mass.
        corner_mass = body.mass / len(self.wheels) * 0.12 + 8.0
        return 0.5 * corner_mass * w.radius * w.radius

    # ------------------------------------------------------ wheel spin
    def _air_spin(self, w, body, control, dt, powered_count):
        w.omega = free_spin_omega(self.tire, control, w.radius,
                                  self.wheel_inertia(w, body), w.powered,
                                  powered_count, w.omega, dt)
        w.spin_air += w.omega * dt

    def integrate_ground_spin(self, body, dt):
        """Grounded wheels: visual angle follows the true rolling relation."""
        for w in self.wheels:
            if not w.grounded:
                continue
            offset = V.sub(w.contact_point, body.position)
            v_contact = relative_velocity(body.velocity,
                                          body.angular_velocity, offset)
            fwd = V.norm(V.body_to_world(
                body.quat, (cos(w.steer_angle), 0.0, sin(w.steer_angle))))
            v_long = V.dot(v_contact, fwd)
            w.omega = relax_to_rolling(w.omega, v_long, w.radius, dt)
            w.spin_ground += w.omega * dt
