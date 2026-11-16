"""Legacy vehicle-contact facade over the wheel-ground module.

This module preserves the original :class:`Vehicle` entry point and its
observable behaviour.  Support queries, suspension and tyre forces now live in
``engine.wheel_ground``; this facade only assembles the chassis rigid-body step
around them.

The pre-refactor records in ``data/evidence/`` are the contract: replaying the
standard scenarios must reproduce every field exactly.  The facade must not
turn a query *hit* into unlimited stable support — reach and support decisions
belong to the wheel-ground module, and an unsupported wheel contributes zero
force.
"""

from . import math3d as V
from .wheel_ground import (TireParameters, Wheel, WheelGroundModule)
from .wheel_ground.legacy import LegacyVehicleAdapter
from .wheel_ground.interface import classify_warnings, wheel_ground_state
from .wheel_ground.support import SupportQuery

GRAVITY = (0.0, -9.81, 0.0)


class Vehicle:
    def __init__(self, terrain, params=None):
        self.terrain = terrain
        p = params or {}

        # Rigid body state (world frame).
        self.mass = p.get("mass", 900.0)
        self.half_len = p.get("half_len", 1.7)
        self.half_w = p.get("half_w", 0.85)
        self.half_h = p.get("half_h", 0.45)
        self.inertia_body = self._box_inertia()

        self.position = list(p.get("position", (0.0, 1.2, 0.0)))
        self.velocity = [0.0, 0.0, 0.0]
        self.quat = V.q_identity()
        self.angular_velocity = [0.0, 0.0, 0.0]

        # Tunable global suspension/tyre parameters (live-editable in UI).
        self.tire_mu = p.get("tire_mu", 1.1)
        self.engine_force = p.get("engine_force", 3200.0)
        self.brake_force = p.get("brake_force", 6000.0)
        self.max_steer = V.rad(p.get("max_steer_deg", 28.0))
        self.rolling_resistance = p.get("rolling_resistance", 26.0)
        self.aero_drag = p.get("aero_drag", 1.6)

        wheel_r = p.get("wheel_radius", 0.32)
        fx, fz = self.half_len * 0.82, self.half_w * 0.9
        self.wheels = [
            Wheel("FL", (fx, -0.15, fz), wheel_r, True, True),
            Wheel("FR", (fx, -0.15, -fz), wheel_r, True, True),
            Wheel("RL", (-fx, -0.15, fz), wheel_r, True, False),
            Wheel("RR", (-fx, -0.15, -fz), wheel_r, True, False),
        ]
        # Apply any per-suspension overrides.
        susp = p.get("suspension", {})
        for w in self.wheels:
            w.rest_length = susp.get("rest_length", w.rest_length)
            w.max_travel = susp.get("max_travel", w.max_travel)
            w.spring_k = susp.get("spring_k", w.spring_k)
            w.tau = susp.get("tau", w.tau)

        self.contact = WheelGroundModule(
            terrain, self.wheels,
            query_kind=p.get("query_kind", "sphere"),
            tire=TireParameters(self.tire_mu, self.engine_force,
                                self.brake_force, self.rolling_resistance),
            steer_limit=self.max_steer)

        # Cached diagnostics.
        self.last_contact_count = 0
        self.warnings = []
        self.body_contact = False

    # ------------------------------------------------------------ inertia
    def _box_inertia(self):
        m, hx, hy, hz = self.mass, self.half_len, self.half_h, self.half_w
        return (m / 3.0 * (hy * hy + hz * hz),
                m / 3.0 * (hx * hx + hz * hz),
                m / 3.0 * (hx * hx + hy * hy))

    # ------------------------------------------------- support query kind
    @property
    def query_kind(self):
        return self.contact.query.kind

    @query_kind.setter
    def query_kind(self, kind):
        self.contact.query = SupportQuery(self.terrain, kind)

    # ------------------------------------------------------- tuning API
    def suspension_params(self):
        """Live parameters shown/edited in the contact window."""
        w0 = self.wheels[0]
        sprung = self.mass / len(self.wheels)
        c, zeta = w0.effective_damping(sprung)
        return {
            "query_kind": self.query_kind,
            "rest_length": w0.rest_length,
            "max_travel": w0.max_travel,
            "spring_k": w0.spring_k,
            "tau": w0.tau,
            "damping_ratio": zeta,
            "damping_c": c,
            "tire_mu": self.tire_mu,
            "wheel_radius": w0.radius,
        }

    def set_suspension_params(self, cfg):
        for w in self.wheels:
            if "rest_length" in cfg:
                w.rest_length = float(cfg["rest_length"])
            if "max_travel" in cfg:
                w.max_travel = float(cfg["max_travel"])
            if "spring_k" in cfg:
                w.spring_k = float(cfg["spring_k"])
            if "tau" in cfg:
                w.tau = float(cfg["tau"])
        if "query_kind" in cfg:
            self.query_kind = cfg["query_kind"]
        if "tire_mu" in cfg:
            self.tire_mu = float(cfg["tire_mu"])
            self.contact.tire.mu = self.tire_mu

    # ------------------------------------------------------------- helpers
    def wheel_anchor_world(self, wheel):
        return self.contact.wheel_anchor_world(self, wheel)

    def suspension_dir(self):
        """Suspension travel direction in world frame (body-down)."""
        return self.contact.suspension_direction(self)

    def wheel_ground(self):
        """Public wheel-ground interface (physics results for consumers)."""
        return wheel_ground_state(self)

    def legacy_adapter(self):
        """Compatibility adapter for the pre-refactor per-wheel entry points."""
        return LegacyVehicleAdapter(self)

    # ====================================================== physics step ==
    def step(self, dt, control):
        """Advance one fixed step.

        ``control``: dict with throttle [0..1], brake [0..1], steer [-1..1].
        Returns the diagnostics snapshot for this step.
        """
        self.warnings = []
        force_total = [0.0, 0.0, 0.0]
        torque_total = [0.0, 0.0, 0.0]

        # Gravity and quadratic aerodynamic drag act on the whole body.
        for i in range(3):
            force_total[i] += GRAVITY[i] * self.mass
        speed = V.length(self.velocity)
        if speed > 1e-6:
            drag = V.scale(self.velocity, -self.aero_drag * speed)
            force_total = V.add(force_total, drag)

        steps, wheel_warnings = self.contact.per_wheel_step(self, control, dt)
        self.warnings.extend(wheel_warnings)

        self.last_contact_count = 0
        for s in steps:
            if not s.grounded:
                continue
            self.last_contact_count += 1
            force_total = V.add(force_total, s.f_suspension)
            force_total = V.add(force_total, s.f_tire)
            r = s.contact_offset
            torque_total = V.add(torque_total, V.cross(r, s.f_suspension))
            torque_total = V.add(torque_total, V.cross(r, s.f_tire))

        if self.last_contact_count == 0:
            self.warnings.append("all wheels airborne")

        # Integrate linear momentum.
        accel = V.scale(force_total, 1.0 / self.mass)
        for i in range(3):
            self.velocity[i] += accel[i] * dt
            self.position[i] += self.velocity[i] * dt

        # Integrate angular momentum in body frame (inertia principal axes).
        tau_body = V.world_to_body(self.quat, torque_total)
        ang_accel_body = (
            tau_body[0] / self.inertia_body[0],
            tau_body[1] / self.inertia_body[1],
            tau_body[2] / self.inertia_body[2],
        )
        omega_body = V.world_to_body(self.quat, self.angular_velocity)
        omega_body = V.add(omega_body, V.scale(ang_accel_body, dt))
        self.angular_velocity = list(V.body_to_world(self.quat, omega_body))
        self.quat = V.q_integrate(self.quat, self.angular_velocity, dt)

        self._resolve_body_penetration()
        self.contact.integrate_ground_spin(self, dt)
        return self.snapshot()

    def _resolve_body_penetration(self):
        """Safety net: the chassis hull must never tunnel through terrain.

        This never drives normal motion; it only cancels an illegal overlap
        and reports a body-contact warning through the contact interface.
        """
        q = self.quat
        self.body_contact = False
        for sx in (-1, 1):
            for sz in (-1, 1):
                local = (sx * self.half_len, -self.half_h, sz * self.half_w)
                corner = V.add(self.position, V.body_to_world(q, local))
                ground = self.terrain.height(corner[0], corner[2])
                if corner[1] < ground:
                    pen = ground - corner[1]
                    self.position[1] += pen
                    if self.velocity[1] < 0:
                        self.velocity[1] = 0.0
                    self.body_contact = True
        if self.body_contact:
            self.warnings.append("chassis hull contacted terrain")

    # ----------------------------------------------------------- snapshot
    def snapshot(self):
        roll, pitch, yaw = V.q_to_euler(self.quat)
        return {
            "position": tuple(self.position),
            "velocity": tuple(self.velocity),
            "speed": V.length(self.velocity),
            "orientation_euler": (V.deg(roll), V.deg(pitch), V.deg(yaw)),
            "angular_velocity": tuple(self.angular_velocity),
            "contact_count": self.last_contact_count,
            "body_contact": self.body_contact,
            "warnings": list(self.warnings),
            "anomalies": classify_warnings(self.warnings),
            "wheels": [
                {
                    "name": w.name,
                    "grounded": w.grounded,
                    "query": w.query_kind,
                    "compression": w.compression,
                    "max_travel": w.max_travel,
                    "travel_ratio": w.compression / max(w.max_travel, 1e-9),
                    "contact_point": w.contact_point,
                    "contact_normal": w.contact_normal,
                    "contact_distance": w.contact_distance,
                    "load": w.load,
                    "slip": w.slip,
                    "steer_deg": V.deg(w.steer_angle),
                    "omega": w.omega,
                    "spin_ground": w.spin_ground,
                    "spin_air": w.spin_air,
                    "spin_total": w.spin_total,
                    "f_suspension": w.f_suspension,
                    "f_tire": w.f_tire,
                }
                for w in self.wheels
            ],
        }
