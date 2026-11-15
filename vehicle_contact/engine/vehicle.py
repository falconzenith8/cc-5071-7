"""Rigid-body wheeled vehicle driven entirely by explicit contact queries.

Per simulation step every wheel performs its own ray or sphere cast against
the terrain.  The query result (contact point, normal, distance) is the *only*
thing that decides whether that wheel is supported: no part of the code moves
the chassis from key presses or projects it onto the ground.

Suspension model
----------------
Each wheel hangs below a body-fixed anchor.  When the cast hits ground inside
``rest_length + max_travel`` the wheel is grounded and its spring/damper
force, plus the tyre forces, are applied at the real contact point.  The
damper is parameterised through a relaxation **time constant** ``tau``
(``c = 2 * sqrt(k * m_eff) / (2*zeta)`` with ``zeta`` derived from ``tau``),
so the requested suspension settling time can be tuned directly and the
effective damping ratio is reported back in diagnostics.

Wheel spin
----------
Two separate angles are accumulated per wheel:

* ``spin_ground``  grows only while grounded, from the true rolling relation
  (contact-point longitudinal velocity / wheel radius);
* ``spin_air``     grows only while airborne, integrating free spin under
  whatever drive/brake torque remains.

The rendered angle is their sum, so an airborne wheel can spin freely without
its visual rotation contradicting chassis motion.
"""

from math import sqrt, sin, cos, atan2, pi
from . import math3d as V

GRAVITY = (0.0, -9.81, 0.0)


class Wheel:
    def __init__(self, name, anchor_body, radius, powered, steered):
        self.name = name
        # Attachment point in body-local coordinates (x forward, z lateral).
        self.anchor_body = anchor_body
        self.radius = radius
        self.powered = powered
        self.steered = steered

        # Suspension state.
        self.rest_length = 0.35
        self.max_travel = 0.22          # extra compression beyond rest
        self.spring_k = 32000.0         # N/m
        self.tau = 0.12                 # relaxation time constant [s]
        self.compression = 0.0          # current compression [m]
        self.compression_vel = 0.0      # d(compression)/dt [m/s]

        # Contact state, refreshed every step by an explicit shape query.
        self.grounded = False
        self.contact_point = None
        self.contact_normal = None
        self.contact_distance = None
        self.query_kind = None

        # Spin bookkeeping (radians, positive = forward rolling).
        self.spin_ground = 0.0
        self.spin_air = 0.0
        self.omega = 0.0                # wheel angular velocity [rad/s]
        self.steer_angle = 0.0

        # Last force contributions (for the diagnostics UI).
        self.f_suspension = (0.0, 0.0, 0.0)
        self.f_tire = (0.0, 0.0, 0.0)
        self.slip = 0.0
        self.load = 0.0

    @property
    def spin_total(self):
        return self.spin_ground + self.spin_air

    def effective_damping(self, sprung_mass):
        """Damper coefficient c from spring k and time constant tau.

        For a mass-spring-damper, zeta = c / (2 sqrt(k m)).  A critically
        damped system settles with tau ~ 1/omega_n where omega_n=sqrt(k/m);
        we map the user time constant onto zeta so small tau => stiff/fast,
        large tau => soft/slow, and clamp to a stable positive range.
        """
        omega_n = sqrt(self.spring_k / max(sprung_mass, 1e-6))
        zeta = 1.0 / max(self.tau * omega_n, 0.05)
        zeta = V.clamp(zeta, 0.15, 3.0)
        c = 2.0 * zeta * sqrt(self.spring_k * max(sprung_mass, 1e-6))
        return c, zeta


class Vehicle:
    def __init__(self, terrain, params=None):
        self.terrain = terrain
        p = params or {}

        # Rigid body state (world frame).
        self.mass = p.get("mass", 900.0)
        # Box-ish inertia, computed from dimensions.
        self.half_len = p.get("half_len", 1.7)
        self.half_w = p.get("half_w", 0.85)
        self.half_h = p.get("half_h", 0.45)
        self.inertia_body = self._box_inertia()

        self.position = list(p.get("position", (0.0, 1.2, 0.0)))
        self.velocity = [0.0, 0.0, 0.0]
        self.quat = V.q_identity()
        self.angular_velocity = [0.0, 0.0, 0.0]

        # Tunable global suspension/tyre parameters (live-editable in UI).
        self.query_kind = p.get("query_kind", "sphere")  # "ray" | "sphere"
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

    # ------------------------------------------------------------- helpers
    def wheel_anchor_world(self, wheel):
        return V.add(self.position, V.body_to_world(self.quat, wheel.anchor_body))

    def suspension_dir(self):
        """Suspension travel direction in world frame (body-down)."""
        return V.norm(V.body_to_world(self.quat, (0.0, -1.0, 0.0)))

    def _cast(self, origin, direction, length, radius):
        if self.query_kind == "ray":
            return self.terrain.raycast(origin, direction, length)
        return self.terrain.sphere_cast(origin, direction, length, radius)

    # ====================================================== physics step ==
    def step(self, dt, control):
        """Advance one fixed step.

        ``control``: dict with throttle [0..1], brake [0..1], steer [-1..1].
        Returns the diagnostics snapshot for this step.
        """
        self.warnings = []
        force_total = [0.0, 0.0, 0.0]
        torque_total = [0.0, 0.0, 0.0]

        # Gravity and quadratic aerodynamic drag act on the whole body:
        # F_drag = -c_drag * v * |v|.
        for i in range(3):
            force_total[i] += GRAVITY[i] * self.mass
        speed = V.length(self.velocity)
        if speed > 1e-6:
            drag = V.scale(self.velocity, -self.aero_drag * speed)
            force_total = V.add(force_total, drag)

        self.last_contact_count = 0
        susp_dir = self.suspension_dir()

        for w in self.wheels:
            self._update_wheel_contact(w, susp_dir, dt)
            if w.grounded:
                self.last_contact_count += 1
                fw, tw = self._wheel_forces(w, control, dt)
                w.f_suspension, w.f_tire = fw, tw
                force_total = V.add(force_total, fw)
                force_total = V.add(force_total, tw)
                r = V.sub(w.contact_point, self.position)
                torque_total = V.add(torque_total, V.cross(r, fw))
                torque_total = V.add(torque_total, V.cross(r, tw))
            else:
                w.f_suspension = (0.0, 0.0, 0.0)
                w.f_tire = (0.0, 0.0, 0.0)
                w.load = 0.0
                self._air_spin(w, control, dt)

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
        self._integrate_ground_spin(dt)
        return self.snapshot()

    # --------------------------------------------------------- contact
    def _update_wheel_contact(self, w, susp_dir, dt):
        """Perform this wheel's own fresh shape query for the step."""
        anchor = self.wheel_anchor_world(w)
        length = w.rest_length + w.max_travel
        hit = self._cast(anchor, susp_dir, length, w.radius)

        prev_grounded = w.grounded
        w.grounded = False
        w.contact_point = None
        w.contact_normal = None
        w.contact_distance = None
        w.query_kind = self.query_kind

        if hit is None:
            # No ground within the full travel: the wheel relaxes to full
            # extension on its suspension time constant, never "remembers"
            # the previous surface as support.
            rate = -w.compression / max(w.tau, 1e-3)
            w.compression_vel = rate
            w.compression = max(0.0, w.compression + rate * dt)
            if prev_grounded:
                self.warnings.append("%s lost support (edge/air)" % w.name)
            return

        d = hit["distance"]
        # Quantise the root to a fixed precision: bisection termination order
        # differs by ~1e-12 across runtimes, and without rounding that tiny
        # geometry delta gets amplified through the damper/tyre branches.
        d = round(d, 9)
        # Compression is measured from the rest position.
        compression = V.clamp(w.rest_length - d, 0.0, w.max_travel)
        # Grounded only when the wheel actually reaches its contact.
        if d <= w.rest_length + w.max_travel:
            w.grounded = True
            w.contact_point = hit["point"]
            w.contact_normal = hit["normal"]
            w.contact_distance = d
            w.compression_vel = (compression - w.compression) / dt
            w.compression = compression
            if not prev_grounded:
                self.warnings.append("%s gained support" % w.name)
            if compression >= w.max_travel - 1e-4:
                self.warnings.append("%s suspension bottomed" % w.name)

    # --------------------------------------------------------- forces
    def _wheel_forces(self, w, control, dt):
        n = w.contact_normal
        sprung = self.mass / len(self.wheels)
        c, _zeta = w.effective_damping(sprung)

        # --- Suspension spring/damper along the travel direction.
        # compression_vel > 0 means the wheel is moving INTO the spring;
        # the damper must then push back harder, never cancel the spring.
        f_spring = w.spring_k * w.compression
        f_damper = c * w.compression_vel
        f_susp_mag = max(f_spring + f_damper, 0.0)
        f_susp = V.scale(n, f_susp_mag)

        # Contact-point velocity includes body translation + rotation lever.
        r = V.sub(w.contact_point, self.position)
        v_contact = V.add(self.velocity, V.cross(self.angular_velocity, r))

        # Wheel forward/lateral axes (steering rotates the tyre heading).
        target_steer = control.get("steer", 0.0) * self.max_steer if w.steered else 0.0
        w.steer_angle += (target_steer - w.steer_angle) * min(1.0, dt / 0.12)
        fwd = V.norm(V.body_to_world(self.quat,
                                     (cos(w.steer_angle), 0.0, sin(w.steer_angle))))
        lat = V.norm(V.cross(n, fwd))

        # Normal load available for friction (normal component of susp force).
        load = max(V.dot(f_susp, n), 0.0)
        w.load = load

        v_long = V.dot(v_contact, fwd)
        # ------------------------------------------------------------
        # Wheel rotational dynamics: drive/brake TORQUE spins the wheel,
        # the contact patch then converts slip into a longitudinal force.
        # Keys are never turned into chassis motion directly.
        # ------------------------------------------------------------
        inertia_w = self._wheel_inertia(w)
        if w.powered:
            n_powered = sum(1 for x in self.wheels if x.powered)
            t_drive = (control.get("throttle", 0.0) * self.engine_force
                       * w.radius / n_powered)
        else:
            t_drive = 0.0
        t_brake = control.get("brake", 0.0) * self.brake_force * w.radius
        # Brake torque always opposes the current spin direction.
        if w.omega > 0.0:
            t_brake = -t_brake
        elif w.omega < 0.0:
            t_brake = abs(t_brake)

        # Longitudinal slip ratio (kinematic), guarded near zero speed.
        rolling_speed = w.omega * w.radius
        denom = max(abs(v_long), abs(rolling_speed), 1.0)
        slip = (rolling_speed - v_long) / denom
        w.slip = slip

        # Friction-circle-limited longitudinal tyre force from slip.
        f_slip = self.tire_mu * load * V.clamp(slip / 0.15, -1.0, 1.0)
        # Rolling resistance always opposes the contact-point motion; near
        # standstill it becomes a small holding force so an undriven vehicle
        # settles to an exact stop instead of creeping on residual wheel spin.
        idle = control.get("throttle", 0.0) == 0.0 and t_brake == 0.0
        if idle and abs(v_long) < 0.6:
            f_roll = -V.clamp(v_long * 800.0,
                              -self.rolling_resistance * 8.0,
                              self.rolling_resistance * 8.0)
        else:
            f_roll = -self.rolling_resistance * (1.0 if v_long >= 0.0 else -1.0)
        f_long = V.clamp(f_slip + f_roll,
                         -self.tire_mu * load, self.tire_mu * load)

        # --- Lateral grip: cancel lateral contact velocity within mu limit.
        v_lat = V.dot(v_contact, lat)
        f_lat_target = -v_lat * self.mass * 6.0
        budget = max((self.tire_mu * load) ** 2 - f_long ** 2, 0.0)
        f_lat = V.clamp(f_lat_target, -sqrt(budget), sqrt(budget))
        f_tire = V.add(V.scale(fwd, f_long), V.scale(lat, f_lat))

        # Integrate wheel spin: drive/brake torque minus tyre reaction torque
        # (reaction opposes the longitudinal force the patch exerts).
        t_reaction = -f_long * w.radius
        w.omega += ((t_drive + t_brake + t_reaction) / inertia_w) * dt
        # With no driver torque the free-rolling state is v_contact/R; relax
        # toward it only while the patch carries load, so a barely-grounded
        # zero-load wheel keeps its spin instead of being snapped to zero.
        if t_drive == 0.0 and t_brake == 0.0 and load > 1.0:
            roll = v_long / w.radius
            w.omega += (roll - w.omega) * min(1.0, dt / 0.25)
        return f_susp, f_tire

    def _wheel_inertia(self, w):
        # Solid-disk approximation of one corner's unsprung mass.
        corner_mass = self.mass / len(self.wheels) * 0.12 + 8.0
        return 0.5 * corner_mass * w.radius * w.radius

    def _air_spin(self, w, control, dt):
        """Airborne wheel: free spin under drive/brake torque, no reaction."""
        inertia_w = self._wheel_inertia(w)
        if w.powered:
            n_powered = sum(1 for x in self.wheels if x.powered)
            t_drive = (control.get("throttle", 0.0) * self.engine_force
                       * w.radius * 0.4 / n_powered)
        else:
            t_drive = 0.0
        t_brake = control.get("brake", 0.0) * self.brake_force * w.radius
        if w.omega > 0.0:
            t_brake = -t_brake
        elif w.omega < 0.0:
            t_brake = abs(t_brake)
        w.omega += ((t_drive + t_brake) / inertia_w) * dt
        w.spin_air += w.omega * dt

    def _integrate_ground_spin(self, dt):
        """Grounded wheels: visual angle follows the true rolling relation."""
        for w in self.wheels:
            if not w.grounded:
                continue
            r = V.sub(w.contact_point, self.position)
            v_contact = V.add(self.velocity, V.cross(self.angular_velocity, r))
            fwd = V.norm(V.body_to_world(self.quat,
                                         (cos(w.steer_angle), 0.0,
                                          sin(w.steer_angle))))
            v_long = V.dot(v_contact, fwd)
            # omega is already integrated from torques; relax it toward the
            # exact no-slip rolling speed so small slip errors do not
            # accumulate into a visual mismatch with chassis motion.
            roll_omega = v_long / w.radius
            w.omega += (roll_omega - w.omega) * min(1.0, dt / 0.10)
            w.spin_ground += w.omega * dt

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
