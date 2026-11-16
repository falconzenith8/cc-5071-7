"""Tyre force and wheel-spin boundary.

Longitudinal slip drives a friction-circle-limited force, lateral grip consumes
the remaining budget, and wheel spin integrates drive/brake torque plus the
tyre reaction.  Forces are returned in world coordinates built from the
contact-frame axes supplied by the caller.

The arithmetic order matches the pre-refactor engine exactly; the shared raw
records in ``data/evidence/`` are compared field-for-field.
"""

from math import sqrt

from .. import math3d as V


class TireParameters:
    __slots__ = ("mu", "engine_force", "brake_force", "rolling_resistance")

    def __init__(self, mu=1.1, engine_force=3200.0, brake_force=6000.0,
                 rolling_resistance=26.0):
        self.mu = mu
        self.engine_force = engine_force
        self.brake_force = brake_force
        self.rolling_resistance = rolling_resistance


class TireResult:
    __slots__ = ("slip", "slip_force", "rolling_force", "longitudinal",
                 "lateral", "force", "drive_torque", "brake_torque",
                 "reaction_torque", "omega")

    def __init__(self, slip, slip_force, rolling_force, longitudinal, lateral,
                 force, drive_torque, brake_torque, reaction_torque, omega):
        self.slip = slip
        self.slip_force = slip_force
        self.rolling_force = rolling_force
        self.longitudinal = longitudinal
        self.lateral = lateral
        self.force = force
        self.drive_torque = drive_torque
        self.brake_torque = brake_torque
        self.reaction_torque = reaction_torque
        self.omega = omega


def update_steer(steer_angle, target_steer, dt):
    """First-order steering actuator response."""
    return steer_angle + (target_steer - steer_angle) * min(1.0, dt / 0.12)


def brake_torque(params, control, radius, omega):
    t_brake = control.get("brake", 0.0) * params.brake_force * radius
    # Brake torque always opposes the current spin direction.
    if omega > 0.0:
        t_brake = -t_brake
    elif omega < 0.0:
        t_brake = abs(t_brake)
    return t_brake


def drive_torque(params, control, radius, powered, powered_count, scale=1.0):
    if not powered:
        return 0.0
    return (control.get("throttle", 0.0) * params.engine_force
            * radius * scale / powered_count)


def evaluate_tire(params, control, load, v_long, v_lat, forward, lateral,
                  omega, radius, inertia, body_mass, powered, powered_count,
                  dt):
    """Return tyre forces (world) and the updated wheel spin.

    ``forward``/``lateral`` are world-space unit axes; ``v_long``/``v_lat`` are
    the corresponding contact-frame velocity components.
    """
    t_drive = drive_torque(params, control, radius, powered, powered_count)
    t_brake = brake_torque(params, control, radius, omega)

    # Longitudinal slip ratio (kinematic), guarded near zero speed.
    rolling_speed = omega * radius
    denom = max(abs(v_long), abs(rolling_speed), 1.0)
    slip = (rolling_speed - v_long) / denom

    f_slip = params.mu * load * V.clamp(slip / 0.15, -1.0, 1.0)
    # Rolling resistance always opposes contact motion; near standstill it
    # becomes a small holding force so an undriven vehicle settles to a stop.
    idle = control.get("throttle", 0.0) == 0.0 and t_brake == 0.0
    if idle and abs(v_long) < 0.6:
        f_roll = -V.clamp(v_long * 800.0,
                          -params.rolling_resistance * 8.0,
                          params.rolling_resistance * 8.0)
    else:
        f_roll = -params.rolling_resistance * (1.0 if v_long >= 0.0 else -1.0)
    f_long = V.clamp(f_slip + f_roll,
                     -params.mu * load, params.mu * load)

    # Lateral grip: cancel lateral contact velocity within the friction budget.
    f_lat_target = -v_lat * body_mass * 6.0
    budget = max((params.mu * load) ** 2 - f_long ** 2, 0.0)
    f_lat = V.clamp(f_lat_target, -sqrt(budget), sqrt(budget))
    f_tire = V.add(V.scale(forward, f_long), V.scale(lateral, f_lat))

    # Drive + brake + tyre reaction, then the low-load free-roll relaxation.
    t_reaction = -f_long * radius
    omega = omega + ((t_drive + t_brake + t_reaction) / inertia) * dt
    if t_drive == 0.0 and t_brake == 0.0 and load > 1.0:
        roll = v_long / radius
        omega += (roll - omega) * min(1.0, dt / 0.25)
    return TireResult(slip, f_slip, f_roll, f_long, f_lat, f_tire, t_drive,
                      t_brake, t_reaction, omega)


def free_spin_omega(params, control, radius, inertia, powered, powered_count,
                    omega, dt):
    """Update an airborne wheel's spin under drive/brake torque only."""
    t_drive = drive_torque(params, control, radius, powered, powered_count,
                           scale=0.4)
    t_brake = brake_torque(params, control, radius, omega)
    return omega + ((t_drive + t_brake) / inertia) * dt


def relax_to_rolling(omega, v_long, radius, dt):
    """Grounded free-rolling relaxation toward the true rolling relation."""
    roll_omega = v_long / radius
    return omega + (roll_omega - omega) * min(1.0, dt / 0.10)
