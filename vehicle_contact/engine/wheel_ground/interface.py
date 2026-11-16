"""Public vehicle wheel-ground interface.

Callers (CLI diagnostics, verification server, contact window) consume this
interface instead of poking at wheel internals.  It returns *physics results*:
per-wheel support state, contact geometry and the wheel centre the renderer
should draw.  The renderer never re-runs a query or re-derives suspension
geometry.

Anomalies are grouped by the subsystem they belong to, so the contact window
can label wheel-ground contact versus chassis-motion problems distinctly.
"""

WHEEL_GROUND_PREFIXES = (
    "gained support",
    "lost support",
    "suspension bottomed",
    "all wheels airborne",
)
CHASSIS_PREFIXES = (
    "chassis hull contacted terrain",
)


def classify_warnings(warnings):
    """Split warning strings into wheel-ground and chassis-motion groups."""
    groups = {"wheel_ground": [], "chassis_motion": []}
    for text in warnings:
        if any(text.startswith(prefix) for prefix in CHASSIS_PREFIXES):
            groups["chassis_motion"].append(text)
        elif any(text.startswith(prefix) for prefix in WHEEL_GROUND_PREFIXES):
            groups["wheel_ground"].append(text)
        else:
            groups["wheel_ground"].append(text)
    return groups


def wheel_ground_state(vehicle):
    """Return the public interface snapshot for a vehicle.

    ``wheel_center`` is the already-solved physical wheel position (the same
    value the solver used), so a visible wheel can draw it directly.
    """
    wheels = []
    for w in vehicle.wheels:
        wheels.append({
            "name": w.name,
            "grounded": w.grounded,
            "query": w.query_kind,
            "center": w.center,
            "anchor_world": w.anchor_world,
            "support_dir": w.support_dir,
            "contact_point": w.contact_point,
            "contact_normal": w.contact_normal,
            "contact_distance": w.contact_distance,
            "compression": w.compression,
            "compression_vel": w.compression_vel,
            "max_travel": w.max_travel,
            "travel_ratio": w.compression / max(w.max_travel, 1e-9),
            "load": w.load,
            "slip": w.slip,
            "omega": w.omega,
            "steer_deg": getattr(w, "steer_deg", None),
            "spin_ground": w.spin_ground,
            "spin_air": w.spin_air,
            "f_suspension": w.f_suspension,
            "f_tire": w.f_tire,
        })
    return {
        "contact_count": vehicle.last_contact_count,
        "body_contact": vehicle.body_contact,
        "wheel_ground": wheels,
        "anomalies": classify_warnings(vehicle.warnings),
    }
