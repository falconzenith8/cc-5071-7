"""Deterministic driving-input profiles for the three validation scenarios.

Each profile is a plain function of time so the same manoeuvre can be
recorded as a replay in headless tests and replayed in the contact window.
"""


def smooth_ramp(t, t0, t1, target):
    """Smooth (cosine) ramp from 0 to target between t0 and t1."""
    import math
    if t <= t0:
        return 0.0
    if t >= t1:
        return target
    u = (t - t0) / (t1 - t0)
    return target * 0.5 * (1.0 - math.cos(math.pi * u))


def slope_profile(t):
    """Gentle throttle up the 12-degree grade, lift off over the crest."""
    throttle = smooth_ramp(t, 0.0, 1.5, 0.55)
    if t > 12.0:
        throttle *= max(0.0, 1.0 - (t - 12.0) / 2.0)
    return {"throttle": round(throttle, 4), "brake": 0.0, "steer": 0.0}


def bump_profile(t):
    """Slow steady drive across the humps with a small steering weave."""
    throttle = smooth_ramp(t, 0.0, 1.5, 0.32)
    steer = 0.0
    if 8.0 < t < 20.0:
        import math
        steer = 0.22 * math.sin(0.7 * (t - 8.0))
    return {"throttle": round(throttle, 4), "brake": 0.0,
            "steer": round(steer, 4)}


def edge_profile(t):
    """Climb the ramp onto the plateau, then steer the right wheels off edge."""
    throttle = smooth_ramp(t, 0.0, 1.0, 0.35)
    steer = 0.0
    if t > 9.0:
        throttle = smooth_ramp(t, 9.0, 10.0, 0.22)
    if 11.0 < t < 14.5:         # turn toward +z: right side crosses the edge
        steer = 0.8
    elif t >= 14.5:
        steer = 0.0
    if t > 16.0:
        throttle = 0.0
    return {"throttle": round(throttle, 4),
            "brake": 0.3 if t > 17.0 else 0.0,
            "steer": round(steer, 4)}


PROFILES = {
    "slope": slope_profile,
    "bump": bump_profile,
    "edge": edge_profile,
}

DURATIONS = {"slope": 22.0, "bump": 24.0, "edge": 20.0}
