"""Compatibility adapters for the pre-refactor wheel-contact entry points.

The refactor moved support queries, suspension and tyre evaluation into
``engine.wheel_ground``.  Callers that still drive a single wheel through the
old ``Vehicle`` methods are kept working here, so the vehicle-contact caller
does not need to know the module layout.

Each adapter delegates to the same wheel-ground primitives the live step uses;
it never re-implements force rules and never lets a query hit become support
beyond the finite reach.
"""

from .. import math3d as V


class LegacyVehicleAdapter:
    """Adapter that preserves the old per-wheel method names on ``vehicle``."""

    def __init__(self, vehicle):
        self.vehicle = vehicle

    def _wheel(self, name):
        for w in self.vehicle.wheels:
            if w.name == name:
                return w
        raise KeyError("no wheel named %r" % (name,))

    def queried_contact(self, name):
        """Run one fresh support query for a wheel and return its contact.

        Mirrors the old ``_update_wheel_contact`` return contract: a dict with
        ``grounded``/``contact_point``/``contact_normal``/``contact_distance``,
        or ``grounded=False`` when nothing supports the wheel within reach.
        """
        veh = self.vehicle
        w = self._wheel(name)
        anchor = veh.wheel_anchor_world(w)
        direction = veh.suspension_dir()
        probe = veh.contact.query.probe(
            anchor, direction, w.rest_length + w.max_travel, w.radius)
        if probe is None:
            return {"grounded": False, "contact_point": None,
                    "contact_normal": None, "contact_distance": None}
        distance = veh.contact.query.quantize_distance(probe.distance)
        if not veh.contact.query.within_reach(distance, w.rest_length,
                                              w.max_travel):
            return {"grounded": False, "contact_point": None,
                    "contact_normal": None, "contact_distance": None}
        return {"grounded": True, "contact_point": probe.point,
                "contact_normal": probe.normal, "contact_distance": distance}

    def wheel_force(self, name, control=None, dt=None):
        """Return the wheel's last force pair ``(f_suspension, f_tire)``."""
        w = self._wheel(name)
        return w.f_suspension, w.f_tire

    def wheel_spin(self, name):
        """Return ``(spin_ground, spin_air, spin_total)`` for a wheel."""
        w = self._wheel(name)
        return w.spin_ground, w.spin_air, w.spin_total

    def contact_velocity(self, name):
        """Contact-point velocity in world coordinates (single lever term)."""
        veh = self.vehicle
        w = self._wheel(name)
        if w.contact_point is None:
            return None
        offset = V.sub(w.contact_point, veh.position)
        return V.add(veh.velocity, V.cross(veh.angular_velocity, offset))
