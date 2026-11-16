"""Explicit support query boundary.

The shape query answers *where* and *in what direction* the terrain is; it does
not decide support on its own.  A hit is only a candidate.  Support requires
the hit distance to lie within the finite suspension reach
``rest_length + max_travel``.  This keeps the rule that a query hit can never
become an unlimited stable support: ground farther away than the suspension can
reach is not support.
"""

from .. import math3d as V


class SupportProbe:
    """One shape-query result, in world coordinates."""

    __slots__ = ("distance", "point", "normal", "centre")

    def __init__(self, distance, point, normal, centre=None):
        self.distance = distance
        self.point = point
        self.normal = normal
        self.centre = centre


class SupportQuery:
    """Ray / sphere-cast query bound to a terrain."""

    KINDS = ("ray", "sphere")

    def __init__(self, terrain, kind="sphere"):
        if kind not in self.KINDS:
            raise ValueError("unknown support query kind: %r" % (kind,))
        self.terrain = terrain
        self.kind = kind

    def probe(self, origin, direction, max_distance, radius):
        """Cast the configured shape; ``None`` when nothing is within reach."""
        if self.kind == "ray":
            hit = self.terrain.raycast(origin, direction, max_distance)
        else:
            hit = self.terrain.sphere_cast(origin, direction, max_distance,
                                           radius)
        if hit is None:
            return None
        return SupportProbe(hit["distance"], hit["point"], hit["normal"],
                            hit.get("centre"))

    @staticmethod
    def quantize_distance(distance):
        """Round the root to fixed precision.

        Bisection termination order differs by ~1e-12 across runtimes; without
        rounding that geometric delta is amplified through the damper / tyre
        branches and breaks replay equality.
        """
        return round(distance, 9)

    @staticmethod
    def within_reach(distance, rest_length, max_travel):
        """Finite-reach support rule: a hit only supports within travel."""
        return distance <= rest_length + max_travel
