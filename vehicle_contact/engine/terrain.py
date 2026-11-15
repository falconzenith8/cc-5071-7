"""Procedural analytic heightfield terrain with explicit shape queries.

Two query primitives are implemented and both return the contact point and
the surface normal, never just a height scalar:

* :meth:`Heightfield.raycast`     - segment/ray intersection
* :meth:`Heightfield.sphere_cast` - swept sphere (shape query) used to model
  a tyre of finite radius so a wheel beside a ledge is *not* supported by the
  old higher ground once it has crossed the edge.

The heightfield is the pointwise maximum of named regions (flat base, a
constant-grade slope, an undulating patch and a raised platform with a hard
edge).  Taking the maximum makes every region merge into one continuous,
walkable surface while preserving the vertical cliff of the platform.
"""

from math import sin, cos, pi, sqrt
from .math3d import sub, scale, add, norm, dot, clamp, lerp


class Heightfield:
    def __init__(self, regions=None):
        # Each region: dict(kind, params...).  None => flat base only.
        self.regions = regions or []

    # ------------------------------------------------------------- geometry
    def region_height(self, kind, p, r):
        x, _, z = p
        if kind == "base":
            return r.get("y", 0.0)
        if kind == "slope":
            if not (r["x0"] <= x <= r["x1"]):
                return None
            t = (x - r["x0"]) / (r["x1"] - r["x0"])
            return r["y0"] + t * (r["y1"] - r["y0"])
        if kind == "bump":
            dx = (x - r["cx"]) / r["sx"]
            dz = (z - r["cz"]) / r["sz"]
            d2 = dx * dx + dz * dz
            if d2 > 1.0:
                return None
            return r["base_y"] + r["amp"] * (0.5 + 0.5 * cos(pi * sqrt(d2)))
        if kind == "platform":
            if not (r["x0"] <= x <= r["x1"] and r["z0"] <= z <= r["z1"]):
                return None
            return r["top_y"]
        raise ValueError("unknown region kind: %s" % kind)

    def height(self, x, z):
        """Pointwise maximum of every region active at (x, z)."""
        h = 0.0
        for r in self.regions:
            rh = self.region_height(r["kind"], (x, 0.0, z), r)
            if rh is not None and rh > h:
                h = rh
        return h

    def normal(self, x, z):
        """Geometric surface normal from the local gradient.

        A small central difference gives the normal of whichever region is
        the active (maximum) surface.  On a vertical cliff the raw gradient
        degenerates to a near-horizontal vector, so inside an edge band the
        normal is blended back toward vertical: the wheel then loses support
        cleanly instead of receiving a sideways kick, while the heightfield
        itself still reports the true (lower) floor height.
        """
        e = 0.05
        hx = (self.height(x + e, z) - self.height(x - e, z)) / (2 * e)
        hz = (self.height(x, z + e) - self.height(x, z - e)) / (2 * e)
        n = norm((-hx, 1.0, -hz))
        steep = max(abs(hx), abs(hz))
        if steep > 1.5:
            band = min(1.0, (steep - 1.5) / 2.5)
            n = norm(lerp(n, (0.0, 1.0, 0.0), band))
        return n

    # ------------------------------------------------------------ queries
    def raycast(self, origin, direction, max_dist):
        """Intersect a ray with the height surface.

        Returns ``None`` when nothing is hit, otherwise a dict with
        ``distance``, ``point`` and ``normal``.  The root is bracketed by
        scanning the signed clearance ``g(t) = ray_y(t) - h(x,z)`` and then
        refined with bisection, so it works on vertical cliff faces too.
        """
        d = norm(direction)
        if d[1] >= -1e-9:
            return None  # only downward rays can meet a heightfield
        steps = 48
        prev_t = 0.0
        prev_g = origin[1] - self.height(origin[0], origin[2])
        hit_t = None
        for i in range(1, steps + 1):
            t = max_dist * i / steps
            pt = add(origin, scale(d, t))
            g = pt[1] - self.height(pt[0], pt[2])
            if g <= 0.0 <= prev_g:
                lo, hi = prev_t, t
                for _ in range(40):
                    mid = 0.5 * (lo + hi)
                    mp = add(origin, scale(d, mid))
                    mg = mp[1] - self.height(mp[0], mp[2])
                    if mg > 0.0:
                        lo = mid
                    else:
                        hi = mid
                hit_t = 0.5 * (lo + hi)
                break
            prev_t, prev_g = t, g
        if hit_t is None:
            return None
        point = add(origin, scale(d, hit_t))
        return {
            "distance": hit_t,
            "point": point,
            "normal": self.normal(point[0], point[2]),
        }

    def sphere_cast(self, origin, direction, max_dist, radius):
        """Swept-sphere query: find where a sphere first touches the surface.

        Solves ``g(t) = ray_y(t) - h(proj) - radius * n_y(proj) = 0`` where
        ``proj`` is the sphere centre projected vertically.  The contact
        normal is the terrain normal at the actual contact point, and the
        reported contact point is pushed onto the sphere skin.  Because the
        sphere has finite radius, a wheel hanging beside (not over) a ledge
        only sees the lower floor: the ledge can no longer prop it up.
        """
        d = norm(direction)
        if d[1] >= -1e-9:
            return None

        def clearance(t):
            c = add(origin, scale(d, t))
            n = self.normal(c[0], c[2])
            return c[1] - self.height(c[0], c[2]) - radius * n[1], c, n

        steps = 48
        prev_t = 0.0
        prev_g, _, _ = clearance(0.0)
        hit_t = None
        for i in range(1, steps + 1):
            t = max_dist * i / steps
            g, _, _ = clearance(t)
            if g <= 0.0 <= prev_g:
                lo, hi = prev_t, t
                for _ in range(40):
                    mid = 0.5 * (lo + hi)
                    mg, _, _ = clearance(mid)
                    if mg > 0.0:
                        lo = mid
                    else:
                        hi = mid
                hit_t = 0.5 * (lo + hi)
                break
            prev_t, prev_g = t, g
        if hit_t is None:
            return None
        _, centre, n = clearance(hit_t)
        # Quantise the horizontal coordinate before evaluating the surface
        # height: a sphere centre landing within ~1e-15 of a region edge can
        # otherwise be classified on different sides across runtimes.
        cx = round(centre[0], 9)
        cz = round(centre[2], 9)
        point = (cx, self.height(cx, cz), cz)
        return {
            "distance": hit_t,
            "point": point,
            "normal": n,
            "centre": centre,
        }


# ------------------------------------------------------------- scene presets
def base_terrain():
    return Heightfield(regions=[{"kind": "base", "y": 0.0}])


def slope_terrain():
    """Flat run-up, 12-degree grade, crest and matching descent.

    The up-slope and down-slope meet at a common height so the surface is
    continuous (no vertical cliff at the crest); the wheel shape queries see
    a smooth transition instead of a drop-off.
    """
    return Heightfield(regions=[
        {"kind": "base", "y": 0.0},
        {"kind": "slope", "x0": 12.0, "x1": 42.0, "y0": 0.0, "y1": 6.4},
        {"kind": "slope", "x0": 42.0, "x1": 72.0, "y0": 6.4, "y1": 0.0},
    ])


def bump_terrain():
    """Undulating cosine humps on otherwise flat ground.

    Amplitudes stay below roughly one wheel radius so a correctly damped
    suspension keeps at least partial contact; losing contact over a crest
    is still possible and must be reported, but the chassis hull must not
    slam into the terrain.
    """
    return Heightfield(regions=[
        {"kind": "base", "y": 0.0},
        {"kind": "bump", "cx": 12.0, "cz": 0.0, "sx": 6.0, "sz": 6.0,
         "amp": 0.38, "base_y": 0.0},
        {"kind": "bump", "cx": 24.0, "cz": 1.6, "sx": 4.5, "sz": 4.5,
         "amp": 0.30, "base_y": 0.0},
        {"kind": "bump", "cx": 35.0, "cz": -1.2, "sx": 5.0, "sz": 5.0,
         "amp": 0.34, "base_y": 0.0},
    ])


def edge_terrain():
    """Drivable ramp onto a flat plateau that ends in a hard lateral edge.

    The 12-degree approach ramp lets all four wheels climb onto the plateau
    without a vertical-face collision.  Once on top, steering toward +z takes
    the right-hand wheels past the plateau side: the floor is 1.0 m below,
    farther than the suspension travel, so those wheels must report
    unsupported instead of keeping the old plateau surface as support.
    """
    return Heightfield(regions=[
        {"kind": "base", "y": 0.0},
        {"kind": "slope", "x0": 4.0, "x1": 9.0, "y0": 0.0, "y1": 1.0},
        {"kind": "platform", "x0": 9.0, "x1": 26.0,
         "z0": -4.0, "z1": 3.0, "top_y": 1.0},
    ])


TERRAINS = {
    "flat": base_terrain,
    "slope": slope_terrain,
    "bump": bump_terrain,
    "edge": edge_terrain,
}
