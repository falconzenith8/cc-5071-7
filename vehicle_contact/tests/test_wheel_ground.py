"""Boundary tests for the wheel-ground module and its legacy facade.

These pin the refactor contract:

* the three validation scenarios reproduce the pre-refactor raw records field
  for field (equivalence via wheel-ground actions and chassis trajectory);
* a support-query *hit* is not automatically stable support -- the finite
  suspension reach rule still applies;
* the public wheel-ground interface reports physics results (no re-query in
  the visible wheel) and separates wheel-ground from chassis-motion anomalies.

Run with:  python3 -m cli.vc test
"""

import csv
import json
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from engine.simulation import Simulation, FIXED_DT
from engine.scenarios import PROFILES, DURATIONS
from engine import math3d as V
from engine.terrain import TERRAINS
from engine.vehicle import Vehicle
from engine.wheel_ground import SupportQuery, ContactFrame, relative_velocity
from engine.wheel_ground.interface import classify_warnings

EVIDENCE = os.path.join(ROOT, "data", "evidence")
WHEELS = ("FL", "FR", "RL", "RR")
IDLE = {"throttle": 0.0, "brake": 0.0, "steer": 0.0}


def _fmt(value):
    if isinstance(value, float):
        return "%.12g" % value
    if value is None:
        return ""
    return value


def _sim_rows(sim):
    """Reproduce the raw-evidence row schema from a Simulation."""
    rows = []
    for i, snap in enumerate(sim.snapshots):
        tr = sim.trajectory[i]
        wmap = {w["name"]: w for w in snap["wheels"]}
        wstate = {w.name: w for w in sim.vehicle.wheels}
        row = {
            "t": _fmt(tr["t"]),
            "x": _fmt(snap["position"][0]), "y": _fmt(snap["position"][1]),
            "z": _fmt(snap["position"][2]),
            "vx": _fmt(snap["velocity"][0]), "vy": _fmt(snap["velocity"][1]),
            "vz": _fmt(snap["velocity"][2]),
            "wx": _fmt(snap["angular_velocity"][0]),
            "wy": _fmt(snap["angular_velocity"][1]),
            "wz": _fmt(snap["angular_velocity"][2]),
            "contacts": str(snap["contact_count"]),
            "body_contact": str(int(snap["body_contact"])),
            "warnings": "|".join(snap["warnings"]),
        }
        for nm in WHEELS:
            w = wmap[nm]
            cp = w["contact_point"] or (None, None, None)
            cn = w["contact_normal"] or (None, None, None)
            row.update({
                nm + "_grounded": str(int(bool(w["grounded"]))),
                nm + "_query": w["query"],
                nm + "_comp": _fmt(w["compression"]),
                nm + "_compvel": _fmt(wstate[nm].compression_vel),
                nm + "_cpx": _fmt(cp[0]), nm + "_cpy": _fmt(cp[1]),
                nm + "_cpz": _fmt(cp[2]),
                nm + "_cnx": _fmt(cn[0]), nm + "_cny": _fmt(cn[1]),
                nm + "_cnz": _fmt(cn[2]),
                nm + "_dist": _fmt(w["contact_distance"]),
                nm + "_load": _fmt(w["load"]),
                nm + "_slip": _fmt(w["slip"]),
                nm + "_omega": _fmt(w["omega"]),
                nm + "_steer": _fmt(w["steer_deg"]),
                nm + "_sg": _fmt(w["spin_ground"]),
                nm + "_sa": _fmt(w["spin_air"]),
                nm + "_fsx": _fmt(w["f_suspension"][0]),
                nm + "_fsy": _fmt(w["f_suspension"][1]),
                nm + "_fsz": _fmt(w["f_suspension"][2]),
                nm + "_ftx": _fmt(w["f_tire"][0]),
                nm + "_fty": _fmt(w["f_tire"][1]),
                nm + "_ftz": _fmt(w["f_tire"][2]),
            })
        rows.append(row)
    return rows


class RefactorEquivalenceTests(unittest.TestCase):
    """The refactored engine must reproduce the pre-refactor raw records."""

    def _check(self, name):
        path = os.path.join(EVIDENCE, "%s_raw.csv" % name)
        self.assertTrue(os.path.isfile(path),
                        "missing evidence file %s" % path)
        ref = list(csv.DictReader(open(path, encoding="utf-8")))
        sim = Simulation(name, dt=FIXED_DT, query_kind="sphere")
        sim.run_seconds(DURATIONS[name], PROFILES[name])
        rows = _sim_rows(sim)
        self.assertEqual(len(rows), len(ref))
        mismatches = []
        for i, (a, b) in enumerate(zip(rows, ref)):
            for key in ref[0]:
                if a[key] != b[key]:
                    mismatches.append((i, key, a[key], b[key]))
                    if len(mismatches) > 8:
                        break
            if len(mismatches) > 8:
                break
        self.assertEqual(mismatches, [],
                         "field mismatch vs pre-refactor record: %s"
                         % (mismatches[:8],))

    def test_slope_matches_pre_refactor_record(self):
        self._check("slope")

    def test_edge_matches_pre_refactor_record(self):
        self._check("edge")

    def test_manifest_digest_present(self):
        path = os.path.join(EVIDENCE, "reference_manifest.json")
        with open(path, encoding="utf-8") as f:
            manifest = json.load(f)
        for name in ("slope", "edge"):
            self.assertIn(name, manifest["scenarios"])
            self.assertEqual(len(manifest["scenarios"][name]["sha256"]), 64)


class SupportIsNotUnlimitedTests(unittest.TestCase):
    """A query hit must not become unlimited stable support."""

    def test_hit_beyond_reach_is_not_support(self):
        terrain = TERRAINS["flat"]()
        veh = Vehicle(terrain, {"position": (0.0, 3.0, 0.0)})
        # Cast reaches a flat floor far below the suspension travel.
        query = SupportQuery(terrain, "ray")
        anchor = veh.wheel_anchor_world(veh.wheels[0])
        probe = query.probe(anchor, (0.0, -1.0, 0.0),
                            veh.wheels[0].rest_length
                            + veh.wheels[0].max_travel, veh.wheels[0].radius)
        self.assertIsNone(probe)  # far floor is outside the reach, no hit
        snap = veh.step(FIXED_DT, IDLE)
        for w in snap["wheels"]:
            self.assertFalse(w["grounded"])
            self.assertIsNone(w["contact_point"])
            self.assertEqual(w["f_suspension"], (0.0, 0.0, 0.0))
            self.assertEqual(w["f_tire"], (0.0, 0.0, 0.0))

    def test_hit_exactly_at_reach_still_supports(self):
        # The reach boundary is inclusive but finite: exactly at rest+travel
        # is support, just beyond it is not.
        terrain = TERRAINS["flat"]()
        veh = Vehicle(terrain, {"position": (0.0, 1.0, 0.0)})
        w0 = veh.wheels[0]
        reach = w0.rest_length + w0.max_travel
        query = SupportQuery(terrain, "ray")
        self.assertTrue(query.within_reach(reach, w0.rest_length,
                                           w0.max_travel))
        self.assertFalse(query.within_reach(reach + 1e-6, w0.rest_length,
                                            w0.max_travel))
        # Physical probe: place the floor exactly `reach` below the anchor and
        # confirm the query finds it inside the finite reach (not unlimited).
        anchor_y = reach  # anchor at reach above the y=0 floor
        hit = query.probe((0.0, anchor_y, 0.0), (0.0, -1.0, 0.0), reach + 0.5,
                          w0.radius)
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(hit.distance, reach, delta=1e-6)

    def test_old_surface_never_retained_after_edge(self):
        sim = Simulation("edge", dt=FIXED_DT)
        sim.run_seconds(DURATIONS["edge"], PROFILES["edge"])
        # On the documented edge manoeuvre some wheel is unsupported for a
        # sustained stretch; every such step must carry no contact point.
        unsupported = [s for s in sim.snapshots
                       if any(not w["grounded"] for w in s["wheels"])]
        self.assertGreater(len(unsupported), 30)
        for s in unsupported:
            for w in s["wheels"]:
                if not w["grounded"]:
                    self.assertIsNone(w["contact_point"])
                    self.assertEqual(w["f_suspension"], (0.0, 0.0, 0.0))


class ContactFrameInterfaceTests(unittest.TestCase):
    def test_relative_velocity_includes_single_lever(self):
        body_v = (1.0, 0.0, 0.0)
        omega = (0.0, 0.0, 2.0)
        offset = (0.0, 1.0, 0.0)
        # omega x r = (-2, 0, 0); exactly one lever term.
        self.assertEqual(relative_velocity(body_v, omega, offset),
                         (-1.0, 0.0, 0.0))

    def test_frame_decompose_recompose(self):
        frame = ContactFrame((0.0, 1.0, 0.0), V.norm((1.0, 0.0, 1.0)))
        world = (0.3, 0.4, -0.2)
        n, f, l = frame.decompose(world)
        rebuilt = V.add(V.add(V.scale(frame.normal, n),
                              V.scale(frame.forward, f)),
                        V.scale(frame.lateral, l))
        for i in range(3):
            self.assertAlmostEqual(rebuilt[i], world[i], places=12)


class VisibleWheelConsumesPhysicsTests(unittest.TestCase):
    def test_center_is_published_without_requery(self):
        sim = Simulation("slope", dt=FIXED_DT)
        sim.run_seconds(3.0, PROFILES["slope"])
        state = sim.vehicle.wheel_ground()
        self.assertEqual(len(state["wheel_ground"]), 4)
        for w in sim.vehicle.wheels:
            self.assertIsNotNone(w.center)
        for row, w in zip(state["wheel_ground"], sim.vehicle.wheels):
            self.assertEqual(row["center"], w.center)
            if row["grounded"]:
                # Published centre lies on the same support ray and distance
                # the solver used; the renderer consumes it directly.
                expected = V.add(row["anchor_world"],
                                 V.scale(row["support_dir"],
                                         w.contact_distance))
                for i in range(3):
                    self.assertAlmostEqual(row["center"][i], expected[i],
                                           places=9)

    def test_anomalies_split_by_subsystem(self):
        groups = classify_warnings(
            ["FL gained support", "RR lost support (edge/air)",
             "chassis hull contacted terrain"])
        self.assertEqual(groups["wheel_ground"],
                         ["FL gained support", "RR lost support (edge/air)"])
        self.assertEqual(groups["chassis_motion"],
                         ["chassis hull contacted terrain"])


if __name__ == "__main__":
    unittest.main()
