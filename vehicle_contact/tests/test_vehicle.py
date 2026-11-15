"""Unit/integration tests for the contact-driven vehicle engine.

Run with:  python3 -m cli.vc test
"""

import math
import os
import sys
import unittest

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)

from engine.simulation import (Simulation, run_replay, FIXED_DT,
                               build_scenario)
from engine.terrain import TERRAINS, Heightfield
from engine.vehicle import Vehicle
from engine.scenarios import PROFILES, DURATIONS
from engine import math3d as V


IDLE = {"throttle": 0.0, "brake": 0.0, "steer": 0.0}


class TerrainQueryTests(unittest.TestCase):
    def setUp(self):
        self.terrain = TERRAINS["edge"]()

    def test_flat_raycast_normal(self):
        hit = self.terrain.raycast((0.0, 2.0, 0.0), (0.0, -1.0, 0.0), 5.0)
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(hit["point"][1], 0.0, places=5)
        self.assertAlmostEqual(hit["normal"][1], 1.0, places=4)

    def test_platform_top_normal(self):
        hit = self.terrain.raycast((12.0, 2.0, 0.0), (0.0, -1.0, 0.0), 5.0)
        self.assertIsNotNone(hit)
        self.assertAlmostEqual(hit["point"][1], 1.0, places=4)

    def test_ray_miss_when_pointing_up(self):
        self.assertIsNone(
            self.terrain.raycast((0.0, 1.0, 0.0), (0.0, 1.0, 0.0), 3.0))

    def test_sphere_cast_does_not_clip_ledge_side(self):
        # Sphere centre beside (z beyond) the platform edge must hit the lower
        # floor, never the platform top, regardless of how close it gets.
        hit = self.terrain.sphere_cast((12.0, 1.6, 3.6),
                                       (0.0, -1.0, 0.0), 3.0, 0.32)
        self.assertIsNotNone(hit)
        self.assertLess(hit["centre"][1], 0.5)
        self.assertAlmostEqual(hit["point"][1], 0.0, places=3)

    def test_heightfield_continuous_at_slope_join(self):
        t = TERRAINS["slope"]()
        self.assertAlmostEqual(t.height(42.0 - 1e-6, 0.0),
                               t.height(42.0 + 1e-6, 0.0), delta=1e-3)


class SuspensionTests(unittest.TestCase):
    def test_static_settle_matches_spring_equilibrium(self):
        sim = Simulation("flat", dt=FIXED_DT)
        for _ in range(int(3.0 / FIXED_DT)):
            snap = sim.update(IDLE, record=False)
        w = snap["wheels"][0]
        expected = (sim.vehicle.mass * 9.81
                    / (4 * sim.vehicle.wheels[0].spring_k))
        self.assertAlmostEqual(w["compression"], expected, delta=2e-3)
        self.assertEqual(snap["contact_count"], 4)
        self.assertAlmostEqual(snap["velocity"][1], 0.0, delta=1e-3)

    def test_time_constant_changes_damping_ratio(self):
        sim = Simulation("flat", dt=FIXED_DT)
        v = sim.vehicle
        sprung = v.mass / 4
        soft = v.wheels[0].effective_damping(sprung)
        v.set_suspension_params({"tau": 0.35})
        stiff = v.wheels[0].effective_damping(sprung)
        # Larger tau -> lower damping ratio.
        self.assertLess(stiff[1], soft[1])
        self.assertGreater(stiff[1], 0.0)

    def test_max_travel_clamps_compression(self):
        sim = Simulation("flat", dt=FIXED_DT)
        # Drop the vehicle hard: compression must never exceed max travel.
        sim.vehicle.position[1] = 4.0
        worst = 0.0
        for _ in range(int(3.0 / FIXED_DT)):
            snap = sim.update(IDLE, record=False)
            worst = max(worst, max(w["travel_ratio"] for w in snap["wheels"]))
        self.assertLessEqual(worst, 1.0 + 1e-9)


class ContactSupportTests(unittest.TestCase):
    def test_edge_crossing_drops_support_immediately(self):
        sim = Simulation("edge", dt=FIXED_DT)
        partial = 0
        stale = 0
        for _ in range(int(DURATIONS["edge"] / FIXED_DT)):
            snap = sim.update(PROFILES["edge"](sim.time), record=False)
            if 0 < snap["contact_count"] < 4:
                partial += 1
                for w in snap["wheels"]:
                    # A grounded wheel's contact point must lie on the sphere
                    # skin at terrain height + radius along the normal; an
                    # unsupported wheel carries no point at all.
                    if w["grounded"]:
                        cp = w["contact_point"]
                        n = w["contact_normal"]
                        ground = sim.vehicle.terrain.height(cp[0], cp[2])
                        # Contact point sits on the terrain surface (sphere
                        # queries report the lowest skin point at ground level).
                        self.assertLessEqual(abs(cp[1] - ground), 0.03)
                        # Normal must point upward; on slopes it tilts but
                        # never sideways/downward.
                        self.assertGreater(n[1], 0.7)
                    else:
                        self.assertIsNone(w["contact_point"])
                        stale += 1
        self.assertGreater(partial, 30)
        self.assertGreater(stale, 30)

    def test_airborne_wheel_has_no_force(self):
        terrain, params = build_scenario("edge")
        veh = Vehicle(terrain, params)
        # Force every wheel beyond the platform edge.
        veh.position = [15.0, 3.0, 6.0]
        snap = veh.step(FIXED_DT, IDLE)
        self.assertEqual(snap["contact_count"], 0)
        for w in snap["wheels"]:
            self.assertFalse(w["grounded"])
            self.assertEqual(w["f_suspension"], (0.0, 0.0, 0.0))
            self.assertEqual(w["f_tire"], (0.0, 0.0, 0.0))


class RollingMotionTests(unittest.TestCase):
    def test_throttle_produces_forward_motion_via_contact(self):
        sim = Simulation("flat", dt=FIXED_DT)
        x0 = sim.vehicle.position[0]
        for _ in range(int(6.0 / FIXED_DT)):
            snap = sim.update({"throttle": 0.5, "brake": 0.0,
                               "steer": 0.0}, record=False)
        self.assertGreater(snap["position"][0], x0 + 5.0)
        self.assertEqual(snap["contact_count"], 4)

    def test_no_throttle_no_creep(self):
        sim = Simulation("flat", dt=FIXED_DT)
        for _ in range(int(4.0 / FIXED_DT)):
            snap = sim.update(IDLE, record=False)
        # Idle rolling resistance must bleed off any residual settling motion.
        self.assertLess(abs(snap["position"][0]), 0.2)
        self.assertLess(snap["speed"], 0.05)

    def test_ground_spin_matches_rolling_relation(self):
        sim = Simulation("flat", dt=FIXED_DT)
        for _ in range(int(6.0 / FIXED_DT)):
            sim.update({"throttle": 0.5, "brake": 0.0, "steer": 0.0},
                       record=False)
        v = sim.vehicle
        for w in v.wheels:
            distance = V.length(V.sub(v.position, (0.0, 0.751, 0.0)))
            expected_angle = distance / w.radius
            self.assertAlmostEqual(abs(w.spin_ground), expected_angle,
                                   delta=expected_angle * 0.25 + 0.5)
            self.assertEqual(w.spin_air, 0.0)

    def test_air_spin_separate_from_ground_spin(self):
        sim = Simulation("flat", dt=FIXED_DT)
        # Move well beyond any suspension reach, then ground spin must freeze
        # while free-air spin accumulates.
        veh = sim.vehicle
        veh.position[1] = 20.0
        veh.velocity = [0.0, 0.0, 0.0]
        for _ in range(2):
            sim.update(IDLE, record=False)
        self.assertEqual(veh.last_contact_count, 0)
        g0 = [w.spin_ground for w in veh.wheels]
        for _ in range(int(1.0 / FIXED_DT)):
            sim.update({"throttle": 0.6, "brake": 0.0, "steer": 0.0},
                       record=False)
        self.assertEqual(veh.last_contact_count, 0)
        for w, g in zip(veh.wheels, g0):
            self.assertEqual(w.spin_ground, g)
            self.assertNotEqual(w.spin_air, 0.0)

    def test_visual_spin_contradiction_check(self):
        # While grounded the rendered wheel angle rate must track v/R rather
        # than the free-air spin rate.
        sim = Simulation("flat", dt=FIXED_DT)
        for _ in range(int(4.0 / FIXED_DT)):
            snap = sim.update({"throttle": 0.4, "brake": 0.0,
                               "steer": 0.0}, record=False)
        w = snap["wheels"][0]
        v = sim.vehicle.velocity[0]
        omega_roll = v / sim.vehicle.wheels[0].radius
        self.assertAlmostEqual(w["omega"], omega_roll,
                               delta=abs(omega_roll) * 0.2 + 0.5)


class ReplayTests(unittest.TestCase):
    def test_replay_is_deterministic(self):
        sim = Simulation("slope", dt=FIXED_DT)
        sim.run_seconds(6.0, PROFILES["slope"])
        replay = sim.to_replay()
        again = run_replay(replay)
        self.assertEqual(len(again.trajectory), len(sim.trajectory))
        for a, b in zip(sim.trajectory, again.trajectory):
            for j in range(3):
                self.assertEqual(a["position"][j], b["position"][j])
            self.assertEqual(a["contact_count"], b["contact_count"])

    def test_replay_suspension_override_changes_travel(self):
        sim = Simulation("bump", dt=FIXED_DT)
        sim.run_seconds(8.0, PROFILES["bump"])
        replay = sim.to_replay()
        soft = run_replay(replay, {"tau": 0.30})
        stiff = run_replay(replay, {"tau": 0.06})
        peak_soft = max(max(tr["travel"]) for tr in soft.trajectory)
        peak_stiff = max(max(tr["travel"]) for tr in stiff.trajectory)
        self.assertNotAlmostEqual(peak_soft, peak_stiff, places=4)


if __name__ == "__main__":
    unittest.main()
