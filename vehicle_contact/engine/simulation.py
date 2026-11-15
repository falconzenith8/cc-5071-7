"""Fixed-step simulation, replay recording/playback and scenario definitions.

A replay is a JSON document containing the terrain/scenario name, the vehicle
parameters, the fixed timestep and a list of timestamped control samples.  The
same file drives both the headless CLI diagnostics and the graphical window,
which is what makes a test *reproducible*: replaying the file must reproduce
the recorded chassis trajectory and contact counts bit-for-bit (same engine,
same dt => deterministic float output).
"""

import json
import os

from .terrain import TERRAINS
from .vehicle import Vehicle

FIXED_DT = 1.0 / 120.0


def build_scenario(name, vehicle_overrides=None):
    terrain = TERRAINS[name]()
    # Start slightly above the static-ride height; Simulation settles the
    # suspension before the first recorded step.
    defaults = {
        "flat": {"position": (0.0, 0.85, 0.0)},
        "slope": {"position": (0.0, 0.85, 0.0)},
        "bump": {"position": (0.0, 0.85, 0.0)},
        "edge": {"position": (0.0, 0.85, 0.0)},
    }
    params = dict(defaults.get(name, {}))
    if vehicle_overrides:
        params.update(vehicle_overrides)
    return terrain, params


def zero_control():
    return {"throttle": 0.0, "brake": 0.0, "steer": 0.0}


def _sample_at(controls, t):
    """Piecewise-constant control lookup for a replay timeline."""
    active = zero_control()
    for c in controls:
        if c["t"] <= t + 1e-12:
            active = {"throttle": c.get("throttle", 0.0),
                      "brake": c.get("brake", 0.0),
                      "steer": c.get("steer", 0.0)}
        else:
            break
    return active


class Simulation:
    """Owns the vehicle, accumulates time and stores replay samples."""

    def __init__(self, scenario="flat", dt=FIXED_DT, params=None,
                 query_kind=None):
        terrain, vp = build_scenario(scenario, params)
        if query_kind:
            vp["query_kind"] = query_kind
        self.vehicle = Vehicle(terrain, vp)
        self.scenario = scenario
        self.dt = dt
        self.time = 0.0
        self.step_index = 0
        self.recorded_controls = []     # for replay export
        self.snapshots = []
        self.trajectory = []
        self._settle()

    def _settle(self, settle_time=1.5):
        """Let the suspension reach static equilibrium before driving.

        These settling steps are neither recorded nor counted in the replay,
        so every run starts from a physically consistent loaded state rather
        than a drop transient that would pollute the contact diagnostics.
        """
        idle = zero_control()
        n = int(round(settle_time / self.dt))
        for _ in range(n):
            self.vehicle.step(self.dt, idle)
        # Reset the clock: the recorded run begins at t=0, already settled.
        self.time = 0.0
        self.step_index = 0

    # ---------------------------------------------------------- live drive
    def update(self, control, record=True):
        if record:
            self.recorded_controls.append({"t": self.time, **control})
        snap = self.vehicle.step(self.dt, control)
        self.time += self.dt
        self.step_index += 1
        self.snapshots.append(snap)
        self.trajectory.append({
            "t": self.time,
            "position": snap["position"],
            "velocity": snap["velocity"],
            "speed": snap["speed"],
            "contact_count": snap["contact_count"],
            "body_contact": snap["body_contact"],
            "travel": [w["compression"] for w in snap["wheels"]],
            "spin_ground": [w["spin_ground"] for w in snap["wheels"]],
            "spin_air": [w["spin_air"] for w in snap["wheels"]],
        })
        return snap

    def run_seconds(self, duration, control_fn):
        """Run headless; ``control_fn(t)`` returns the control dict."""
        n = int(round(duration / self.dt))
        for _ in range(n):
            self.update(control_fn(self.time))
        return self.snapshots[-1]

    # ------------------------------------------------------------- replay
    def to_replay(self, meta=None):
        # Compress the timeline: keep only samples where a value changes.
        compact = []
        last = None
        for c in self.recorded_controls:
            triple = (c["throttle"], c["brake"], c["steer"])
            if triple != last:
                compact.append(c)
                last = triple
        return {
            "format": "vehicle-contact-replay/1",
            "scenario": self.scenario,
            "dt": self.dt,
            "duration": self.time,
            "vehicle_params": self._vehicle_params(),
            "controls": compact,
            "meta": meta or {},
        }

    def _vehicle_params(self):
        v = self.vehicle
        w0 = v.wheels[0]
        return {
            "mass": v.mass,
            "query_kind": v.query_kind,
            "tire_mu": v.tire_mu,
            "suspension": {
                "rest_length": w0.rest_length,
                "max_travel": w0.max_travel,
                "spring_k": w0.spring_k,
                "tau": w0.tau,
            },
        }

    def save_replay(self, path, meta=None):
        os.makedirs(os.path.dirname(path) or ".", exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_replay(meta), f, indent=1)
        return path


def load_replay(path):
    with open(path, "r", encoding="utf-8") as f:
        return json.load(f)


def run_replay(replay, suspension_overrides=None):
    """Rebuild a Simulation from a replay file and re-simulate every step.

    Returns the Simulation; its ``trajectory`` is the replayed ground truth.
    """
    params = dict(replay.get("vehicle_params", {}))
    if suspension_overrides:
        susp = dict(params.get("suspension", {}))
        susp.update(suspension_overrides)
        params["suspension"] = susp
    sim = Simulation(scenario=replay["scenario"], dt=replay["dt"],
                     params=params)
    controls = replay["controls"]
    n = int(round(replay["duration"] / sim.dt))
    for i in range(n):
        t = i * sim.dt
        sim.update(_sample_at(controls, t), record=False)
    return sim
