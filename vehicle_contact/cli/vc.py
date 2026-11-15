#!/usr/bin/env python3
"""Headless helper for the wheel-contact module.

The graphical contact window (see docs/contact_window.md) is the primary
interface.  This CLI only supports it by:

  run      run a scenario/replay and print contact + chassis diagnostics
  replay   replay a recorded JSON input file and verify the trajectory
  record   generate the standard scenario replay files
  serve    start the local web server that hosts the contact window
"""

import argparse
import csv
import json
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from engine.simulation import (Simulation, load_replay, run_replay, FIXED_DT)
from engine.scenarios import PROFILES, DURATIONS
from engine import math3d as V

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DATA_DIR = os.path.join(ROOT, "data", "replays")


def _print_summary(sim):
    snaps = sim.snapshots
    final = snaps[-1]
    min_contacts = min(s["contact_count"] for s in snaps)
    max_travel = max(w["travel_ratio"] for s in snaps for w in s["wheels"])
    airborne_steps = sum(1 for s in snaps if s["contact_count"] == 0)
    partial_steps = sum(1 for s in snaps if 0 < s["contact_count"] < 4)
    start, end = snaps[0]["position"], final["position"]
    print("scenario          : %s" % sim.scenario)
    print("steps / dt        : %d @ %.5f s (%.2f s)"
          % (len(snaps), sim.dt, sim.time))
    print("chassis start  XYZ: (%.3f %.3f %.3f)" % start)
    print("chassis end    XYZ: (%.3f %.3f %.3f)" % end)
    print("net displacement  : %.3f m" % V.length(V.sub(end, start)))
    print("peak speed        : %.3f m/s" % max(s["speed"] for s in snaps))
    print("min contact count : %d   (airborne steps: %d, partial: %d)"
          % (min_contacts, airborne_steps, partial_steps))
    print("peak travel ratio : %.2f of max suspension travel" % max_travel)
    print("body hull contact : %s"
          % ("yes" if any(s["body_contact"] for s in snaps) else "no"))
    warn = {}
    for s in snaps:
        for w in s["warnings"]:
            warn[w] = warn.get(w, 0) + 1
    if warn:
        print("contact warnings  :")
        for k, v in sorted(warn.items()):
            print("  %-32s %d steps" % (k, v))


def _write_csv(sim, path):
    with open(path, "w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["t", "x", "y", "z", "vx", "vy", "vz", "speed",
                     "contacts", "body_contact",
                     "comp_FL", "comp_FR", "comp_RL", "comp_RR",
                     "spin_ground_FL", "spin_air_FL"])
        for tr in sim.trajectory:
            p, vel = tr["position"], tr["velocity"]
            wr.writerow([round(tr["t"], 5), p[0], p[1], p[2],
                         vel[0], vel[1], vel[2], tr["speed"],
                         tr["contact_count"], int(tr["body_contact"]),
                         *[round(c, 6) for c in tr["travel"]],
                         round(tr["spin_ground"][0], 5),
                         round(tr["spin_air"][0], 5)])
    print("trajectory CSV    : %s" % path)


def cmd_run(args):
    sim = Simulation(args.scenario, dt=FIXED_DT,
                     query_kind=args.query)
    profile = PROFILES[args.scenario]
    sim.run_seconds(args.duration or DURATIONS[args.scenario], profile)
    _print_summary(sim)
    if args.csv:
        _write_csv(sim, args.csv)
    if args.save_replay:
        path = sim.save_replay(args.save_replay,
                               meta={"profile": args.scenario})
        print("replay saved      : %s" % path)
    if args.snapshot_out:
        with open(args.snapshot_out, "w", encoding="utf-8") as f:
            json.dump(sim.snapshots, f)
        print("snapshots JSON    : %s" % args.snapshot_out)
    return 0


def cmd_record(_args):
    os.makedirs(DATA_DIR, exist_ok=True)
    for name, profile in PROFILES.items():
        sim = Simulation(name, dt=FIXED_DT)
        sim.run_seconds(DURATIONS[name], profile)
        path = os.path.join(DATA_DIR, "%s.json" % name)
        sim.save_replay(path, meta={"profile": name, "kind": "standard"})
        csv_path = os.path.join(DATA_DIR, "%s.csv" % name)
        _write_csv(sim, csv_path)
        print("recorded %-6s -> %s" % (name, path))
    return 0


def cmd_replay(args):
    replay = load_replay(args.file)
    overrides = None
    if args.tau is not None or args.travel is not None or args.k is not None:
        overrides = {}
        if args.tau is not None:
            overrides["tau"] = args.tau
        if args.travel is not None:
            overrides["max_travel"] = args.travel
        if args.k is not None:
            overrides["spring_k"] = args.k
    sim = run_replay(replay, overrides)
    _print_summary(sim)
    if args.csv:
        _write_csv(sim, args.csv)
    return 0


def cmd_serve(args):
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    from server import run_server
    run_server(host=args.host, port=args.port)
    return 0


def cmd_test(_args):
    loader = unittest.TestLoader()
    suite = loader.discover(os.path.join(ROOT, "tests"),
                            pattern="test_*.py")
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    return 0 if result.wasSuccessful() else 1


def main(argv=None):
    ap = argparse.ArgumentParser(
        prog="vehicle-contact",
        description="Headless helper for the wheel contact-motion module.")
    sub = ap.add_subparsers(dest="cmd", required=True)

    p_run = sub.add_parser("run", help="run a scenario with its input profile")
    p_run.add_argument("scenario", choices=list(PROFILES))
    p_run.add_argument("--query", choices=["ray", "sphere"], default="sphere")
    p_run.add_argument("--duration", type=float, default=None)
    p_run.add_argument("--csv", default=None)
    p_run.add_argument("--save-replay", default=None)
    p_run.add_argument("--snapshot-out", default=None)
    p_run.set_defaults(func=cmd_run)

    p_rec = sub.add_parser("record", help="generate standard replay files")
    p_rec.set_defaults(func=cmd_record)

    p_rep = sub.add_parser("replay", help="replay a recorded input file")
    p_rep.add_argument("file")
    p_rep.add_argument("--tau", type=float, default=None)
    p_rep.add_argument("--travel", type=float, default=None)
    p_rep.add_argument("--k", type=float, default=None)
    p_rep.add_argument("--csv", default=None)
    p_rep.set_defaults(func=cmd_replay)

    p_srv = sub.add_parser("serve", help="host the graphical contact window")
    p_srv.add_argument("--host", default="127.0.0.1")
    p_srv.add_argument("--port", type=int, default=8000)
    p_srv.set_defaults(func=cmd_serve)

    p_test = sub.add_parser("test", help="run the engine test suite")
    p_test.set_defaults(func=cmd_test)

    args = ap.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    raise SystemExit(main())
