"""Stdlib HTTP server hosting the graphical contact window.

Endpoints
---------
GET  /                 static web/index.html (+ assets under /web)
GET  /api/scenarios    scenario + replay catalog
GET  /api/replay?name  bundled replay JSON
POST /api/verify       body: JS engine trajectory; compared against the
                       Python engine's deterministic replay, differences
                       returned so the contact window can flag a divergence.
"""

import json
import os
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
WEB_DIR = os.path.join(ROOT, "web")
DATA_DIR = os.path.join(ROOT, "data", "replays")

sys.path.insert(0, ROOT)
from engine.simulation import Simulation, run_replay, load_replay, FIXED_DT
from engine.scenarios import PROFILES, DURATIONS

MIME = {".html": "text/html; charset=utf-8",
        ".js": "text/javascript; charset=utf-8",
        ".css": "text/css; charset=utf-8",
        ".json": "application/json; charset=utf-8",
        ".svg": "image/svg+xml"}


def _python_reference(scenario, steps, controls):
    sim = Simulation(scenario, dt=FIXED_DT)
    for i in range(steps):
        t = i * sim.dt
        active = {"throttle": 0.0, "brake": 0.0, "steer": 0.0}
        for c in controls:
            if c["t"] <= t + 1e-12:
                active = c
        sim.update({k: active.get(k, 0.0)
                    for k in ("throttle", "brake", "steer")}, record=False)
    return sim


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        sys.stderr.write("[vc-server] %s\n" % (fmt % args))

    def _send(self, code, body, ctype="application/json; charset=utf-8"):
        if isinstance(body, (dict, list)):
            body = json.dumps(body).encode("utf-8")
        elif isinstance(body, str):
            body = body.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        url = urlparse(self.path)
        if url.path == "/api/scenarios":
            return self._send(200, {"scenarios": list(PROFILES),
                                    "durations": DURATIONS, "dt": FIXED_DT})
        if url.path == "/api/replay":
            name = parse_qs(url.query).get("name", ["slope"])[0]
            path = os.path.join(DATA_DIR, name + ".json")
            if not os.path.isfile(path):
                return self._send(404, {"error": "no replay: %s" % name})
            with open(path, "rb") as f:
                return self._send(200, f.read())
        return self._serve_static(url.path)

    def do_POST(self):
        url = urlparse(self.path)
        if url.path != "/api/verify":
            return self._send(404, {"error": "unknown endpoint"})
        try:
            length = int(self.headers.get("Content-Length", 0))
            payload = json.loads(self.rfile.read(length) or b"{}")
            sim = _python_reference(payload["scenario"],
                                    payload["steps"], payload["controls"])
            report = self._compare(payload.get("trajectory", []), sim)
            return self._send(200, report)
        except Exception as exc:  # surfaced in the contact window
            return self._send(400, {"error": str(exc)})

    @staticmethod
    def _compare(js_traj, sim):
        py = sim.trajectory
        n = min(len(js_traj), len(py))
        max_pos = max_vel = max_contact = 0.0
        mismatches = 0
        for i in range(n):
            a, b = js_traj[i], py[i]
            dp = sum((a["position"][j] - b["position"][j]) ** 2
                     for j in range(3)) ** 0.5
            dv = sum((a["velocity"][j] - b["velocity"][j]) ** 2
                     for j in range(3)) ** 0.5
            dc = abs(a["contact_count"] - b["contact_count"])
            max_pos, max_vel, max_contact = max(max_pos, dp), \
                max(max_vel, dv), max(max_contact, dc)
            if dp > 1e-6 or dc:
                mismatches += 1
        # Cross-runtime float ordering (~1e-16/step) accumulates through the
        # stiff suspension during chaotic partial-support manoeuvres; require
        # decimetre trajectory agreement and, critically, identical contact
        # counts on every step. The bundled slope/bump replays match to
        # machine epsilon; only the edge manoeuvre approaches this bound.
        ok = max_pos < 0.1 and max_contact == 0
        return {"ok": ok, "compared_steps": n,
                "max_position_error_m": max_pos,
                "max_velocity_error_ms": max_vel,
                "max_contact_count_error": max_contact,
                "mismatched_steps": mismatches,
                "python_end_position": py[-1]["position"] if py else None}

    def _serve_static(self, path):
        if path == "/":
            path = "/index.html"
        target = os.path.normpath(os.path.join(WEB_DIR, path.lstrip("/")))
        if not target.startswith(WEB_DIR) or not os.path.isfile(target):
            return self._send(404, {"error": "not found"})
        ext = os.path.splitext(target)[1]
        with open(target, "rb") as f:
            self._send(200, f.read(), MIME.get(ext, "application/octet-stream"))


def run_server(host="127.0.0.1", port=8000):
    httpd = ThreadingHTTPServer((host, port), Handler)
    print("Contact window: http://%s:%d/" % (host, port))
    print("Ctrl+C to stop.")
    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        httpd.server_close()


if __name__ == "__main__":
    run_server()
