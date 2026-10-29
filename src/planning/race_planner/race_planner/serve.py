"""The live planner as a process of its own: JSON lines on stdin, JSON lines on stdout. A
simulator whose Python lacks CasADi (Isaac's) drives it through a pipe, one request per control
tick, as a ROS node will later be driven over topics.

    python -m race_planner.serve

First line in: {"init": {"map_dir": ..., "spec": ..., "classes": ..., "l0": <npz or null>,
    "ds": 4.0, "length": 200.0, "grip": 0.92, "budget": 1.0, "laps": 3}}  (laps: a circuit's)
  out: {"ready": true, "s0": <the track's first s>}
Then per tick in: {"state": {"t", "x", "y", "yaw", "vx", "vy", "r", "delta"}} (map frame, the
  CoM; vx, vy in the body frame; yaw the body's)
  out: {"accel", "tire_angle", "ok", "fallback", "replan", "stop", "s", "n", "xi", "solve_s"}
  ("stop": why the car is being braked to a stop, or null)
"{"stop": true}" ends it. The first state starts L1's first plan (not under its budget).

The state's s, n and xi are the track's (the planner's centreline, on its grid), found here, so
the simulator needs to know nothing of the planner's track.
"""
import json
import math
import sys

import numpy as np

from race_planner.car import Car
from race_planner.track import SurfaceTable, Track


def _out(obj):
    sys.stdout.write(json.dumps(obj, default=lambda o: o.item()) + "\n")   # numpy scalars
    sys.stdout.flush()


class Frenet:
    """The car's place on the track: s, n (left > 0) and xi (heading off the line's)."""

    def __init__(self, track, first=None):
        # first: how many nodes the first search looks in (a circuit's first lap: its laps
        # overlap, and the nearest node of all may be two laps on)
        self.t, self.i, self.first = track, 0, first or len(track.s)

    def __call__(self, x, y, yaw):
        tr = self.t
        lo, hi = max(0, self.i - 30), min(len(tr.s), self.i + 60)
        d = np.hypot(tr.x[lo:hi] - x, tr.y[lo:hi] - y)
        if self.i == 0:
            d = np.hypot(tr.x[:self.first] - x, tr.y[:self.first] - y)
            lo = 0
        self.i = i = lo + int(np.argmin(d))
        ps = float(tr.psi[i])
        dx, dy = x - tr.x[i], y - tr.y[i]
        along = dx * math.cos(ps) + dy * math.sin(ps)
        n = -dx * math.sin(ps) + dy * math.cos(ps)
        xi = (yaw - ps + math.pi) % (2 * math.pi) - math.pi
        return float(tr.s[i] + along), n, xi


def main():
    from race_planner import live
    from race_planner.tracker import Tracker
    init = json.loads(sys.stdin.readline())["init"]
    car = Car.from_spec(init["spec"])
    track = Track.from_swb_course(init["map_dir"], SurfaceTable.from_yaml(init["classes"]), car)
    l0 = dict(np.load(init["l0"])) if init.get("l0") else None
    lap = None
    if track.closed:                  # a circuit: driven as `laps` laps of open road
        laps = int(init.get("laps", 3))
        lap = track.length + float(track.s[1] - track.s[0])
        track = track.unrolled(laps)
        if l0 is not None:            # its lap's speed, lap after lap
            l0 = {"s": np.concatenate([l0["s"] + k * lap for k in range(laps + 1)]),
                  "speed": np.tile(l0["speed"], laps + 1)}
    track = track.resampled(float(init.get("ds", 4.0)))
    driver = live.Driver(car, track, Tracker(car, track), l0=l0,
                         ds=float(init.get("ds", 4.0)), length=float(init.get("length", 200.0)),
                         grip=float(init.get("grip", 0.92)), budget=float(init.get("budget", 1.0)))
    frenet = Frenet(track, None if lap is None else int(lap / float(track.s[1] - track.s[0])))
    _out({"ready": True, "s0": float(track.s[0])})
    started = False
    for line in sys.stdin:
        msg = json.loads(line)
        if msg.get("stop"):
            break
        st = msg["state"]
        s, n, xi = frenet(st["x"], st["y"], st["yaw"])
        state = {"t": st["t"], "s": s, "n": n, "xi": xi, "vx": st["vx"], "vy": st["vy"],
                 "r": st["r"], "delta": st["delta"],
                 "mu_ratio": float(st.get("mu_ratio", 1.0))}   # the car's friction estimate
        if not started:
            driver.start(state)
            started = True
        cmd = driver.step(state)
        last = driver.log[-1]
        _out({"accel": cmd["accel"], "tire_angle": cmd["tire_angle"], "ok": cmd.get("ok", True),
              "fallback": cmd.get("fallback", False), "replan": last["replan"], "s": s, "n": n,
              "xi": xi, "n_plan": last["n_plan"], "v_plan": last["v_plan"], "stop": last["stopping"],
              "beta_plan_deg": last["beta_plan_deg"],
              "solve_s": driver.solves[-1]["solve_s"] if driver.solves else 0.0})
    summary = {"solves": len(driver.solves), "solves_ok": sum(x["ok"] for x in driver.solves)}
    _out({"done": True, **summary})


if __name__ == "__main__":
    main()
