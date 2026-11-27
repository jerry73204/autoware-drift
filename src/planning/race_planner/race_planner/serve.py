"""The live planner as a process of its own: JSON lines on stdin, JSON lines on stdout. A
simulator whose Python lacks CasADi (Isaac's) drives it through a pipe, one request per control
tick, as a ROS node will later be driven over topics.

    python -m race_planner.serve

First line in: {"init": {"map_dir": ..., "spec": ..., "classes": ..., "l0": <npz or null>,
    "ds": 4.0, "length": 200.0, "grip": 0.92, "budget": 1.0, "laps": 3,
    "multistart": null, "style": null}}  (laps: a circuit's; multistart: live.Driver's, null
    for one start, or MultiStart's options, e.g. {"flick": true, "cpus": [8, 9, 10]}: grip or
    drift chosen live; style: the driving style record, style.Style's fields, or a YAML path,
    null for fastest (roadmap 065); a bad record is refused, naming the field, before ready)
  out: {"ready": true, "s0": <the track's first s>, "style": <its name>}
Then per tick in: {"state": {"t", "x", "y", "yaw", "vx", "vy", "r", "delta"}} (map frame, the
  CoM; vx, vy in the body frame; yaw the body's)
  out: {"accel", "tire_angle", "ok", "fallback", "replan", "stop", "s", "n", "xi", "solve_s",
  "slot", "mode", "plan_id"} ("stop": why the car is being braked to a stop, or null; slot: the
  start the plan the car is on came from, G/H/D/F, G with one start; mode: grip or drift as the
  multistart classed it, null with one start; plan_id: that plan's number, 0 the first), and
  on the tick a plan is adopted "plan": {"s", "n", "beta_deg", "v"} (plan_message)
Opt-in init keys (FastSlowRR 003 R5: the planner behind a ROS node, race_planner_ros):
  "track_npz": a Track saved as arrays (track.Track's fields; FastSlowRR's planner_track.npz)
      instead of map_dir + classes; "l0" then also takes a CSV with s_m and v_ms columns
  "car": {"drag_scale", "diff", "steer_share"}: the spec's drag times drag_scale; the
      differential as named (tracker.py's rear slip needs "locked"); L1's steering lock times
      steer_share (L2 keeps the whole lock: room at the tightest bends)
  "pose": "rear_axle": the state's x, y and vy are the rear axle's (Autoware's base_link), not
      the CoM's: moved here by the car's lr
  "det": true: no clock in the planner (IPOPT uncut by wall time, every solve timed 0 s): a plan
      adopted at the budget's fixed latency whatever the host's load, as a lockstep run wants;
      with "itercap": n, a budgeted solve cut at n iterations a second of its budget instead
  "cold_retry": live.Driver's (iterations; 0 off)
  "beta_max_deg": live.Driver's beta_max, in degrees (the body's sideslip bound in every L1
      window; null: free)
  "slips": {"everywhere": bool, "front": k, "rear": k}: live.Driver's slip_everywhere,
      front_slip_max and rear_slip_max (the axles' slip angles within k of the peak)
  "stop_decel": live.Driver's (m/s^2; null: STOP_DECEL)
  "tracker_cold_retry": true: tracker.Tracker's cold_retry
  "plan_xy": true: a new plan's message carries its line in the map frame too: "x", "y",
      "psi" (heading of the line's tangent) and "t" (s) at its nodes
A style change at any time: {"style": {...}} (as init's), validated as at init and applied at
  the next re-plan, not within 25 m of a committed drift (live.Driver.set_style, style_due)
  out: {"style": <name>, "pending": true}, or {"style_error": <why>} with the style kept
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


def style_message(driver, record, t=None):
    """A runtime style message's answer: the style asked for (pending until the next re-plan
    due), or why it was refused (the current style kept, the car not stopped)."""
    try:
        st = driver.set_style(record, t)
    except (ValueError, NotImplementedError) as ex:
        return {"style_error": f"{type(ex).__name__}: {ex}", "style": driver.style.name}
    return {"style": st.name, "pending": True}


class PlanIds:
    """Numbers the plans the driver adopts, for plan_message."""

    def __init__(self):
        self.ids, self.keep = {}, []


def plan_message(driver, ids, track=None):
    """The plan the driver is on: its seed (slot), mode and number, and the plan itself the
    first time it is seen (s, n, sideslip in deg, speed at its nodes; with `track`, its line in
    the map frame too: plan_xy)."""
    p = driver.plan
    if p is None:
        return {"slot": None, "mode": None, "plan_id": None}
    out = {"slot": p.get("slot", "G"), "mode": p.get("mode")}
    k = id(p)
    if k not in ids.ids:
        ids.ids[k] = len(ids.keep)
        ids.keep.append(p)                     # held, so no later plan reuses its id
        X = np.asarray(p["X"], float)
        out["plan"] = {"s": np.round(np.asarray(p["s"], float), 2).tolist(),
                       "n": np.round(X[3], 3).tolist(),
                       "beta_deg": np.round(np.degrees(np.arctan2(X[1], np.maximum(X[0], 0.5))),
                                            2).tolist(),
                       "v": np.round(np.hypot(X[0], X[1]), 2).tolist()}
        if track is not None:
            out["plan"].update(plan_xy(track, p))
    out["plan_id"] = ids.ids[k]
    return out


def deterministic(itercap=None):
    """No clock in the planner (init "det"): studies/mode_choice_m4.py's RP_DET=1, here."""
    import types
    from race_planner import multistart, tracker, window
    clock = types.SimpleNamespace(time=lambda: 0.0, perf_counter=lambda: 0.0)
    window.time = tracker.time = multistart.time = clock
    init = window.Window.__init__

    def uncut(self, *a, **kw):
        if itercap and kw.get("budget") is not None:
            kw["max_iter"] = max(1, int(round(float(itercap) * kw["budget"])))
        kw["budget"] = None
        init(self, *a, **kw)
    window.Window.__init__ = uncut


def _load_l0(path):
    if str(path).endswith(".csv"):
        z = np.genfromtxt(path, delimiter=",", names=True)
        return {"s": z["s_m"], "speed": z["v_ms"]}
    return dict(np.load(path))


def plan_xy(track, plan):
    """The plan's line in the map frame: x, y, the tangent's heading psi, and t, at its nodes."""
    X = np.asarray(plan["X"], float)
    s = np.asarray(plan["s"], float)
    n = X[3]
    psi_c = np.interp(s, track.s, np.unwrap(track.psi))
    x = np.interp(s, track.s, track.x) - n * np.sin(psi_c)
    y = np.interp(s, track.s, track.y) + n * np.cos(psi_c)
    return {"x": np.round(x, 3).tolist(), "y": np.round(y, 3).tolist(),
            "psi": np.round(psi_c + X[4], 4).tolist(),
            "t": np.round(np.asarray(plan["t"], float), 3).tolist()}


def main():
    from dataclasses import replace
    from race_planner import live
    from race_planner.tracker import Tracker
    from race_planner.style import as_style
    init = json.loads(sys.stdin.readline())["init"]
    if init.get("det"):
        deterministic(init.get("itercap"))
    style = as_style(init.get("style")).require_implemented()   # refused before the track loads
    car = Car.from_spec(init["spec"])
    adj = init.get("car") or {}
    if adj.get("drag_scale") is not None:
        car = replace(car, drag=car.drag * float(adj["drag_scale"]))
    if adj.get("diff"):
        car = replace(car, diff=str(adj["diff"]))
    plan_car = car
    if adj.get("steer_share") is not None:
        plan_car = replace(car, steer_max=car.steer_max * float(adj["steer_share"]))
    if init.get("track_npz"):
        z = np.load(init["track_npz"])
        track = Track(**{k: (z[k].item() if z[k].ndim == 0 else z[k]) for k in z.files})
    else:
        track = Track.from_swb_course(init["map_dir"], SurfaceTable.from_yaml(init["classes"]), car)
    l0 = _load_l0(init["l0"]) if init.get("l0") else None
    lap = None
    if track.closed:                  # a circuit: driven as `laps` laps of open road
        laps = int(init.get("laps", 3))
        lap = track.length + float(track.s[1] - track.s[0])
        track = track.unrolled(laps)
        if l0 is not None:            # its lap's speed, lap after lap
            l0 = {"s": np.concatenate([l0["s"] + k * lap for k in range(laps + 1)]),
                  "speed": np.tile(l0["speed"], laps + 1)}
    track = track.resampled(float(init.get("ds", 4.0)))
    ms = init.get("multistart")
    if ms is not None and lap is not None and style.name == "drift_budget":
        ms = dict(ms, lap=lap)        # S2's budget a lap (roadmap 065 call 3)
    slips = init.get("slips") or {}
    extra = {}
    if slips.get("everywhere"):
        extra["slip_everywhere"] = True
    if slips.get("front") is not None:
        extra["front_slip_max"] = float(slips["front"])
    if slips.get("rear") is not None:
        extra["rear_slip_max"] = float(slips["rear"])
    if init.get("stop_decel") is not None:
        extra["stop_decel"] = float(init["stop_decel"])
    driver = live.Driver(plan_car, track,
                         Tracker(car, track, cold_retry=bool(init.get("tracker_cold_retry"))),
                         l0=l0, **extra,
                         ds=float(init.get("ds", 4.0)), length=float(init.get("length", 200.0)),
                         grip=float(init.get("grip", 0.92)), budget=float(init.get("budget", 1.0)),
                         use_estimate=bool(init.get("friction_estimate", False)),
                         multistart=ms, style=style, cold_retry=int(init.get("cold_retry", 0)),
                         beta_max=(None if init.get("beta_max_deg") is None
                                   else math.radians(float(init["beta_max_deg"]))))
    rear_axle = init.get("pose") == "rear_axle"
    with_xy = bool(init.get("plan_xy"))
    frenet = Frenet(track, None if lap is None else int(lap / float(track.s[1] - track.s[0])))
    _out({"ready": True, "s0": float(track.s[0]), "style": driver.style.name})
    started, ids = False, PlanIds()
    for line in sys.stdin:
        msg = json.loads(line)
        if msg.get("stop"):
            break
        if "style" in msg:
            _out(style_message(driver, msg["style"]))
            continue
        st = msg["state"]
        if rear_axle:                 # base_link to the CoM, lr ahead along the body
            c, sn = math.cos(st["yaw"]), math.sin(st["yaw"])
            st = dict(st, x=st["x"] + car.lr * c, y=st["y"] + car.lr * sn,
                      vy=st["vy"] + car.lr * st["r"])
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
              "solve_s": driver.solves[-1]["solve_s"] if driver.solves else 0.0,
              "n_solves": len(driver.solves),
              "solve_status": driver.solves[-1]["status"] if driver.solves else None,
              **plan_message(driver, ids, driver.track if with_xy else None)})
    driver.close()
    summary = {"solves": len(driver.solves), "solves_ok": sum(x["ok"] for x in driver.solves),
               "style": driver.style.name, "style_log": driver.style_log}
    if driver.multi is not None:
        summary["mode_changes"] = sum(r["mode"] != r["mode_before"] for r in driver.multi.records)
    _out({"done": True, **summary})


if __name__ == "__main__":
    main()
