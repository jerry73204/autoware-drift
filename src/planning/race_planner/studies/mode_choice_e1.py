"""Grip or drift chosen live, without a plant (roadmap 061 M3, experiment E1;
docs/mode-choice-m3.md).

live.Driver re-plans along a course while the car follows the adopted plan exactly (the state at
any time is the plan's at the car's place: no tracker, no plant, no model error), so what is
measured is the chooser and the solver alone: how often the mode a corner is planned in changes,
whether it flips back and forth, how long the course takes on the plans adopted, how often a
re-plan overruns its budget, and how many iterations each seed takes, re-plan after re-plan.

Courses (the planner's friction, grip 0.92 times the road's, as live):
- `chain-1.03`, `chain-0.45`, `chain-0.3`: the drift study's corners (R 9, 15, 25, 40, 60 m at 90
  and 180 deg) chained left and right with 120 m straights between, 8 m wide, fall-off 0.8;
- `haruna`, `haruna-0.45`, `haruna-0.3`: swb's Haruna (asphalt, 1.03), and with three patches of
  the friction given (its road's mu scaled there) over about half of it.

Configs: `single` (060's one start) and `ms<threshold %>` ({G, D, F}, F into hairpins only, each
seed under 0.9 s of wall time; the mode switch threshold in % of the window's time).

Run: env -u PYTHONPATH .venv/bin/python studies/mode_choice_e1.py all [--jobs 2]
     (or `run <course> <config> [--cpus 8,9,10]`), then `report`.
Results: swb's play_log/race_planner/mode_choice/m3/ (<course>_<config>.json, report.md).
"""
import argparse
import json
import math
import os
import subprocess
import sys
import time
import types

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from race_planner import modes  # noqa: E402
from race_planner.car import Car  # noqa: E402
from race_planner.live import Driver  # noqa: E402
from race_planner.multistart import MultiStart  # noqa: E402
from race_planner.sim import NoTracker, PlanFollower  # noqa: E402
from race_planner.track import SurfaceTable, Track  # noqa: E402

SWB = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
SPEC = os.path.join(SWB, "src/sim_vehicle_spec/vehicles/drift_mustang.yaml")
CLASSES = os.path.join(SWB, "src/swb_isaac_world/config/surface_classes.yaml")
HARUNA = os.path.join(SWB, "data/courses/haruna")
OUT = os.environ.get("E1_OUT", os.path.join(SWB, "play_log/race_planner/mode_choice/m3"))

GRIP = 0.92
DS = 4.0
DT = 0.05                     # s: the follower's step (no plant: the plan is the car)
SEED_BUDGET = 0.9             # s: each seed's max_wall_time (the re-plan's latency is 1.0)
CHAIN = ((25, 90), (9, 180), (60, 90), (15, 180), (40, 180), (9, 90), (25, 180), (60, 180),
         (15, 90), (40, 90))
STRAIGHT, RAMP, HALF = 120.0, 8.0, 4.0
PATCHES = ((220.0, 560.0), (840.0, 1060.0), (1160.0, 1300.0))   # Haruna's s, m
COURSES = ("chain-1.03", "chain-0.45", "chain-0.3", "haruna", "haruna-0.45", "haruna-0.3")
CONFIGS = ("single", "ms0.1", "ms0.3", "ms0.5")


def chain(mu_eff, sliding=0.8, ds=1.0):
    """The study's corners in a row (CHAIN), left and right in turn, STRAIGHT m apart, RAMP m
    curvature ramps, 80 m before the first and 150 m after the last."""
    pts, s = [(0.0, 0.0)], 80.0
    pts.append((s, 0.0))
    for k, (R, th) in enumerate(CHAIN):
        sg = 1.0 if k % 2 == 0 else -1.0
        arc = math.radians(th) * R - RAMP
        for d, kap in ((RAMP, sg / R), (arc, sg / R), (RAMP, 0.0), (STRAIGHT, 0.0)):
            s += d
            pts.append((s, kap))
    s += 30.0
    pts.append((s, 0.0))
    sk, kk = np.array(pts).T
    ss = np.arange(0.0, sk[-1] + ds / 2, ds)
    k = np.interp(ss, sk, kk)
    psi = np.concatenate([[0.0], np.cumsum(0.5 * (k[1:] + k[:-1]) * ds)])
    x = np.concatenate([[0.0], np.cumsum(0.5 * (np.cos(psi[1:]) + np.cos(psi[:-1])) * ds)])
    y = np.concatenate([[0.0], np.cumsum(0.5 * (np.sin(psi[1:]) + np.sin(psi[:-1])) * ds)])
    one = np.ones_like(ss)
    return Track(s=ss, x=x, y=y, z=0 * ss, psi=psi, kappa=k, w_left=HALF * one,
                 w_right=HALF * one, mu=mu_eff / GRIP * one, closed=False,
                 sliding=sliding * one, grade=0 * ss, name=f"chain-{mu_eff}")


def haruna(car, patch=None):
    """Haruna; with `patch`, the planner's friction `patch` over PATCHES (eased over 10 m)."""
    t = Track.from_swb_course(HARUNA, SurfaceTable.from_yaml(CLASSES), car)
    if patch is not None:
        w = np.zeros_like(t.s)
        for a, b in PATCHES:
            w = np.maximum(w, np.clip(np.minimum(t.s - a, b - t.s) / 10.0, 0.0, 1.0))
        mu = t.mu * (1 - w) + w * patch / GRIP
        t = Track(**{**t.__dict__, "mu": mu})
    return t


def course(name, car):
    if name.startswith("chain"):
        return chain(float(name.split("-")[1])).resampled(DS)
    return haruna(car, float(name.split("-")[1]) if "-" in name else None).resampled(DS)


def corner_mode(plan, c):
    """The plan's mode in corner c: its rear REAR_PAST past the peak from 30 m before the
    turn-in to the exit."""
    sel = (plan["s"] >= c.s_in - 30.0) & (plan["s"] <= c.s_out)
    return "drift" if sel.any() and plan["past"][sel].max() >= modes.REAR_PAST else "grip"


def run(course_name, config, cpus=None):
    car = Car.from_spec(SPEC)
    track = course(course_name, car)
    ms = None
    if config != "single":
        ms = {"threshold": float(config[2:]) / 100.0, "flick": True, "seed_budget": SEED_BUDGET,
              "cpus": cpus}
    d = Driver(car, track, NoTracker(), ds=DS, grip=GRIP, multistart=ms)
    cls = d.multi if d.multi is not None else types.SimpleNamespace(
        car=car, track=d.track, grip=GRIP)
    cs = modes.corners(track, GRIP)
    s_end = float(track.s[-1]) - 10.0
    f = PlanFollower(d, 10.0, 12.0)
    st = f.state()
    c0 = time.time()
    d.start(st)
    last, adoptions, trace = None, [], []
    result = "timeout"
    while st["t"] < 600.0:
        d.step(st)
        if d.plan is not last:
            last = d.plan
            if "past" not in last:
                MultiStart.classify(cls, last, last["i0"], None)
            adoptions.append({"t": round(st["t"], 3), "s": round(st["s"], 2),
                              "s0": float(last["s"][0]), "s1": float(last["s"][-1]),
                              "slot": last.get("slot", "G"), "mode": last["mode"],
                              "corners": {i: corner_mode(last, c) for i, c in enumerate(cs)
                                          if last["s"][0] <= c.s_in
                                          and last["s"][-1] >= 0.5 * (c.s_in + c.s_out)}})
        if d.stopping:
            result = "stopped: " + d.stopping
            break
        st = f.step(DT)
        i = int(np.searchsorted(d.plan["s"], st["s"]))
        past = float(np.interp(st["s"], d.plan["s"], d.plan["past"]))
        trace.append((round(st["t"], 2), round(st["s"], 2), round(math.degrees(past), 2)))
        if st["s"] >= s_end:
            result = "done"
            break
    wall = time.time() - c0
    d.close()
    tr = np.array(trace)
    driven = []
    for c in cs:
        sel = (tr[:, 1] >= c.s_in - 30.0) & (tr[:, 1] <= c.s_out)
        t_in = float(np.interp(c.s_in - 50.0, tr[:, 1], tr[:, 0]))
        t_out = float(np.interp(min(c.s_out + 50.0, s_end), tr[:, 1], tr[:, 0]))
        driven.append({"s_in": c.s_in, "s_out": c.s_out, "R": round(c.R, 1),
                       "turn": round(math.degrees(c.turn)), "mu": round(c.mu, 3),
                       "sliding": c.sliding, "hairpin": c.hairpin,
                       "driven": "drift" if sel.any() and tr[sel, 2].max() >= 3.0 else "grip",
                       "past_max": round(float(tr[sel, 2].max()), 2) if sel.any() else None,
                       "time": round(t_out - t_in, 3)})
    out = {"course": course_name, "config": config, "result": result, "t": round(st["t"], 3),
           "s": round(st["s"], 1), "s_end": s_end, "wall": round(wall, 1),
           "solves": [{k: (round(v, 4) if isinstance(v, float) else v) for k, v in x.items()}
                      for x in d.solves],
           "records": d.multi.records if d.multi is not None else None,
           "adoptions": adoptions, "corners": driven}
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, f"{course_name}_{config}.json"), "w") as fh:
        json.dump(out, fh, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print(course_name, config, result, out["t"], "s", len(d.solves), "re-plans", round(wall), "s wall",
          flush=True)
    return out


def run_all(jobs, configs, courses):
    """Every course x config as its own process (a Driver's seed processes cannot be a pool's
    children), `jobs` at a time, each multistart run's seeds pinned to three quiet cores."""
    todo = [(c, k) for k in configs for c in courses]
    cores = [[8, 9, 10], [11, 12, 13], [14, 15, 24], [25, 26, 27]][:jobs]
    free, running = list(range(jobs)), {}
    os.makedirs(OUT, exist_ok=True)
    while todo or running:
        while todo and free:
            slot = free.pop(0)
            c, k = todo.pop(0)
            log = open(os.path.join(OUT, f"{c}_{k}.log"), "w")
            cpu = ",".join(map(str, cores[slot]))
            cmd = ["taskset", "-c", cpu, sys.executable, __file__, "run", c, k, "--cpus", cpu]
            running[slot] = (subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT), c, k)
        time.sleep(2.0)
        for slot, (p, c, k) in list(running.items()):
            if p.poll() is not None:
                print(c, k, "exit", p.returncode, flush=True)
                del running[slot]
                free.append(slot)


def _pct(a, q):
    return round(float(np.percentile(a, q)), 3) if len(a) else None


def report():
    """report.md: the gate's numbers per course and config."""
    rows, lines = {}, []
    for c in COURSES:
        for k in CONFIGS:
            p = os.path.join(OUT, f"{c}_{k}.json")
            if os.path.exists(p):
                rows[(c, k)] = json.load(open(p))
    say = lines.append
    say("# E1: the receding chain without a plant (roadmap 061 M3)\n")
    say("| course | config | result | time (s) | vs single | re-plans | overruns > 1.0 s | "
        "wall p50/p95/max | corners | drift-planned corners | max changes a corner | "
        "flip-flops | driven drift |")
    say("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for (c, k), r in rows.items():
        base = rows.get((c, "single"))
        walls = np.array([x["solve_s"] for x in r["solves"][1:]])
        per = {}
        for a in r["adoptions"]:
            for i, m in a["corners"].items():
                per.setdefault(i, []).append(m)
        changes = {i: sum(1 for a, b in zip(v, v[1:]) if a != b) for i, v in per.items()}
        flips = sum(1 for v in per.values() if any(v[j] != v[j + 1] and v[j + 1] != v[j + 2]
                                                    for j in range(len(v) - 2)))
        dp = sum(1 for v in per.values() if "drift" in v)
        dr = sum(1 for x in r["corners"] if x["driven"] == "drift")
        vs = (f"{100 * (r['t'] - base['t']) / base['t']:+.2f} %"
              if base and k != "single" and base["result"] == "done" and r["result"] == "done"
              else "")
        say(f"| {c} | {k} | {r['result']} | {r['t']:.2f} | {vs} | {len(walls)} | "
            f"{int((walls > 1.0).sum())} ({100 * (walls > 1.0).mean():.1f} %) | "
            f"{_pct(walls, 50)}/{_pct(walls, 95)}/{walls.max():.2f} | {len(r['corners'])} | {dp} | "
            f"{max(changes.values()) if changes else 0} | {flips} | {dr} |")
    say("\n## Iterations per seed and start (multistart runs)\n")
    say("| course | config | seed | start | solves | ok | cut by the budget | iterations p50/p95 "
        "| wall p50/p95 |")
    say("|---|---|---|---|---|---|---|---|---|")
    for (c, k), r in rows.items():
        if not r["records"]:
            continue
        groups = {}
        for rec in r["records"]:
            for slot, x in rec["seeds"].items():
                groups.setdefault((slot, x["start"]), []).append(x)
        for (slot, start), xs in sorted(groups.items()):
            it = np.array([x["iters"] for x in xs])
            ws = np.array([x["solve_s"] for x in xs])
            cut = sum(1 for x in xs if x["status"] == "Maximum_WallTime_Exceeded")
            say(f"| {c} | {k} | {slot} | {start} | {len(xs)} | {sum(x['ok'] for x in xs)} | "
                f"{cut} | {_pct(it, 50)}/{_pct(it, 95)} | {_pct(ws, 50)}/{_pct(ws, 95)} |")
    say("\n## D's iterations along a run of its own warm re-plans (n-th re-plan since its seed)\n")
    say("| course | config | 0 (seed) | 1 | 2 | 3 | 4+ |")
    say("|---|---|---|---|---|---|---|")
    for (c, k), r in rows.items():
        if not r["records"]:
            continue
        by, n = {}, None
        for rec in r["records"]:
            x = rec["seeds"].get("D")
            if x is None:
                n = None
                continue
            n = 0 if x["start"] != "own" else (n + 1 if n is not None else 1)
            by.setdefault(min(n, 4), []).append(x["iters"])
        say(f"| {c} | {k} | " + " | ".join(
            f"{_pct(by.get(j, []), 50)}/{_pct(by.get(j, []), 95)} ({len(by.get(j, []))})"
            for j in range(5)) + " |")
    say("\n## The chooser's decisions, by corner\n")
    say("A switch is the gate's (the other mode faster beyond the threshold on two re-plans in a "
        "row) or forced (the current mode has no plan: a drift whose slide is over, or a grip "
        "plan that slid). Each is put on the first corner whose exit is ahead of the window's "
        "start.\n")
    say("| course | config | gate grip>drift | gate drift>grip | forced drift>grip | forced "
        "grip>drift | most gate switches on a corner | corners flipped by the gate |")
    say("|---|---|---|---|---|---|---|---|")
    for (c, k), r in rows.items():
        if not r["records"]:
            continue
        kinds, per = {}, {}
        prev = None
        for rec in r["records"]:
            if rec["mode"] != rec["mode_before"]:
                gate = prev is not None and prev.get("count") == 1
                key = ("gate " if gate else "forced ") + rec["mode_before"] + ">" + rec["mode"]
                kinds[key] = kinds.get(key, 0) + 1
                ci = next((i for i, x in enumerate(r["corners"]) if x["s_out"] > rec["s0"]), None)
                if gate:
                    per.setdefault(ci, []).append(rec["mode"])
            prev = rec
        flips = sum(1 for v in per.values() if len(set(v)) > 1)
        say(f"| {c} | {k} | {kinds.get('gate grip>drift', 0)} | {kinds.get('gate drift>grip', 0)} "
            f"| {kinds.get('forced drift>grip', 0)} | {kinds.get('forced grip>drift', 0)} | "
            f"{max((len(v) for v in per.values()), default=0)} | {flips} |")
    say("\n## The chooser's mode, corner by corner\n")
    say("The chooser's mode over the re-plans whose choice was about a corner (its turn-in 10 m "
        "or more ahead), from the mode it had before the first of them: its changes there (gate "
        "or forced), and the corners where it left a mode and took it again.\n")
    say("| course | config | corners | most changes on a corner | corners left and retaken |")
    say("|---|---|---|---|---|")
    for (c, k), r in rows.items():
        if not r["records"]:
            continue
        seq = {}
        for rec in r["records"]:
            if rec.get("corner") is None:
                continue
            v = seq.setdefault(rec["corner"], [rec["mode_before"]])
            v.append(rec["mode"])
        ch = {kk: sum(1 for a, b in zip(v, v[1:]) if a != b) for kk, v in seq.items()}
        flips = sum(1 for v in seq.values() if any(v[j] != v[j + 1] and v[j + 1] != v[j + 2]
                                                    for j in range(len(v) - 2)))
        say(f"| {c} | {k} | {len(seq)} | {max(ch.values(), default=0)} | {flips} |")
    say("\n## When both modes had a plan\n")
    say("The other mode's gain over the current one's (% of the window's time, > 0: the other is "
        "faster), over the re-plans where both had a feasible plan.\n")
    say("| course | config | re-plans | both modes | gain p5/p50/p95 (%) | beyond 0.1 / 0.3 / 0.5 % |")
    say("|---|---|---|---|---|---|")
    for (c, k), r in rows.items():
        if not r["records"]:
            continue
        g = np.array([100 * x["gain"] for x in r["records"] if "gain" in x])
        say(f"| {c} | {k} | {len(r['records'])} | {len(g)} | "
            f"{_pct(g, 5)}/{_pct(g, 50)}/{_pct(g, 95)} | "
            f"{int((g > 0.1).sum())} / {int((g > 0.3).sum())} / {int((g > 0.5).sum())} |")
    say("\n## The seeds against G, re-plan by re-plan\n")
    say("Where D (or F) ran: its plan's mode in the corner ahead against G's, or which failed "
        "(not feasible within the budget).\n")
    say("| course | config | seed | runs | same mode as G | drift where G grip | grip where G "
        "drift | seed failed | G failed, seed ok | both failed |")
    say("|---|---|---|---|---|---|---|---|---|---|")
    for (c, k), r in rows.items():
        if not r["records"]:
            continue
        for slot in ("D", "F"):
            n = {"same": 0, "dg": 0, "gd": 0, "sf": 0, "gf": 0, "bf": 0}
            runs = 0
            for rec in r["records"]:
                x, g = rec["seeds"].get(slot), rec["seeds"]["G"]
                if x is None:
                    continue
                runs += 1
                if not g["ok"]:
                    n["gf" if x["ok"] else "bf"] += 1
                elif not x["ok"]:
                    n["sf"] += 1
                elif x["mode"] == g["mode"]:
                    n["same"] += 1
                else:
                    n["dg" if x["mode"] == "drift" else "gd"] += 1
            say(f"| {c} | {k} | {slot} | {runs} | {n['same']} | {n['dg']} | {n['gd']} | "
                f"{n['sf']} | {n['gf']} | {n['bf']} |")
    say("\n## Mode changes (the chooser's) and corners\n")
    for (c, k), r in rows.items():
        if not r["records"]:
            continue
        ch = [(round(x["s0"]), x["mode_before"], x["mode"]) for x in r["records"]
              if x["mode"] != x["mode_before"]]
        say(f"- {c} {k}: {len(ch)} changes {ch}")
    for (c, k), r in rows.items():
        if k != "ms0.3":
            continue
        base = rows.get((c, "single"))
        say(f"\n### {c}: corners (ms0.3 against single)\n")
        say("| s_in | R | turn | mu | planned modes (ms0.3) | driven | time | single's |")
        say("|---|---|---|---|---|---|---|---|")
        per = {}
        for a in r["adoptions"]:
            for i, m in a["corners"].items():
                per.setdefault(int(i), []).append(m[0])
        for i, x in enumerate(r["corners"]):
            bt = base["corners"][i]["time"] if base else None
            say(f"| {x['s_in']:.0f} | {x['R']} | {x['turn']} | {x['mu']} | "
                f"{''.join(per.get(i, []))} | {x['driven']} ({x['past_max']}) | {x['time']} | {bt} |")
    text = "\n".join(lines) + "\n"
    open(os.path.join(OUT, "report.md"), "w").write(text)
    print(text)


def main():
    if sys.argv[1:] == ["report"]:
        return report()
    ap = argparse.ArgumentParser()
    ap.add_argument("what")
    ap.add_argument("course", nargs="?")
    ap.add_argument("config", nargs="?")
    ap.add_argument("--cpus", default="")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--configs", default=",".join(CONFIGS))
    ap.add_argument("--courses", default=",".join(COURSES))
    a = ap.parse_args()
    if a.what == "run":
        cpus = [int(x) for x in a.cpus.split(",") if x] or None
        return run(a.course, a.config, cpus)
    return run_all(a.jobs, a.configs.split(","), a.courses.split(","))


if __name__ == "__main__":
    main()
