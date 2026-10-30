"""L1's starts chosen from what lies ahead: the receding-chain check (roadmap 061 M3b;
docs/mode-choice-m3b.md).

As E1 (studies/mode_choice_e1.py): live.Driver re-plans along a course while the car follows the
adopted plan exactly (sim.PlanFollower: no plant, no tracker, no model error), so what is measured
is the planner alone: which starts ran at each re-plan, how many solves that took, whether the
course was finished, and its time on the plans adopted.

Courses:
- `haruna`, `tsukuba` (one lap, unrolled): swb's courses, dry asphalt;
- `zones`: E1's chain of the drift study's corners (R 9-60 m, 90 and 180 deg, left and right in
  turn, 120 m straights, 8 m wide) on dry asphalt, each corner in a zone of one of swb's classes
  (ZONES: from APPROACH m before its turn-in to 20 m past its exit), the map right (declared =
  true); one zone declared asphalt at confidence 0.85 with mud_wet the alternative.

Configs: `single` (060's one start), `sched` (the scheduler: schedule.Scheduler, G in the
driver's process, H D F in theirs), `always` (G, H, D every re-plan, F into hairpins), all on the
planner's track the map gives (surface.SurfaceMap.planner_track: the lower quantile).

Run: env -u PYTHONPATH .venv/bin/python studies/mode_choice_m3b.py run <course> <config>
     [--cpus 8,9,10]; `report` (report.md).
Results: swb's play_log/race_planner/mode_choice/m3b/.
"""
import argparse
import json
import math
import os
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

from race_planner import live, modes  # noqa: E402
from race_planner.car import Car  # noqa: E402
from race_planner.live import Driver  # noqa: E402
from race_planner.multistart import MultiStart  # noqa: E402
from race_planner.schedule import Scheduler  # noqa: E402
from race_planner.sim import NoTracker, PlanFollower  # noqa: E402
from race_planner.surface import SurfaceMap, Zone  # noqa: E402
from race_planner.track import SurfaceTable, Track  # noqa: E402
import mode_choice_e1 as e1  # noqa: E402

SWB = e1.SWB
OUT = os.environ.get("M3B_OUT", os.path.join(SWB, "play_log/race_planner/mode_choice/m3b"))
GRIP, DS, DT, SEED_BUDGET = e1.GRIP, e1.DS, e1.DT, e1.SEED_BUDGET
APPROACH = 60.0
SPIN_DEG = 89.0       # deg: the follower stops past this sideslip (live's 60 is the plant's)
# the zones course: each of E1's CHAIN corners in order, its class (declared = true)
ZONES = ("asphalt?", "mud_wet", "snow_loose", "gravel", "ice", "snow_loose", "snow_packed",
         "asphalt", "mud_wet", "asphalt")
COURSES = ("haruna", "tsukuba", "zones")
CONFIGS = ("single", "sched", "always")


def table():
    return SurfaceTable.from_yaml(e1.CLASSES)


def course(name, car):
    """(planner's track, surface map or None) for a course."""
    tab = table()
    if name in ("haruna", "tsukuba"):
        t = Track.from_swb_course(os.path.join(SWB, "data/courses", name), tab, car)
        if t.closed:
            t = t.unrolled(1)
        return t.resampled(DS), None
    base = e1.chain(1.0).resampled(DS)       # geometry only: the map sets every surface
    zones = []
    for c, cls in zip(modes.corners(base, GRIP), ZONES):
        if cls.endswith("?"):
            zones.append(Zone(c.s_in - APPROACH, c.s_out + 20.0, cls[:-1], 0.85, None,
                              ("mud_wet",)))
        else:
            zones.append(Zone(c.s_in - APPROACH, c.s_out + 20.0, cls))
    smap = SurfaceMap(tab, car, zones)
    return smap.planner_track(base), smap


def expected(track, smap, car):
    """The table's starts for each corner (the scheduler's rows, the corner seen 25 m+ ahead)."""
    sch = Scheduler(track, car, table(), smap=smap, grip=GRIP)
    out = []
    for c in sch.corners:
        scn = sch.scan(c.s_in - 100.0, c.s_in + 160.0)
        ca = next(x for x in scn.corners if x.corner.s_in == c.s_in)
        add, why = sch.corner_starts(ca)
        out.append({"s_in": c.s_in, "R": round(c.R, 1), "turn": round(math.degrees(c.turn)),
                    "starts": "".join(k for k in "HDF" if k in add), "why": why,
                    "probs": {k: round(v, 2) for k, v in ca.probs.items() if v > 0}})
    return out


def run(course_name, config, cpus=None):
    # the spin stop is the plant's safety rule (a car past 60 deg of sideslip); with no plant the
    # car is its plan, and on mud M2's tyre plans hairpins past it (single: stopped at s 248)
    live.SPIN_BETA = math.radians(SPIN_DEG)
    car = Car.from_spec(e1.SPEC)
    track, smap = course(course_name, car)
    ms = None
    if config != "single":
        ms = {"threshold": 0.003, "seed_budget": SEED_BUDGET, "cpus": cpus,
              "schedule": {"table": table(), "smap": smap, "always": config == "always"}}
    d = Driver(car, track, NoTracker(), ds=DS, grip=GRIP, multistart=ms)
    cs = modes.corners(track, GRIP)
    s_end = float(track.s[-1]) - 10.0
    f = PlanFollower(d, 10.0, 12.0)
    st = f.state()
    c0 = time.time()
    d.start(st)
    result = "timeout"
    while st["t"] < 900.0:
        d.step(st)
        if d.stopping:
            result = "stopped: " + d.stopping
            break
        st = f.step(DT)
        if st["s"] >= s_end:
            result = "done"
            break
    wall = time.time() - c0
    d.close()
    out = {"course": course_name, "config": config, "result": result, "t": round(st["t"], 3),
           "s": round(st["s"], 1), "s_end": s_end, "wall": round(wall, 1),
           "solves": [{k: (round(v, 4) if isinstance(v, float) else v) for k, v in x.items()}
                      for x in d.solves],
           "records": d.multi.records if d.multi is not None else None,
           "corners": [{"s_in": c.s_in, "s_out": c.s_out, "R": round(c.R, 1),
                        "turn": round(math.degrees(c.turn))} for c in cs],
           "expected": expected(track, smap, car)}
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, f"{course_name}_{config}.json"), "w") as fh:
        json.dump(out, fh, default=lambda o: o.item() if hasattr(o, "item") else str(o))
    print(course_name, config, result, out["t"], "s", len(d.solves), "re-plans", round(wall),
          "s wall", flush=True)
    return out


def _pct(a, q):
    return round(float(np.percentile(a, q)), 3) if len(a) else None


def report():
    rows = {}
    for c in COURSES:
        for k in CONFIGS:
            p = os.path.join(OUT, f"{c}_{k}.json")
            if os.path.exists(p):
                rows[(c, k)] = json.load(open(p))
    L = ["# 061 M3b: the receding-chain check (no plant)", ""]
    L += ["| course | config | result | time (s) | vs single | re-plans | solves / re-plan | "
          "G only | with H | with D | with F | wall p50/p95/max | overruns > 1.0 s |",
          "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for (c, k), r in rows.items():
        base = rows.get((c, "single"))
        walls = np.array([x["solve_s"] for x in r["solves"][1:]])
        recs = r["records"] or []
        n = len(recs) if recs else len(r["solves"])
        sol = (np.mean([len(x["slots"]) for x in recs]) if recs else 1.0)
        cnt = lambda s: sum(1 for x in recs if s in x["slots"])
        g_only = sum(1 for x in recs if x["slots"] == ["G"]) if recs else n
        vs = (f"{100 * (r['t'] - base['t']) / base['t']:+.3f} %"
              if base and k != "single" and base["result"] == "done" and r["result"] == "done"
              else "")
        L.append(f"| {c} | {k} | {r['result']} | {r['t']:.2f} | {vs} | {n} | {sol:.2f} | "
                 f"{g_only} ({100 * g_only / max(n, 1):.0f} %) | {cnt('H')} | {cnt('D')} | "
                 f"{cnt('F')} | {_pct(walls, 50)}/{_pct(walls, 95)}/{walls.max():.2f} | "
                 f"{int((walls > 1.0).sum())} |")
    for c in COURSES:
        r = rows.get((c, "sched"))
        if r is None:
            continue
        L += ["", f"## {c}: the starts the table asks for, and what ran (sched)", "",
              "Ran: the starts run on the re-plans whose choice was about the corner (its turn-in "
              "10 m or more ahead), with how many re-plans each.", "",
              "| s_in | R | turn | classes (p) | table | why | ran |", "|---|---|---|---|---|---|---|"]
        for e in r["expected"]:
            ran = {}
            for x in r["records"]:
                if x.get("corner") == e["s_in"]:
                    key = "".join(k for k in "GHDF" if k in x["slots"])
                    ran[key] = ran.get(key, 0) + 1
            pr = ", ".join(f"{k} {v}" for k, v in e["probs"].items())
            L.append(f"| {e['s_in']:.0f} | {e['R']} | {e['turn']} | {pr} | G{e['starts']} | "
                     f"{'; '.join(e['why'])} | "
                     f"{', '.join(f'{k} x{v}' for k, v in sorted(ran.items()))} |")
        L += ["", "Mode changes: " + str([(round(x["s0"]), x["mode_before"], x["mode"])
                                         for x in r["records"] if x["mode"] != x["mode_before"]])]
    text = "\n".join(L) + "\n"
    open(os.path.join(OUT, "report.md"), "w").write(text)
    print(text)


def main():
    if sys.argv[1:] == ["report"]:
        return report()
    ap = argparse.ArgumentParser()
    ap.add_argument("what")
    ap.add_argument("course")
    ap.add_argument("config")
    ap.add_argument("--cpus", default="")
    a = ap.parse_args()
    cpus = [int(x) for x in a.cpus.split(",") if x] or None
    return run(a.course, a.config, cpus)


if __name__ == "__main__":
    main()
