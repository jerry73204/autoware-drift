"""Roadmap 061 M4b's tables (docs/mode-choice-m4b.md) from its results and M4's.

Run: env -u PYTHONPATH .venv/bin/python studies/m4b_aggregate.py
Reads swb's play_log/race_planner/mode_choice/m4 (M4) and m4b (M4b: tagged by configuration,
`quiet/` the dry courses on an idle host); writes m4b/aggregate.md and aggregate.json. Every
result's configuration is read from its own JSON (tag, front/rear slip bounds), not its name.
"""
import glob
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
SWB = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
BASE = os.path.join(SWB, "play_log/race_planner/mode_choice")
M4, M4B = os.path.join(BASE, "m4"), os.path.join(BASE, "m4b")
ARMS = ("single", "sched", "always")
CLASSES = ("asphalt", "mud_wet", "snow_loose", "gravel", "snow_packed", "ice")


def rows(d):
    out = []
    for f in sorted(glob.glob(os.path.join(d, "*.json"))):
        if os.path.basename(f).startswith("aggregate"):
            continue
        r = json.load(open(f))
        r["_file"] = os.path.basename(f)
        r["_m4"] = d == M4
        out.append(r)
    return out


def cfg(r):
    """A result's configuration from its JSON: its tag, else M4's free planner (m4: M4's own
    runs; free: M4b's untagged runs, issue 001's rescue on the free planner)."""
    return r.get("tag") or ("m4" if r.get("_m4") else "free")


def stop(r):
    return r["result"] != "done"


def corners_table(rs, classes, cfgs, ref="m4"):
    L = ["| class | arm | config | runs | done | stops | failed re-plans | overruns | L1 p50 (median) "
         "| time vs M4 (corners both finished) | load (median) |", "|---" * 11 + "|"]
    out = {}
    for cls in classes:
        for arm in ARMS:
            base = {r["course"]: r for r in rs if cfg(r) == ref and r["arm"] == arm
                    and r["course"].startswith(f"c:{cls}:") and r["rep"] == 1}
            for c in cfgs:
                sel = [r for r in rs if cfg(r) == c and r["arm"] == arm
                       and r["course"].startswith(f"c:{cls}:") and r["rep"] == 1]
                if not sel:
                    continue
                both = [r for r in sel if not stop(r) and r["course"] in base
                        and not stop(base[r["course"]]) and r["t_timed"] and
                        base[r["course"]]["t_timed"]]
                dt = ([100 * (r["t_timed"] - base[r["course"]]["t_timed"])
                       / base[r["course"]]["t_timed"] for r in both])
                stops = sorted({r["result"].split(":")[0] for r in sel if stop(r)})
                x = {"runs": len(sel), "done": sum(not stop(r) for r in sel),
                     "failed": sum(r["failed"] for r in sel),
                     "replans": sum(r["replans"] for r in sel),
                     "overruns": sum(r["overruns"] for r in sel),
                     "p50": float(np.median([r["solve_p50"] or 0 for r in sel])),
                     "dt": float(np.mean(dt)) if dt else None, "n_both": len(both),
                     "load": float(np.median([r["load0"][0] for r in sel]))}
                out[f"{cls}/{arm}/{c}"] = x
                L.append(f"| {cls} | {arm} | {c} | {x['runs']} | {x['done']} | {', '.join(stops)} | "
                         f"{x['failed']} / {x['replans']} | {x['overruns']} | {x['p50']:.2f} | "
                         + (f"{x['dt']:+.2f} % ({x['n_both']})" if x["dt"] is not None else "-")
                         + f" | {x['load']:.1f} |")
    return L, out


def zones_table(rs, cfgs):
    L = ["| config | arm | map wrong | perc / est | result (s) | time (s) | failed / re-plans | "
         "solves / re-plan | estimate min | load |", "|---" * 10 + "|"]
    out = []
    for c in cfgs:
        sel = sorted([r for r in rs if r["course"] == "zones" and cfg(r) == c],
                     key=lambda r: (r["err"], r["seed"], r["arm"], r["perc"], r["est"], r["rep"]))
        for r in sel:
            res = r["result"] if r["result"] == "done" else f"{r['result'].split(':')[-1].strip()}@{r['s']:.0f}"
            out.append({"cfg": c, "arm": r["arm"], "err": r["err"], "seed": r["seed"],
                        "perc": r["perc"], "est": r["est"], "rep": r["rep"], "result": r["result"],
                        "s": r["s"], "t": r["t_course"], "wrong": r["info"].get("wrong"),
                        "estimate_min": r["estimate_min"]})
            L.append(f"| {c} | {r['arm']} | {r['err']} % s{r['seed']} {r['info'].get('wrong') or ''} | "
                     f"{'perc' if r['perc'] else '-'} / {'est' if r['est'] else '-'} | {res} | "
                     f"{r['t_course'] or '-'} | {r['failed']} / {r['replans']} | "
                     f"{r['per_replan']:.2f} | {r['estimate_min']:.2f} | "
                     f"{r['load0'][0]:.1f}-{r['load1'][0]:.1f} |")
    return L, out


def main():
    m4, m4b = rows(M4), rows(M4B)
    quiet = rows(os.path.join(M4B, "quiet"))
    agg = {}
    L = ["# 061 M4b: the aggregate (studies/m4b_aggregate.py)", "",
         "Configurations (each result's `tag`): m4, M4's free planner (swb m4/); fg1r3 / fin, "
         "the front at its peak and the rear within 3 x it on surfaces without fall-off "
         "(fin: and ice's rear at its peak, the first three stages free: M4b's defaults); "
         "a, the first stages free alone; rl1, a and the rear held at its peak on ice and "
         "packed snow. One repeat (r1) an arm unless said.", ""]
    L += ["## Dry courses, idle host (quiet/)", "",
          "| run | result | time (s) | solves / re-plan | G only | failed / overruns | load |",
          "|---|---|---|---|---|---|---|"]
    for r in quiet:
        L.append(f"| {r['_file'][:-5]} | {r['result']}@{r['s']:.0f} | {r['t_course'] or '-'} | "
                 f"{r['per_replan']:.2f} | {r['g_only']:.2f} | {r['failed']} / {r['overruns']} | "
                 f"{r['load0'][0]:.1f}-{r['load1'][0]:.1f} |")
    agg["quiet"] = [{k: r[k] for k in ("course", "arm", "rep", "result", "s", "t_course",
                                       "per_replan", "g_only", "failed", "overruns", "load0")}
                    for r in quiet]
    allc = [r for r in m4 + m4b if r["course"].startswith("c:")]
    for title, classes, cfgs in (
            ("Mud and loose snow: the slip bound", ("mud_wet", "snow_loose"), ("m4", "fin")),
            ("Ice and packed snow: the first stages free, the rear held", ("ice", "snow_packed"),
             ("m4", "a", "rl1"))):
        T, o = corners_table(allc, classes, cfgs)
        L += ["", f"## Corners: {title}", ""] + T
        agg.update(o)
    i001 = [r for r in m4b if r["course"] in ("c:mud_wet:9:180:15", "c:ice:9:180:6")
            and cfg(r) in ("free", "fg2", "fg2r3")]
    L += ["", "## Issue 001's corners (both repeats)", "",
          "| run | config | result | time (s) | first plan | load |", "|---|---|---|---|---|---|"]
    for r in i001:
        L.append(f"| {r['_file'][:-5]} | {cfg(r)} | {r['result']}@{r['s']:.0f} | "
                 f"{r['t_timed'] or '-'} | v0 {r['v0']:.1f} | {r['load0'][0]:.1f} |")
    zc = [c for c in ("m4", "fg2", "fg2r3", "fg15r3", "fg1r3", "fin") if any(
        r["course"] == "zones" and cfg(r) == c for r in m4 + m4b)]
    T, o = zones_table(m4 + m4b, zc)
    L += ["", "## The class-zone chain", ""] + T
    agg["zones"] = o
    sweep = [r for r in m4b if r["course"] == "zones" and (r.get("tag") or "").endswith("_fin")]
    if sweep:
        L += ["", "## Sweep (p_add, gain_min, sd), zone chain", "",
              "| tag | map | result | time (s) | solves / re-plan |", "|---|---|---|---|---|"]
        for r in sweep:
            L.append(f"| {r['tag']} | {r['err']} % s{r['seed']} | {r['result']}@{r['s']:.0f} | "
                     f"{r['t_course'] or '-'} | {r['per_replan']:.2f} |")
    open(os.path.join(M4B, "aggregate.md"), "w").write("\n".join(L) + "\n")
    json.dump(agg, open(os.path.join(M4B, "aggregate.json"), "w"), indent=1, default=str)
    print("\n".join(L))


if __name__ == "__main__":
    main()
