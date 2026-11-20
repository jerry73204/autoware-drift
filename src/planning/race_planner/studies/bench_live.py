"""L1/L2 timing of the live loop on this machine (race stack, phase 1).

Runs studies/mode_choice_m4.py's closed loop (live.Driver re-plans = L1, tracker.Tracker = L2,
sim.Plant drives) with the wall clock ON and reports the two layers' solve times:

- L1: each re-plan's wall seconds (the run's `solve_all`), against the re-plan budget (1 s);
- L2: each Tracker.step, timed here with time.perf_counter() around the call (our own wrapper,
  not the tracker's internal `solve_s`), against the control tick (0.02 s). L2 runs in the main
  process; L1's seed solves run in spawned workers, which import the original class.

Run:  env -u PYTHONPATH .venv/bin/python studies/bench_live.py run
          --courses c:asphalt:25:90:8,c:gravel:15:90:8,zones --arm sched --cpus 8,9,10,11
          --label NAME --out DIR [--length M] [--budget S]
Writes DIR/<label>.json = {host, label, length, budget, per_course, all}. Refuses RP_DET=1 (that run
has no clock). --length and --budget are live.Driver's `length` (L1's window, m; default the
Driver's) and `budget` (L1's re-plan budget and period, s), set on the Driver m4.run builds.
"""
import argparse
import glob
import json
import os
import platform
import subprocess
import sys
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
BUDGET = 1.0     # L1's re-plan budget, s
TICK = 0.02      # L2's control tick, s


def _stats(prefix, xs, limit):
    a = np.asarray(xs, dtype=float)
    if a.size == 0:
        return {f"{prefix}_{k}": None for k in ("p50", "p95", "p99", "max")} | \
               {f"{prefix}_overruns": 0, f"{prefix}_n": 0}
    return {f"{prefix}_p50": float(np.percentile(a, 50)),
            f"{prefix}_p95": float(np.percentile(a, 95)),
            f"{prefix}_p99": float(np.percentile(a, 99)),
            f"{prefix}_max": float(a.max()),
            f"{prefix}_overruns": int((a > limit).sum()),
            f"{prefix}_n": int(a.size)}


def summarise(l1, l2, budget, tick):
    """Percentiles, max, overruns (L1 > budget, L2 > tick) and count of each layer's times."""
    return {**_stats("l1", l1, budget), **_stats("l2", l2, tick)}


def _read_int(path):
    try:
        with open(path) as fh:
            return int(fh.read().strip())
    except (OSError, ValueError):
        return None


def _freqs(d, names):
    got = {k: _read_int(os.path.join(d, f)) for k, f in zip(("min", "max", "cur"), names)}
    return got if any(v is not None for v in got.values()) else None


def _pinned(table):
    if not table:
        return None
    return all(v["min"] is not None and v["min"] == v["max"] for v in table.values())


def clocks(root="/"):
    """CPU and GPU clock ranges from sysfs, readable without root (jetson_clocks --show is not).
    jetson_clocks pins by raising the floor to the ceiling, so *_pinned is min == max on every
    policy / GPU devfreq node. cpu: kHz per cpufreq policy; gpu: Hz per devfreq node named *gpu*.
    A missing tree is None."""
    cpu = {}
    for d in sorted(glob.glob(os.path.join(root, "sys/devices/system/cpu/cpufreq/policy*"))):
        f = _freqs(d, ("scaling_min_freq", "scaling_max_freq", "scaling_cur_freq"))
        if f:
            cpu[os.path.basename(d)] = f
    gpu = {}
    for d in sorted(glob.glob(os.path.join(root, "sys/class/devfreq/*gpu*"))):
        f = _freqs(d, ("min_freq", "max_freq", "cur_freq"))
        if f:
            gpu[os.path.basename(d)] = f
    cpu, gpu = cpu or None, gpu or None
    return {"cpu": cpu, "gpu": gpu, "cpu_pinned": _pinned(cpu), "gpu_pinned": _pinned(gpu)}


def host():
    try:
        import casadi
        cas = casadi.__version__
    except Exception:
        cas = None
    nv = None
    try:
        r = subprocess.run(["nvpmodel", "-q"], capture_output=True, text=True, timeout=10)
        nv = r.stdout.strip() if r.returncode == 0 and r.stdout.strip() else None
    except (OSError, subprocess.SubprocessError):
        pass
    return {"machine": platform.machine(), "cpu_count": os.cpu_count(),
            "loadavg": list(os.getloadavg()), "nvpmodel": nv,
            "clocks": clocks(),
            "python": platform.python_version(), "casadi": cas}


def _bench(a):
    os.makedirs(a.out, exist_ok=True)
    os.environ["M4_OUT"] = os.path.join(a.out, "m4")      # before m4 is imported (OUT is read then)
    sys.path.insert(0, HERE)
    import mode_choice_m4 as m4
    from race_planner.tracker import Tracker
    cpus = [int(x) for x in (a.cpus or "").split(",") if x] or None
    if cpus:
        os.sched_setaffinity(0, {cpus[0]})
    over = {k: v for k, v in (("length", a.length), ("budget", a.budget)) if v is not None}
    budget = a.budget if a.budget is not None else BUDGET
    orig_driver = getattr(m4, "Driver", None)
    if over and orig_driver is not None:
        def driver(*args, **kw):      # m4.run builds its Driver from this module global
            return orig_driver(*args, **{**kw, **over})
        m4.Driver = driver
    l2 = []
    orig = Tracker.step

    def timed(self, *args, **kw):
        c = time.perf_counter()
        try:
            return orig(self, *args, **kw)
        finally:
            l2.append(time.perf_counter() - c)

    Tracker.step = timed
    per, all1, all2 = {}, [], []
    try:
        for name in [c for c in a.courses.split(",") if c]:
            del l2[:]
            # [1:]: the seed workers' cores, as m4's driver; cpus[0] is this process's
            res = m4.run(name, arm=a.arm, cpus=(cpus[1:] or None) if cpus else None, tag=a.label)
            l1 = res.get("solve_all") or []
            per[name] = {**summarise(l1, l2, budget, TICK), "result": res.get("result"),
                         "t": res.get("t")}
            all1 += l1
            all2 += l2
    finally:
        Tracker.step = orig
        if over and orig_driver is not None:
            m4.Driver = orig_driver
    out = {"host": host(), "label": a.label, "length": a.length, "budget": budget,
           "per_course": per, "all": summarise(all1, all2, budget, TICK)}
    path = os.path.join(a.out, a.label + ".json")
    with open(path, "w") as fh:
        json.dump(out, fh, indent=1)
    print(path, json.dumps(out["all"]), flush=True)
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="what", required=True)
    r = sub.add_parser("run")
    r.add_argument("--courses", required=True, help="comma list of m4 course names")
    r.add_argument("--arm", default="sched")
    r.add_argument("--cpus", default="")
    r.add_argument("--length", type=float, default=None,
                   help="L1's window length, m (live.Driver.length; default the Driver's 200)")
    r.add_argument("--budget", type=float, default=None,
                   help=f"L1's re-plan budget and period, s (live.Driver.budget; default {BUDGET})")
    r.add_argument("--label", required=True)
    r.add_argument("--out", required=True)
    a = ap.parse_args(argv)
    if os.environ.get("RP_DET") == "1":
        print("bench_live: RP_DET=1 is set; that run has no clock, so it measures nothing. "
              "Unset it.", file=sys.stderr)
        return 1
    try:   # the car and the surfaces, from scensim (pip, or the $SWB checkout)
        from scensim_paths import scensim
        scensim.spec("drift_mustang"), scensim.surface_classes()
    except FileNotFoundError as e:
        print(f"bench_live: {e}", file=sys.stderr)
        return 2
    return _bench(a)


if __name__ == "__main__":
    sys.exit(main())
