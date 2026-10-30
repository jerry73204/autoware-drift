"""M4's batches (studies/mode_choice_m4.py), at most four processes at once on cores of their own:
one-process jobs (single start, the oracle) four at a time, one core each; multi-start jobs one
at a time over all four (the driver on the first, the seeds on the rest). A job whose result is
already there is skipped, so a batch can be re-run.

Run: env -u PYTHONPATH .venv/bin/python studies/m4_batch.py <batch> [--cores 8,9,10,11]
"""
import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import mode_choice_m4 as m4  # noqa: E402

PY = sys.executable


def jobs(batch, reps=(1, 2), beta=45.0):
    """[(kind, argv, result path)]: kind 1 one process, 4 multi-start."""
    out = []

    def add(course, arm, err=0, seed=0, perc=False, est=False, rep=1, tag="", extra=()):
        key = m4.run_key(course, arm, err, seed, perc, est, rep, tag)
        argv = ["run", course, "--arm", arm, "--err", str(err), "--seed", str(seed),
                "--rep", str(rep)] + (["--perc"] if perc else []) + (["--est"] if est else []) \
            + (["--tag", tag] if tag else []) + list(extra)
        out.append((1 if arm == "single" else 4, argv, os.path.join(m4.OUT, key + ".json")))

    if batch == "oracle":
        for c in m4.corner_courses():
            out.append((1, ["oracle", c], os.path.join(m4.OUT, "oracle",
                                                       c.replace(":", "_") + ".json")))
    elif batch == "corners":
        for rep in (1, 2):
            for c in m4.corner_courses():
                for arm in ("single", "sched", "always"):
                    add(c, arm, rep=rep)
    elif batch == "zones":
        for rep in (1, 2):
            for arm in ("single", "always", "sched"):
                add("zones", arm, rep=rep)
            add("zones", "sched", est=True, rep=rep)
        for err in (10, 30):
            for seed in (1, 2, 3):
                for arm in ("single", "always", "sched"):
                    add("zones", arm, err, seed)
                add("zones", "sched", err, seed, perc=True)
                add("zones", "sched", err, seed, perc=True, est=True)
    elif batch == "dry":
        for rep in reps:
            for c in ("haruna", "tsukuba"):
                for arm in ("single", "sched", "always"):
                    add(c, arm, rep=rep)
    elif batch == "beta":         # L1's sideslip bound on the deep-slide classes' corners
        for rep in reps:
            for c in m4.corner_courses():
                if c.split(":")[1] in ("mud_wet", "snow_loose"):
                    for arm in ("single", "sched"):
                        add(c, arm, rep=rep, tag=f"b{beta:g}", extra=["--beta", str(beta)])
        bx = ["--beta", str(beta)]
        for rep in reps:              # the zone chain, true map and 10 % wrong
            for arm in ("single", "always", "sched"):
                add("zones", arm, rep=rep, tag=f"b{beta:g}", extra=bx)
        for seed in (1, 2, 3):
            for arm in ("single", "always", "sched"):
                add("zones", arm, 10, seed, tag=f"b{beta:g}", extra=bx)
            add("zones", "sched", 10, seed, perc=True, est=True, tag=f"b{beta:g}", extra=bx)
    elif batch == "sweep":
        for tag, extra in (("p005", ["--rules", '{"p_add": 0.05}']),
                           ("p030", ["--rules", '{"p_add": 0.3}']),
                           ("g010", ["--rules", '{"gain_min": 0.1, "gain_falloff": 0.1}']),
                           ("g050", ["--rules", '{"gain_min": 0.5, "gain_falloff": 0.5}']),
                           ("sd01", ["--sd", "0.1"])):
            add("zones", "sched", 10, 1, tag=tag, extra=extra)
            add("zones", "sched", 0, 0, tag=tag, extra=extra)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("batch")
    ap.add_argument("--cores", default="8,9,10,11")
    ap.add_argument("--max-load", type=float, default=16.0,
                    help="a job starts only when the host's 1-min load is below this (timing "
                         "under others' load cuts solves at the budget)")
    ap.add_argument("--reps", default="1,2")
    ap.add_argument("--beta", type=float, default=45.0, help="batch beta: the bound, deg")
    a = ap.parse_args()
    cores = [int(x) for x in a.cores.split(",")]
    todo = [j for j in jobs(a.batch, tuple(int(x) for x in a.reps.split(",")), a.beta) if not os.path.exists(j[2])]
    print(len(todo), "jobs", flush=True)
    os.makedirs(m4.OUT, exist_ok=True)
    logdir = os.path.join(m4.OUT, "logs")
    os.makedirs(logdir, exist_ok=True)
    free, running = list(cores), {}

    def reap(block):
        while running:
            for p, (cs, j) in list(running.items()):
                if p.poll() is not None:
                    free.extend(cs)
                    del running[p]
                    return
            if not block:
                return
            time.sleep(0.5)

    def launch(j, cs):
        name = os.path.basename(j[2])[:-5]
        log = open(os.path.join(logdir, name + ".log"), "w")
        p = subprocess.Popen([PY, os.path.join(HERE, "mode_choice_m4.py")] + j[1]
                             + ["--cpus", ",".join(map(str, cs))], stdout=log,
                             stderr=subprocess.STDOUT, env=dict(os.environ, OMP_NUM_THREADS="1"))
        running[p] = (cs, j)
        for c in cs:
            free.remove(c)

    for j in sorted(todo, key=lambda j: j[0]):      # one-process jobs first
        need = len(cores) if j[0] == 4 else 1
        while len(free) < need:
            reap(True)
        while os.getloadavg()[0] >= a.max_load:     # others' work on the host: wait it out
            reap(False)
            time.sleep(15)
        launch(j, sorted(free)[:need])
    while running:
        reap(True)
    print("done", flush=True)


if __name__ == "__main__":
    main()
