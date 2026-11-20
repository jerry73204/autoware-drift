"""The regression gate: race_planner at HEAD against a base revision, bit for bit.

Each run of the set (regress_set.yaml) is studies/mode_choice_m4.py's `run` with `--digest`,
under RP_DET=1, so a run is repeatable bit for bit and two trees that plan alike give equal
digests. The head side is this checkout as it stands (uncommitted changes included); the base
side is `git worktree add --detach` of the base revision, put first on sys.path with RP_ROOT.
Both sides use this checkout's studies/ harness, so what is compared is the race_planner
package only. A base side's results are cached under ~/.cache/race_planner/regress/<base sha>/
(keyed also by the harness, the set's argv and scensim's car and surface files), so a second check
pays for the head side only.

  regress.py check [--base origin/main] [--set FILE] [--jobs N]
      exit 0: every run the same; 1: a run differs; 2: a setup error (no scensim, a missing
      file, an unknown base).
  regress.py run [--base REV] [--set FILE] [--jobs N]
      one side's outcomes, for inspection. HEAD (the default) means this checkout as it
      stands; any other revision runs in a worktree.

The harness reads scensim's vehicle spec, surface classes and ECU (race_planner/scensim.py):
the pip-installed scensim's (`pip install --user scensim; scensim install`), or a checkout's
when $SWB names one. The set's courses are synthetic, so no course data is needed. The base
side runs its own race_planner with the same environment: one before race_planner/scensim.py
finds the ECU only in $SWB, so set SWB when the base predates it.
"""
import argparse
import concurrent.futures as cf
import hashlib
import json
import os
import shlex
import shutil
import signal
import subprocess
import sys
import tempfile
import threading
import time

import yaml

TOOLS = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(TOOLS)
STUDIES = os.path.join(PKG, "studies")
HARNESS = os.path.join(STUDIES, "mode_choice_m4.py")
PKG_REL = "src/planning/race_planner"
DEFAULT_SET = os.path.join(TOOLS, "regress_set.yaml")
CACHE = os.path.join(os.path.expanduser("~/.cache/race_planner"), "regress")
sys.path.insert(0, STUDIES)
from scensim_paths import scensim  # noqa: E402  (scensim from pip, or the $SWB checkout)

FIELDS = ("result", "t", "plans_digest", "traj_digest", "n_plans", "error")
RUN_TIMEOUT = 1800


class SetupError(Exception):
    pass


# ---- git ----

def _git(*args, cwd=None):
    r = subprocess.run(["git", *args], cwd=cwd or PKG, capture_output=True, text=True)
    if r.returncode != 0:
        raise SetupError(f"git {' '.join(args)}: {r.stderr.strip()}")
    return r.stdout.strip()


def repo_root():
    return _git("rev-parse", "--show-toplevel")


def resolve(rev):
    try:
        return _git("rev-parse", "--verify", "--quiet", f"{rev}^{{commit}}")
    except SetupError:
        raise SetupError(f"unknown base revision: {rev}") from None


def dirty(root):
    """race_planner has uncommitted changes (untracked files included)."""
    return bool(_git("status", "--porcelain", "--", PKG_REL, cwd=root))


# ---- the set ----

def load_set(path):
    """[(key, argv)]: argv is `mode_choice_m4.py run`'s arguments, `--digest` always among
    them. The file maps each key to its argument string."""
    if not os.path.isfile(path):
        raise SetupError(f"no such set file: {path}")
    with open(path) as fh:
        d = yaml.safe_load(fh) or {}
    runs = d.get("runs", d) if isinstance(d, dict) else None
    if not isinstance(runs, dict) or not runs:
        raise SetupError(f"{path}: expected a mapping `runs: {{key: argv}}`")
    out = []
    for key, args in runs.items():
        argv = shlex.split(args) if isinstance(args, str) else [str(a) for a in args]
        if "--digest" not in argv:
            argv.append("--digest")
        out.append((str(key), argv))
    return out


# ---- running a side ----

def _last_line(text):
    lines = [x for x in (text or "").splitlines() if x.strip()]
    return lines[-1].strip() if lines else ""


class _Live:
    """The process groups a side has started, so an interrupt can kill them: each run is in
    its own session (start_new_session), where a terminal's SIGINT never reaches it."""

    def __init__(self):
        self.lock = threading.Lock()
        self.procs = set()
        self.stopped = False

    def start(self, *a, **kw):
        with self.lock:
            if self.stopped:
                raise KeyboardInterrupt("the side was interrupted")
            p = subprocess.Popen(*a, **kw)
            self.procs.add(p)
            return p

    def done(self, p):
        with self.lock:
            self.procs.discard(p)

    def kill_all(self):
        with self.lock:
            self.stopped = True
            for p in self.procs:
                try:
                    os.killpg(p.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass


def run_one(key, argv, rp_root, out_dir, live=None):
    live = live or _Live()
    out = os.path.join(out_dir, key)
    shutil.rmtree(out, ignore_errors=True)
    os.makedirs(out)
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    env.pop("RP_ROOT", None)
    env.update(RP_DET="1", M4_OUT=out)
    if rp_root:
        env["RP_ROOT"] = rp_root
    t0 = time.time()
    p = live.start([sys.executable, HARNESS, "run", *argv], env=env, cwd=PKG,
                   stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
                   start_new_session=True)
    try:
        _, err = p.communicate(timeout=RUN_TIMEOUT)
    except subprocess.TimeoutExpired:
        os.killpg(p.pid, signal.SIGKILL)
        p.communicate()
        return dict({f: None for f in FIELDS}, error=f"timeout after {RUN_TIMEOUT} s",
                    wall=round(time.time() - t0, 1))
    finally:
        live.done(p)
    wall = round(time.time() - t0, 1)
    o = {f: None for f in FIELDS}
    o["wall"] = wall
    if p.returncode != 0:
        o["error"] = _last_line(err) or f"exit {p.returncode}"
        return o
    files = [f for f in os.listdir(out) if f.endswith(".json") and not f.endswith(".traj.json")]
    if len(files) != 1:
        o["error"] = f"expected one result file in {out}, found {files}"
        return o
    with open(os.path.join(out, files[0])) as fh:
        res = json.load(fh)
    for f in FIELDS[:-1]:
        o[f] = res.get(f)
    return o


def run_side(runs, rp_root, out_dir, jobs=2, label=""):
    """{key: Outcome}: each run in its own subprocess, `jobs` at a time."""
    res = {}
    live = _Live()
    ex = cf.ThreadPoolExecutor(max_workers=max(1, jobs))
    try:
        fut = {ex.submit(run_one, k, a, rp_root, out_dir, live): k for k, a in runs}
        for f in cf.as_completed(fut):
            k = fut[f]
            res[k] = f.result()
            print(f"  [{label}] {k}: {res[k]['result'] or res[k]['error']} "
                  f"({res[k]['wall']} s)", file=sys.stderr, flush=True)
    except BaseException:
        # Ctrl-C (or a run raising): no queued run starts, every started one's process group
        # dies, and the threads are joined before the caller removes the worktree under them
        live.kill_all()
        ex.shutdown(wait=True, cancel_futures=True)
        raise
    ex.shutdown(wait=True)
    return {k: res[k] for k, _ in runs}


# ---- the base side's cache ----

def _cache_dir(base_sha, runs):
    h = hashlib.sha256()
    for f in sorted(os.listdir(STUDIES)):
        if f.endswith(".py"):
            h.update(f.encode())
            with open(os.path.join(STUDIES, f), "rb") as fh:
                h.update(fh.read())
    for p in (scensim.spec("drift_mustang"), scensim.surface_classes()):
        with open(p, "rb") as fh:
            h.update(fh.read())
    h.update(json.dumps(runs).encode())
    return os.path.join(CACHE, base_sha, h.hexdigest()[:16])


def _cached(cdir, runs):
    got = {}
    for k, _ in runs:
        f = os.path.join(cdir, k + ".json")
        try:
            with open(f) as fh:
                got[k] = json.load(fh)
        except (OSError, ValueError):     # absent, or cut short by an earlier crash: a miss
            pass
    return got


def _store(cdir, outs):
    os.makedirs(cdir, exist_ok=True)
    for k, o in outs.items():
        if o.get("error") is None:          # a failure may be the host's: never cached
            f = os.path.join(cdir, k + ".json")
            fd, tmp = tempfile.mkstemp(dir=cdir, prefix=k + ".", suffix=".tmp")
            try:
                with os.fdopen(fd, "w") as fh:
                    json.dump(o, fh)
                os.replace(tmp, f)          # whole or not at all: a reader never sees half
            except BaseException:
                os.unlink(tmp)
                raise


def run_rev(runs, sha, root, tmp, jobs):
    """REV's outcomes, from the cache where it has them, else from a worktree of REV."""
    cdir = _cache_dir(sha, runs)
    got = _cached(cdir, runs)
    todo = [(k, a) for k, a in runs if k not in got]
    if todo:
        wt = os.path.join(tmp, "base")
        _git("worktree", "add", "--detach", wt, sha, cwd=root)
        try:
            outs = run_side(todo, os.path.join(wt, PKG_REL), os.path.join(tmp, "base-out"),
                            jobs, "base")
        finally:
            subprocess.run(["git", "worktree", "remove", "--force", wt], cwd=root,
                           capture_output=True)
            subprocess.run(["git", "worktree", "prune"], cwd=root, capture_output=True)
        _store(cdir, outs)
        got.update(outs)
    else:
        print(f"  [base] all {len(runs)} runs from the cache ({cdir})", file=sys.stderr)
    return {k: got[k] for k, _ in runs}


# ---- comparing ----

def _show(f, v):
    if v is not None and f.endswith("_digest"):
        return str(v)[:8]
    return repr(v)


def compare(head, base):
    """One string per run that differs, naming the run and both values; [] is a pass. A run
    that raised on either side is never a pass, even when both raised alike: the error is the
    gate's (a broken venv, an import error in the shared harness), not a planner result."""
    out = []
    for k in list(head) + [k for k in base if k not in head]:
        if k not in head or k not in base:
            out.append(f"{k}: only on the {'head' if k in head else 'base'} side")
            continue
        h, b = head[k], base[k]
        if h.get("error") is not None or b.get("error") is not None:
            out.append(f"{k}: error, the run raised: head={h.get('error')!r} "
                       f"(result {h.get('result')!r}) base={b.get('error')!r} "
                       f"(result {b.get('result')!r})")
            continue
        diffs = [f"{f} head={_show(f, h.get(f))} base={_show(f, b.get(f))}"
                 for f in FIELDS if h.get(f) != b.get(f)]
        if diffs:
            out.append(f"{k}: " + "; ".join(diffs))
    return out


def _d8(o, f):
    return (o.get(f) or "-")[:8]


def report(head, base):
    w = max(len(k) for k in head)
    print(f"{'run':<{w}}  {'result':<10} {'t':>9}  {'plans head/base':<17}  "
          f"{'traj head/base':<17}  verdict")
    for k, h in head.items():
        b = base.get(k, {})
        same = not compare({k: h}, {k: b})
        t = h.get("t")
        print(f"{k:<{w}}  {str(h.get('result') or 'error'):<10} "
              f"{(f'{t:.3f}' if isinstance(t, (int, float)) else '-'):>9}  "
              f"{_d8(h, 'plans_digest')}/{_d8(b, 'plans_digest')}  "
              f"{_d8(h, 'traj_digest')}/{_d8(b, 'traj_digest')}  {'same' if same else 'DIFF'}")


# ---- CLI ----

def _check_env(set_path):
    try:
        scensim.spec("drift_mustang"), scensim.surface_classes(), scensim.ecu_path()
    except FileNotFoundError as e:
        raise SetupError(str(e))
    return load_set(set_path)


def _tmpdir(root):
    d = os.path.join(root, ".regress")
    os.makedirs(d, exist_ok=True)
    return tempfile.mkdtemp(dir=d)


def cmd_check(a):
    runs = _check_env(a.set)
    root = repo_root()
    head_sha, base_sha = resolve("HEAD"), resolve(a.base)
    is_dirty = dirty(root)
    print(f"head: {head_sha[:12]} " + ("working tree (dirty)" if is_dirty else "(clean)"))
    print(f"base: {a.base} {base_sha[:12]}")
    if base_sha == head_sha and not is_dirty:
        print("nothing to compare: the base is HEAD and race_planner is clean")
        return 0
    tmp = _tmpdir(root)
    try:
        t0 = time.time()
        base = run_rev(runs, base_sha, root, tmp, a.jobs)
        head = run_side(runs, None, os.path.join(tmp, "head-out"), a.jobs, "head")
        print(f"({len(runs)} runs, {time.time() - t0:.0f} s)")
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    report(head, base)
    diffs = compare(head, base)
    for d in diffs:
        print("DIFF " + d)
    print("FAIL" if diffs else "PASS")
    return 1 if diffs else 0


def cmd_run(a):
    runs = _check_env(a.set)
    root = repo_root()
    sha = resolve(a.base)
    here = sha == resolve("HEAD")
    print(f"side: {a.base} {sha[:12]}" + ((" working tree (dirty)" if dirty(root) else " (clean)")
                                          if here else ""))
    tmp = _tmpdir(root)
    try:
        outs = (run_side(runs, None, os.path.join(tmp, "out"), a.jobs, "head") if here
                else run_rev(runs, sha, root, tmp, a.jobs))
    finally:
        shutil.rmtree(tmp, ignore_errors=True)
    for k, o in outs.items():
        print(k, json.dumps({f: o.get(f) for f in FIELDS + ("wall",)}))
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name, base in (("check", "origin/main"), ("run", "HEAD")):
        p = sub.add_parser(name)
        p.add_argument("--base", default=base)
        p.add_argument("--set", default=DEFAULT_SET)
        p.add_argument("--jobs", type=int, default=2)
    a = ap.parse_args(argv)
    sys.stdout.reconfigure(line_buffering=True)
    try:
        return cmd_check(a) if a.cmd == "check" else cmd_run(a)
    except SetupError as e:
        print(f"regress: {e}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
