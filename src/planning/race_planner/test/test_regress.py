"""The regression gate (tools/regress.py): its comparison and its CLI's cheap exits. No solver
runs here; the gate itself is proved by `just regress-check` (README, "Before pushing to main")."""
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.join(HERE, "..", "tools")
REGRESS = os.path.join(TOOLS, "regress.py")
sys.path.insert(0, TOOLS)
import regress  # noqa: E402


def outcome(**kw):
    o = {"result": "done", "t": 12.5, "plans_digest": "a" * 64, "traj_digest": "b" * 64,
         "n_plans": 9, "error": None}
    o.update(kw)
    return o


def side(**over):
    s = {"asphalt_single": outcome(), "gravel_sched": outcome(t=14.0)}
    for k, kw in over.items():
        s[k] = outcome(**{**s[k], **kw})
    return s


def test_equal_outcomes_pass():
    x = side()
    assert regress.compare(x, x) == []


def test_a_digest_difference_names_the_run():
    base = side()
    head = side(gravel_sched={"plans_digest": "c" * 64})
    d = regress.compare(head, base)
    assert len(d) == 1
    assert "gravel_sched" in d[0] and "c" * 8 in d[0] and "a" * 8 in d[0]


def test_one_side_raising_fails():
    base = side()
    head = side(gravel_sched={"error": "Traceback (most recent call last)", "result": None})
    d = regress.compare(head, base)
    assert len(d) == 1
    assert "gravel_sched" in d[0] and "error" in d[0] and "done" in d[0]


def test_both_failing_alike_pass():
    x = side(gravel_sched={"result": "off_road"})
    y = side(gravel_sched={"result": "off_road"})
    assert regress.compare(x, y) == []


def _cli(*args, env=None):
    e = dict(os.environ)
    e.pop("PYTHONPATH", None)
    e.update(env or {})
    return subprocess.run([sys.executable, REGRESS, *args], capture_output=True, text=True,
                          env=e, timeout=60)


def test_missing_swb_is_a_setup_error():
    r = _cli("check", env={"SWB": "/nonexistent"})
    assert r.returncode == 2
    assert "/nonexistent" in r.stderr


def test_same_rev_is_nothing_to_compare():
    if regress.dirty(regress.repo_root()):
        import pytest
        pytest.skip("race_planner has uncommitted changes: HEAD is not the working tree")
    t0 = time.time()
    r = _cli("check", "--base", "HEAD",
             env={"SWB": os.environ.get("SWB", os.path.expanduser("~/repos/scenario-world-bridge"))})
    assert r.returncode == 0, r.stderr
    assert "nothing to compare" in r.stdout
    assert time.time() - t0 < 5


def test_both_raising_alike_fails():
    x = side(gravel_sched={"error": "X", "result": None})
    y = side(gravel_sched={"error": "X", "result": None})
    d = regress.compare(x, y)
    assert len(d) == 1
    assert "gravel_sched" in d[0] and "error" in d[0] and "X" in d[0]


def _gone(pid, within=5.0):
    t0 = time.time()
    while time.time() - t0 < within:
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return True
        try:                                  # a zombie of ours counts as gone once reaped
            if os.waitpid(pid, os.WNOHANG)[0] == pid:
                return True
        except ChildProcessError:
            pass
        time.sleep(0.05)
    return False


def test_an_interrupt_kills_the_runs_and_starts_no_more(tmp_path, monkeypatch):
    """A fake harness (a sleeper with a grandchild in its group): an interrupt in run_side kills
    both and never starts the queued run."""
    import signal
    import threading
    fake = tmp_path / "fake_harness.py"
    fake.write_text(
        "import os, subprocess, sys, time\n"
        "d = sys.argv[2]\n"
        "c = subprocess.Popen(['sleep', '60'])\n"
        "open(os.path.join(d, 'pids'), 'w').write(f'{os.getpid()} {c.pid}')\n"
        "time.sleep(60)\n")
    monkeypatch.setattr(regress, "HARNESS", str(fake))
    dirs = [tmp_path / f"r{i}" for i in range(3)]
    for d in dirs:
        d.mkdir()
    runs = [(f"r{i}", [str(d)]) for i, d in enumerate(dirs)]

    def interrupt_once_started():
        t0 = time.time()
        while time.time() - t0 < 20:
            if all((d / "pids").exists() for d in dirs[:2]):
                break
            time.sleep(0.05)
        os.kill(os.getpid(), signal.SIGINT)   # a real SIGINT, as Ctrl-C sends

    threading.Thread(target=interrupt_once_started, daemon=True).start()
    t0 = time.time()
    import pytest
    with pytest.raises(KeyboardInterrupt):
        regress.run_side(runs, None, str(tmp_path / "out"), jobs=2)
    assert time.time() - t0 < 20
    for d in dirs[:2]:
        for pid in map(int, (d / "pids").read_text().split()):
            assert _gone(pid), f"pid {pid} outlived the interrupt"
    assert not (dirs[2] / "pids").exists(), "a queued run started after the interrupt"
