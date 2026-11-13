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
