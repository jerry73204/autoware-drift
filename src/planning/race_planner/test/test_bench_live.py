"""studies/bench_live.py: the L1/L2 timing study's summary, host record and CLI guards."""
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
STUDIES = os.path.join(HERE, "..", "studies")
sys.path.insert(0, STUDIES)
import bench_live  # noqa: E402

SCRIPT = os.path.join(STUDIES, "bench_live.py")


def _cli(env_extra, *args):
    env = {k: v for k, v in os.environ.items() if k not in ("RP_DET", "PYTHONPATH")}
    env.update(env_extra)
    return subprocess.run([sys.executable, SCRIPT, "run", "--courses", "c:gravel:15:90:8",
                           "--arm", "sched", "--label", "t", "--out", "/nonexistent-out", *args],
                          env=env, capture_output=True, text=True, timeout=60)


def test_summarise_counts_overruns():
    s = bench_live.summarise([0.5, 1.2, 0.9], [0.001, 0.03], 1.0, 0.02)
    assert s["l1_overruns"] == 1 and s["l2_overruns"] == 1
    assert s["l1_max"] == 1.2 and s["l1_n"] == 3 and s["l2_n"] == 2
    for k in ("l1_p50", "l1_p95", "l1_p99", "l2_p50", "l2_p95", "l2_p99", "l2_max"):
        assert k in s


def test_host_has_load_and_power_mode_keys():
    h = bench_live.host()
    for k in ("machine", "cpu_count", "loadavg", "nvpmodel", "jetson_clocks", "python", "casadi"):
        assert k in h
    assert len(h["loadavg"]) == 3
    if h["machine"] == "x86_64":
        assert h["nvpmodel"] is None


def test_refuses_rp_det():
    r = _cli({"RP_DET": "1", "SWB": os.environ.get("SWB", "/nonexistent")})
    assert r.returncode != 0
    assert "RP_DET" in r.stderr


def test_missing_swb_stops_before_running():
    r = _cli({"SWB": "/nonexistent"})
    assert r.returncode == 2
    assert "/nonexistent" in r.stderr


def _run_stubbed(monkeypatch, tmp_path, cpus):
    import types
    seen = {}

    def fake_run(name, **kw):
        seen.update(kw)
        return {"result": "done", "t": 1.0, "solve_all": [0.1]}

    stub = types.ModuleType("mode_choice_m4")
    stub.run = fake_run
    monkeypatch.setitem(sys.modules, "mode_choice_m4", stub)
    monkeypatch.setattr(os, "sched_setaffinity", lambda *a: None)
    monkeypatch.setenv("SWB", str(tmp_path))
    monkeypatch.delenv("RP_DET", raising=False)
    monkeypatch.setenv("M4_OUT", str(tmp_path))
    rc = bench_live.main(["run", "--courses", "c:gravel:15:90:8", "--arm", "sched", "--cpus", cpus,
                          "--label", "t", "--out", str(tmp_path / "out")])
    assert rc == 0
    return seen


def test_main_core_is_not_given_to_the_seed_workers(monkeypatch, tmp_path):
    assert _run_stubbed(monkeypatch, tmp_path, "8,9,10,11")["cpus"] == [9, 10, 11]


def test_single_core_list_leaves_workers_unpinned(monkeypatch, tmp_path):
    assert _run_stubbed(monkeypatch, tmp_path, "8")["cpus"] is None
