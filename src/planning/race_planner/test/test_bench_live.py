"""studies/bench_live.py: the L1/L2 timing study's summary, host record and CLI guards."""
import json
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
    for k in ("machine", "cpu_count", "loadavg", "nvpmodel", "clocks", "python", "casadi"):
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


def _run_stubbed(monkeypatch, tmp_path, cpus, extra=()):
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
    monkeypatch.setattr(bench_live, "host", lambda: {})
    cpu_arg = [] if cpus is None else ["--cpus", cpus]
    rc = bench_live.main(["run", "--courses", "c:gravel:15:90:8", "--arm", "sched", *cpu_arg,
                          "--label", "t", "--out", str(tmp_path / "out"), *extra])
    assert rc == 0
    seen["json"] = json.loads((tmp_path / "out" / "t.json").read_text())
    return seen


def test_main_core_is_not_given_to_the_seed_workers(monkeypatch, tmp_path):
    assert _run_stubbed(monkeypatch, tmp_path, "8,9,10,11")["cpus"] == [9, 10, 11]


def test_single_core_list_leaves_workers_unpinned(monkeypatch, tmp_path):
    assert _run_stubbed(monkeypatch, tmp_path, "8")["cpus"] is None


def test_no_cpus_leaves_everything_unpinned(monkeypatch, tmp_path):
    assert _run_stubbed(monkeypatch, tmp_path, None)["cpus"] is None


def test_empty_cpus_leaves_everything_unpinned(monkeypatch, tmp_path):
    assert _run_stubbed(monkeypatch, tmp_path, "")["cpus"] is None


def _sysfs(root, cpu, gpu):
    for name, (lo, hi, cur) in cpu.items():
        d = root / "sys/devices/system/cpu/cpufreq" / name
        d.mkdir(parents=True)
        for f, v in (("scaling_min_freq", lo), ("scaling_max_freq", hi), ("scaling_cur_freq", cur)):
            (d / f).write_text(f"{v}\n")
    for name, (lo, hi, cur) in gpu.items():
        d = root / "sys/class/devfreq" / name
        d.mkdir(parents=True)
        for f, v in (("min_freq", lo), ("max_freq", hi), ("cur_freq", cur)):
            (d / f).write_text(f"{v}\n")


def test_clocks_read_from_sysfs_without_root(tmp_path):
    _sysfs(tmp_path, {"policy0": (2201600, 2201600, 2201600), "policy4": (115200, 2201600, 729600)},
           {"17000000.gpu": (306000000, 1300500000, 306000000), "15340000.vic": (1, 2, 1)})
    c = bench_live.clocks(str(tmp_path))
    assert c == {
        "cpu": {"policy0": {"min": 2201600, "max": 2201600, "cur": 2201600},
                "policy4": {"min": 115200, "max": 2201600, "cur": 729600}},
        "gpu": {"17000000.gpu": {"min": 306000000, "max": 1300500000, "cur": 306000000}},
        "cpu_pinned": False, "gpu_pinned": False}


def test_clocks_pinned_when_min_equals_max(tmp_path):
    _sysfs(tmp_path, {"policy0": (5, 5, 5)}, {"17000000.gpu": (9, 9, 9)})
    c = bench_live.clocks(str(tmp_path))
    assert c["cpu_pinned"] is True and c["gpu_pinned"] is True


def test_clocks_missing_paths_are_none(tmp_path):
    assert bench_live.clocks(str(tmp_path)) == {"cpu": None, "gpu": None,
                                                "cpu_pinned": None, "gpu_pinned": None}


def test_length_and_budget_reach_the_driver_and_the_json(monkeypatch, tmp_path):
    made = {}

    class FakeDriver:
        def __init__(self, *a, **kw):
            made.update(kw)

    import types
    stub_holder = {}

    def fake_run(name, **kw):
        stub_holder["m4"].Driver()             # m4.run constructs its Driver from the module global
        return {"result": "done", "t": 1.0, "solve_all": [0.6]}

    stub = types.ModuleType("mode_choice_m4")
    stub.run = fake_run
    stub.Driver = FakeDriver
    stub_holder["m4"] = stub
    monkeypatch.setitem(sys.modules, "mode_choice_m4", stub)
    monkeypatch.setattr(os, "sched_setaffinity", lambda *a: None)
    monkeypatch.setattr(bench_live, "host", lambda: {})
    monkeypatch.setenv("SWB", str(tmp_path))
    monkeypatch.delenv("RP_DET", raising=False)
    rc = bench_live.main(["run", "--courses", "c:gravel:15:90:8", "--label", "t",
                          "--out", str(tmp_path / "out"), "--length", "120", "--budget", "0.5"])
    assert rc == 0
    assert made == {"length": 120.0, "budget": 0.5}
    out = json.loads((tmp_path / "out" / "t.json").read_text())
    assert out["length"] == 120.0 and out["budget"] == 0.5
    assert out["all"]["l1_overruns"] == 1          # 0.6 s against the 0.5 s budget


def test_default_length_and_budget_leave_the_driver_alone(monkeypatch, tmp_path):
    out = _run_stubbed(monkeypatch, tmp_path, "8")["json"]
    assert out["length"] is None and out["budget"] == 1.0
