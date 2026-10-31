"""The style record (style.py, roadmap 065 M1): validation that refuses by name, the default
(fastest), the planner refusing a style it cannot run yet, a live change deferred near a
committed drift, and S1 with the record bit for bit S1 without it."""
import dataclasses
import json
import math
import subprocess
import sys
import types

import numpy as np
import pytest

from race_planner import modes, style as style_mod, window
from race_planner.car import Car
from race_planner.live import Driver
from race_planner.multistart import AHEAD, CORNER_FROM, MultiStart
from race_planner.serve import style_message
from race_planner.sim import NoTracker, PlanFollower
from race_planner.style import FASTEST, Style, as_style
from test_ocp import SPEC, road


# ---- validation ----

REFUSED = [
    ({"colour": "red"}, "colour"),                                   # unknown
    ({"grip": 0.95}, "grip"),                                        # invariants, not fields
    ({"rear_slip_max": 5.0}, "rear_slip_max"),
    ({"free_stages": 0}, "free_stages"),
    ({"edge_margin": 0.1}, "edge_margin"),
    ({"name": "d1"}, "name"),
    ({"name": "drift_budget", "eps_window": 0.001}, "eps_window"),   # below the noise
    ({"name": "drift_budget", "eps_window": 0.05}, "eps_window"),    # above a lap's worth
    ({"name": "drift_budget", "eps_lap_s": -0.1}, "eps_lap_s"),
    ({"name": "drift_max", "classes": ["mud_wet", "ice"]}, "classes"),
    ({"name": "drift_max", "classes": ["mud_wet", "mud_wet"]}, "classes"),
    ({"name": "drift_max", "speed_floor": 1.0}, "speed_floor"),
    ({"name": "drift_max", "width_use": 1.2}, "width_use"),
    ({"name": "drift_max", "straights": "yes"}, "straights"),
    ({"name": "drift_max", "swing_rate_max": 0.0}, "swing_rate_max"),
    ({"name": "drift_max", "slip_floor_past_deg": float("nan")}, "slip_floor_past_deg"),
    ({"eps_lap_s": 0.2}, "eps_lap_s"),                               # S2/S3 fields under S1
    ({"eps_window": 0.01}, "eps_window"),
    ({"drift_past_deg": 5.0}, "drift_past_deg"),
    ({"straights": False}, "straights"),
    ({"classes": ["mud_wet"]}, "classes"),
    ({"name": "drift_budget", "speed_floor": 0.2}, "speed_floor"),   # S3's under S2
    ({"name": "drift_max", "eps_lap_s": 0.2}, "eps_lap_s"),          # S2's under S3
]


@pytest.mark.parametrize("record,field", REFUSED)
def test_a_bad_record_is_refused_naming_its_field(record, field):
    with pytest.raises(ValueError, match=rf"^'{field}'"):
        Style.from_dict(record)
    with pytest.raises(ValueError, match=rf"^'{field}'"):     # wrapped, as in a YAML file
        Style.from_dict({"style": record})


def test_an_invariant_is_refused_as_one():
    with pytest.raises(ValueError, match="invariant"):
        Style.from_dict({"front_slip_max": 2.0})


def test_the_default_record_is_fastest():
    for st in (Style(), Style.from_dict(None), Style.from_dict({}), Style.from_dict({"style": None}),
               Style.from_dict({"style": {"name": "fastest"}}), as_style(None), FASTEST):
        assert st == Style() and st.name == "fastest" and st.implemented
    assert "ice" not in Style().classes and "asphalt" not in Style().classes


def test_good_records_load_from_yaml(tmp_path):
    p = tmp_path / "s.yaml"
    p.write_text("style:\n  name: drift_budget\n  eps_lap_s: 0.2\n  eps_window: 0.01\n")
    st = Style.from_yaml(str(p))
    assert (st.name, st.eps_lap_s, st.eps_window) == ("drift_budget", 0.2, 0.01)
    st = Style.from_dict({"name": "drift_max", "classes": ["mud_wet", "sand"], "l2_slip_gain": 4})
    assert st.classes == ("mud_wet", "sand") and Style.from_dict(st.to_dict()) == st
    assert Style.from_dict({"name": "drift_budget", "eps_window": 0.003}).eps_window == 0.003


def test_a_class_must_be_the_tables():
    from race_planner.track import SurfaceTable
    tab = SurfaceTable("asphalt", {"asphalt": 1.0, "mud_wet": 0.5})
    with pytest.raises(ValueError, match="^'classes'"):
        Style.from_dict({"name": "drift_max", "classes": ["mud"]}).validate(tab)
    Style.from_dict({"name": "drift_max", "classes": ["mud_wet"]}).validate(tab)


def test_the_slip_floor_may_not_pass_the_rear_cap():
    st = Style.from_dict({"name": "drift_max", "slip_floor_past_deg": 13.0})
    st.check_slip_floor({"mud_wet": math.radians(7.0)})            # 7 + 13 <= 21
    with pytest.raises(ValueError, match="^'slip_floor_past_deg'"):
        st.check_slip_floor({"sand": math.radians(6.0)})           # 6 + 13 > 18
    st.check_slip_floor({})                                         # nothing to compute: passes
    Style().check_slip_floor({"x": math.radians(1.0)})              # S1 has no floor


# ---- the planner runs S1 only (M1) ----

def straight(mu=1.12, sliding=None):
    t = road(lambda s: 0.0 if s < 150 else (1 / 9.0 if s < 150 + 9 * math.pi else 0.0), 600.0,
             ds=4.0, w=5.0, mu=mu)
    if sliding is not None:
        t = dataclasses.replace(t, sliding=np.full_like(t.s, sliding))
    return t


def test_the_planner_refuses_a_style_not_built_yet():
    c = Car.from_spec(SPEC)
    for rec in ({"name": "drift_budget", "eps_lap_s": 0.2}, {"name": "drift_max"}):
        with pytest.raises(NotImplementedError, match=rec["name"]):
            Driver(c, straight(), NoTracker(), style=rec)
        with pytest.raises(NotImplementedError, match=rec["name"]):   # before any process
            MultiStart(types.SimpleNamespace(style=None), style=rec)
    with pytest.raises(ValueError, match="^'eps_window'"):
        Driver(c, straight(), NoTracker(), style={"eps_window": 0.01})
    assert Driver(c, straight(), NoTracker()).style == FASTEST


def test_the_driver_checks_the_slip_floor_on_its_tracks_surfaces():
    c = Car.from_spec(SPEC)
    t = straight(mu=0.5, sliding=1.0)             # no fall-off: the rear cap at 3 x the peak
    with pytest.raises(ValueError, match="^'slip_floor_past_deg'"):
        Driver(c, t, NoTracker(), style={"name": "drift_max", "slip_floor_past_deg": 60.0})


def test_serve_refuses_a_bad_record_before_ready():
    p = subprocess.run([sys.executable, "-m", "race_planner.serve"],
                       input=json.dumps({"init": {"style": {"colour": 1}}}) + "\n",
                       capture_output=True, text=True, timeout=60)
    assert p.returncode != 0 and "'colour'" in p.stderr and "ready" not in p.stdout


# ---- a style change at a re-plan, deferred near a committed drift ----

def _driver_with_slide(monkeypatch):
    monkeypatch.setattr(style_mod, "IMPLEMENTED", style_mod.NAMES)   # a style to change to
    c = Car.from_spec(SPEC)
    d = Driver(c, straight(), NoTracker())
    corner = modes.corners(d.track, d.grip)[0]
    s = d.track.s[:80].copy()
    past = np.where((s >= corner.s_in) & (s <= corner.s_out), math.radians(8.0), 0.0)
    d.plan = {"s": s, "past": past, "i0": 0, "X": np.zeros((6, len(s)))}
    return d, corner


def test_a_style_change_waits_while_a_committed_drift_is_within_ahead(monkeypatch):
    d, c = _driver_with_slide(monkeypatch)
    zone0 = c.s_in - CORNER_FROM
    assert d.set_style({"name": "drift_budget", "eps_lap_s": 0.2}).name == "drift_budget"
    assert d.style.name == "fastest"                                  # not before a re-plan
    for s in (zone0 - AHEAD + 1.0, zone0 + 5.0, c.s_out - 1.0):       # near it, in it
        assert d.style_due({"t": 0.0, "s": s}) == "deferred"
        assert d.style.name == "fastest" and d.style_pending.name == "drift_budget"
    assert d.style_due({"t": 1.0, "s": c.s_out + 1.0}) == "applied"   # past it
    assert d.style.name == "drift_budget" and d.style_pending is None
    assert d.style_due({"t": 2.0, "s": c.s_out + 2.0}) is None
    assert [e["event"] for e in d.style_log] == ["requested"] + ["deferred"] * 3 + ["applied"]


def test_a_style_change_applies_at_once_far_from_a_drift_or_on_a_grip_plan(monkeypatch):
    d, c = _driver_with_slide(monkeypatch)
    d.set_style({"name": "drift_max"})
    assert d.style_due({"t": 0.0, "s": c.s_in - CORNER_FROM - AHEAD - 5.0}) == "applied"
    d, c = _driver_with_slide(monkeypatch)
    d.plan["past"] = np.zeros_like(d.plan["past"])                     # a grip plan: no commit
    d.set_style({"name": "drift_max"})
    assert d.style_due({"t": 0.0, "s": c.s_in}) == "applied"


def test_a_refused_runtime_style_keeps_the_current_one():
    c = Car.from_spec(SPEC)
    d = Driver(c, straight(), NoTracker())
    out = style_message(d, {"name": "drift_max"})
    assert "NotImplementedError" in out["style_error"] and out["style"] == "fastest"
    out = style_message(d, {"name": "drift_budget", "eps_window": 0.5})
    assert "'eps_window'" in out["style_error"]
    assert d.style == FASTEST and d.style_pending is None
    assert style_message(d, {"name": "fastest"}) == {"style": "fastest", "pending": True}


# ---- S1 with the record is S1 without it, bit for bit ----

def _drive(track, st, monkeypatch, s_end=260.0):
    monkeypatch.setattr(window, "time", types.SimpleNamespace(time=lambda: 0.0))   # no clock in it
    c = Car.from_spec(SPEC)
    kw = {} if st is None else {"style": st}
    d = Driver(c, track, NoTracker(), budget=3.0, **kw)
    f = PlanFollower(d, 10.0, 12.0)
    st_ = f.state()
    d.start(st_)
    plans = [d.plan]
    while st_["s"] < s_end and d.stopping is None:
        d.step(st_)
        if plans[-1] is not d.plan:
            plans.append(d.plan)
        st_ = f.step(0.05)
    return d, plans


def test_s1_with_the_default_record_plans_bit_for_bit_as_without_one(monkeypatch):
    t = straight()
    a, pa = _drive(t, None, monkeypatch)
    b, pb = _drive(t, Style(), monkeypatch)
    assert b.style == a.style == FASTEST
    assert len(pa) == len(pb) >= 4
    for x, y in zip(pa, pb):
        for k in ("X", "U", "s", "t"):
            assert np.asarray(x[k]).tobytes() == np.asarray(y[k]).tobytes(), k
    assert a.solves and [(s["t"], s["iters"]) for s in a.solves] == \
        [(s["t"], s["iters"]) for s in b.solves]
