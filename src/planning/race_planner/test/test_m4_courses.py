"""The M4 harness's one-class courses (studies/mode_choice_m4.py, `<course>@<class>`) and the
plan the live process reports (serve.plan_message)."""
import os
import sys

import numpy as np
import pytest

from race_planner.car import Car
from race_planner.serve import PlanIds, plan_message

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "studies"))
import mode_choice_m4 as m4  # noqa: E402


@pytest.fixture(scope="module")
def car():
    return Car.from_spec(m4.e1.SPEC)


@pytest.mark.parametrize("name", ["haruna@snow_packed", "zones@gravel"])
def test_a_one_class_course_is_that_class_everywhere(car, name):
    cls = name.split("@")[1]
    tab = m4.table()
    planner, world, smap, *_, info = m4.tracks(name, car)
    assert info["class"] == cls
    for t in (planner, world):
        assert np.allclose(t.mu, tab.mu(cls, car))
        assert np.allclose(t.sliding, tab.sliding(cls))
    assert set(smap.true_class(world.s)) == {cls}


def test_a_one_class_course_keeps_the_dry_geometry(car):
    dry, *_ = m4.course("haruna", car)
    snow, *_ = m4.course("haruna@snow_packed", car)
    assert np.array_equal(dry.x, snow.x) and np.array_equal(dry.w_left, snow.w_left)


@pytest.mark.parametrize("name, word", [("haruna@slush", "slush"), ("monaco@ice", "monaco")])
def test_an_unknown_class_or_course_is_refused_by_name(car, name, word):
    with pytest.raises(ValueError, match=word):
        m4.course(name, car)


def test_plan_message_numbers_each_plan_once_and_sends_it_once():
    X = np.zeros((6, 3))
    X[0], X[3] = 10.0, [0.0, 0.5, 1.0]

    class D:
        plan = {"X": X, "s": np.array([0.0, 4.0, 8.0]), "slot": "D", "mode": "drift"}
    ids = PlanIds()
    a = plan_message(D, ids)
    assert a["plan_id"] == 0 and a["slot"] == "D" and a["mode"] == "drift"
    assert a["plan"]["n"] == [0.0, 0.5, 1.0] and a["plan"]["v"] == [10.0] * 3
    b = plan_message(D, ids)
    assert b["plan_id"] == 0 and "plan" not in b
    D.plan = dict(D.plan, slot="G", mode="grip")
    assert plan_message(D, ids)["plan_id"] == 1
