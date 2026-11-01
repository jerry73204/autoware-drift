"""axle.py's brakes are the ECU's: for a request a at a speed, the regen, the front and the rear
braking forces L1 plans with are what sim_vehicle_ecu (the code the car runs) puts on the
wheels once its actuators have settled."""
import pytest

import casadi as ca

from race_planner import axle
from race_planner.car import Car
from race_planner.sim import _ecu_module
from test_ocp import SPEC
import yaml


def ecu_forces(ecu_mod, cal, car, a, v, steps=120):
    """The ECU's settled (regen, front friction, rear friction) for request a at speed v, N at
    the tyres, the car rolling straight (every wheel at v)."""
    e = ecu_mod.Ecu(cal)
    w = v / car.r_wheel
    for _ in range(steps):
        act = e.step(ecu_mod.Request(float(a), 60.0, 0.0),
                     ecu_mod.Sensors((w, w, w, w), 0.0, float(a), 0.0), 1.0 / 60.0)
    R = car.r_wheel
    motor = (act.drive[2] + act.drive[3]) / R              # < 0: regenerating
    return max(-motor, 0.0), (act.brake[0] + act.brake[1]) / R, \
        (act.brake[2] + act.brake[3]) / R, max(motor, 0.0)


@pytest.mark.parametrize("v", [5.0, 12.0, 20.0, 30.0, 45.0])
@pytest.mark.parametrize("a", [-1.0, -2.5, -4.0, -6.0, -9.0])
def test_the_brakes_split_as_the_ecu_splits_them(a, v):
    car = Car.from_spec(SPEC)
    mod = _ecu_module()
    cal = mod.Calibration.from_spec(yaml.safe_load(open(SPEC)))
    regen, front, rear_friction, motor = ecu_forces(mod, cal, car, a, v)
    drive, r_, f_, rear = (float(q) for q in axle.brakes(car, a, v))
    tol = 60.0 + 0.02 * (front + rear_friction + regen)        # the smooth max's width, lags
    assert drive == pytest.approx(motor, abs=tol)           # a mild request at speed drives
    assert r_ == pytest.approx(regen, abs=tol)
    assert f_ == pytest.approx(front, abs=tol)
    assert rear == pytest.approx(regen + rear_friction, abs=tol)


def _held_ecu_forces(ecu_mod, cal, car, a, v, mu, steps=120):
    """As ecu_forces, the ECU having found the rear's friction mu (a regen slip on that road,
    issue 009): its regen slip control then holds regen within regen_grip_share of the grip."""
    e = ecu_mod.Ecu(cal)
    e.powertrain.axle_mu = mu
    w = v / car.r_wheel
    for _ in range(steps):
        act = e.step(ecu_mod.Request(float(a), 60.0, 0.0),
                     ecu_mod.Sensors((w, w, w, w), 0.0, float(a), 0.0), 1.0 / 60.0)
    R = car.r_wheel
    return (max(-(act.drive[2] + act.drive[3]) / R, 0.0), (act.brake[0] + act.brake[1]) / R,
            (act.brake[2] + act.brake[3]) / R)


@pytest.mark.parametrize("v", [8.0, 15.0])
@pytest.mark.parametrize("a", [-0.8, -1.5, -2.2])
@pytest.mark.parametrize("mu", [0.2, 0.265])
def test_on_a_slick_road_regen_is_held_as_the_ecu_holds_it(mu, a, v):
    # issue 009: the car's regen slip control, once it has found the rear's grip, holds regen
    # within regen_grip_share of it and the front's friction brakes take their share; the model
    # (at the road's friction, as L2 plans, on a road that falls off as ice) splits the brakes
    # the same way
    mod = _ecu_module()
    cal = mod.Calibration.from_spec(yaml.safe_load(open(SPEC)))
    if not hasattr(cal, "regen_slip_control"):
        pytest.skip("this sim_vehicle_ecu has no regen slip control (SIM_VEHICLE_ECU)")
    car = Car.from_spec(SPEC)
    assert car.regen_grip_share == cal.regen_grip_share
    regen, front, rear_friction = _held_ecu_forces(mod, cal, car, a, v, mu)
    o = axle.dynamics(car, [v, 0.0, 0.0], [0.0, a], mu, [a, 0.0], sliding=0.7)
    tol = 60.0 + 0.02 * (front + rear_friction + regen)
    assert float(o["front_brake"]) == pytest.approx(front, abs=tol)
    assert float(o["rear_brake"]) == pytest.approx(regen + rear_friction, abs=tol)
    # the regen in it: the hold, or the rear's own share where that is more (the ECU then
    # regenerates the share the rear's friction brakes would take: the same force on the axle)
    own = (1.0 - float(o["share"])) * float(o["brake"])
    if own < float(o["regen"]) - tol:
        assert float(o["regen"]) == pytest.approx(regen, abs=tol)
    else:
        assert regen == pytest.approx(own, abs=tol)


def test_the_hold_is_in_the_model_only_on_a_slick_road_that_falls_off():
    car = Car.from_spec(SPEC)
    thr = axle.regen_slip_mu(car)
    assert 0.40 < thr < 0.45          # drift_mustang: ice, snow, sand, mud below; grass above
    assert axle.regen_hold_car(car, [1.12, 0.265], [0.8, 0.7]) is car          # ice
    for mu, sl in (([1.12, 0.55], [0.8, 0.75]),    # grass: grippy enough
                   ([1.12, 0.2], [0.8, 0.8]),      # packed snow: keeps its force past the peak
                   ([1.12, 0.29], [0.8, 1.0])):    # mud
        free = axle.regen_hold_car(car, mu, sl)
        assert free.regen_grip_share is None and car.regen_grip_share == 0.7
    o = axle.dynamics(car, [10.0, 0.0, 0.0], [0.0, -1.5], 0.2, [-1.5, 0.0], sliding=0.8)
    p = axle.dynamics(free, [10.0, 0.0, 0.0], [0.0, -1.5], 0.2, [-1.5, 0.0], sliding=0.8)
    assert float(o["rear_brake"]) == float(p["rear_brake"])      # not held on snow
    # without it the model is the one before issue 009's hold, expression for expression
    x = [ca.SX.sym(n) for n in ("vx", "vy", "r")]
    u = [ca.SX.sym("d"), ca.SX.sym("a")]
    mu = ca.SX.sym("mu")
    o = axle.dynamics(free, x, u, mu, [u[1], 0.0])
    assert "if_else" not in str(o["rear_brake"]) and "?" not in str(o["rear_brake"])
