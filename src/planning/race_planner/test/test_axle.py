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
