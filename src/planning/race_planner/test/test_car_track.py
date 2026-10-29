"""The car from sim_vehicle_spec and a track from swb's course generator, as the planner sees them."""
import math
import os

import numpy as np
import pytest

from race_planner.car import Car
from race_planner.track import Track, SurfaceTable

SWB = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
SPEC = os.path.join(SWB, "src/sim_vehicle_spec/vehicles/drift_mustang.yaml")
CLASSES = os.path.join(SWB, "src/swb_isaac_world/config/surface_classes.yaml")
HARUNA = os.path.join(SWB, "data/courses/haruna")


def test_car_reads_the_spec():
    car = Car.from_spec(SPEC)
    assert car.name == "drift_mustang"
    assert car.m == 1450.0
    assert car.lf == pytest.approx(1.443) and car.lr == pytest.approx(1.443)
    assert car.width == pytest.approx(1.586 + 2 * 0.1545)
    assert car.h_cg == 0.48 and car.track == 1.586 and car.r_wheel == 0.32
    assert car.steer_max == pytest.approx(1.01)
    assert car.diff == "locked" and car.drive_axle == "rear"
    # yaw inertia of the inertia box (4.72 x 1.90): m (l^2 + w^2) / 12
    assert car.Iz == pytest.approx(1450 * (4.72 ** 2 + 1.90 ** 2) / 12)
    # the motor through its gear: 700 N m x 6 at the axle, 330 kW
    assert car.axle_torque_max == pytest.approx(4200.0)
    assert car.power_max == 330000.0
    assert car.drag == pytest.approx(0.5 * 1.2 * 0.45 * 1.9)


def test_surface_mu_is_the_tire_friction_scaled_by_the_class():
    table = SurfaceTable.from_yaml(CLASSES)
    car = Car.from_spec(SPEC)
    assert table.mu("asphalt", car) == pytest.approx(1.12)       # absolute, at tyre friction 1.0
    for low in ("ice", "snow_packed", "gravel", "grass"):  # the classes as swb has them
        assert 0.1 < table.mu(low, car) < 0.65, low


@pytest.mark.skipif(not os.path.isdir(HARUNA), reason="Haruna's course data is not here")
def test_haruna_from_the_course_generator():
    car = Car.from_spec(SPEC)
    t = Track.from_swb_course(HARUNA, SurfaceTable.from_yaml(CLASSES), car)
    assert not t.closed
    assert t.s[-1] == pytest.approx(1527, abs=2)
    assert np.all(np.diff(t.s) > 0)
    assert 1.0 / np.max(np.abs(t.kappa)) == pytest.approx(9.2, abs=1.0)   # H1's apex, R 9.2
    assert np.allclose(t.w_left, 4.0) and np.allclose(t.w_right, 4.0)
    assert np.allclose(t.mu, 1.12)                                        # asphalt everywhere
    assert -0.09 < np.min(t.grade) < -0.06                               # downhill, ~8 % at worst
    # heading and curvature agree: the heading's change is the curvature's integral
    turn = np.trapezoid(t.kappa, t.s)
    assert turn == pytest.approx(t.psi[-1] - t.psi[0], abs=math.radians(2))


def test_track_resamples_and_interpolates():
    s = np.linspace(0, 100, 51)
    t = Track(s=s, x=s, y=0 * s, z=0 * s, psi=0 * s, kappa=0 * s, w_left=3 + 0 * s, w_right=3 + 0 * s,
              mu=1 + 0 * s, closed=False)
    r = t.resampled(5.0)
    assert r.s[1] - r.s[0] == pytest.approx(5.0) and r.s[-1] == pytest.approx(100.0)
    assert r.at(42.0)["x"] == pytest.approx(42.0)


def test_surface_sliding_share_comes_from_the_class():
    table = SurfaceTable.from_yaml(CLASSES)
    assert table.sliding("asphalt") == pytest.approx(0.8)
    car = Car.from_spec(SPEC)
    if os.path.isdir(HARUNA):
        assert np.allclose(Track.from_swb_course(HARUNA, table, car).sliding, 0.8)


def test_car_reads_its_brake_distribution():
    assert Car.from_spec(SPEC).brake_distribution == "static"
