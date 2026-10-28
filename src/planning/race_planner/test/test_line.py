"""The minimum-curvature line: inside the road less the car's half width and a margin, and on
it the speed profile is faster than on the centreline."""
import math
import os

import numpy as np
import pytest

from race_planner.car import Car
from race_planner.line import min_curvature_line
from race_planner.qss import lap_time, speed_profile
from race_planner.track import SurfaceTable, Track

SWB = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
SPEC = os.path.join(SWB, "src/sim_vehicle_spec/vehicles/drift_mustang.yaml")
CLASSES = os.path.join(SWB, "src/swb_isaac_world/config/surface_classes.yaml")
HARUNA = os.path.join(SWB, "data/courses/haruna")


def road(kappa, s_end, ds=1.0, w=5.0, closed=False):
    s = np.arange(0.0, s_end + ds / 2, ds)
    k = np.vectorize(kappa)(s).astype(float)
    psi = np.concatenate([[0.0], np.cumsum(0.5 * (k[1:] + k[:-1]) * ds)])
    x = np.concatenate([[0.0], np.cumsum(0.5 * (np.cos(psi[1:]) + np.cos(psi[:-1])) * ds)])
    y = np.concatenate([[0.0], np.cumsum(0.5 * (np.sin(psi[1:]) + np.sin(psi[:-1])) * ds)])
    one = np.ones_like(s)
    return Track(s=s, x=x, y=y, z=0 * s, psi=psi, kappa=k, w_left=w * one, w_right=w * one,
                 mu=one, closed=closed)


def free(t, half, margin):
    return t.w_left - half - margin, t.w_right - half - margin


def test_a_straight_keeps_the_centre():
    t = road(lambda s: 0.0, 200.0)
    line, n = min_curvature_line(t, half_width=1.0, margin=0.3)
    assert np.max(np.abs(n)) < 0.01
    assert np.max(np.abs(line.kappa)) < 1e-4


def test_a_right_angle_bend_is_cut_outside_inside_outside():
    """A 90 deg left bend of R 20 between straights, 5 m either side: the line enters wide (right),
    touches the inside (left) at the bend's middle, and its tightest radius is much larger."""
    r, a, b = 20.0, 150.0, 150.0 + 0.5 * math.pi * 20.0
    t = road(lambda s: 1.0 / r if a <= s < b else 0.0, b + 150.0)
    half, margin = 1.0, 0.3
    line, n = min_curvature_line(t, half_width=half, margin=margin)
    wl, wr = free(t, half, margin)
    assert np.all(n <= wl + 1e-3) and np.all(n >= -wr - 1e-3)
    mid = np.searchsorted(t.s, 0.5 * (a + b))
    assert n[mid] > wl[mid] - 0.3                               # apex: on the inside edge
    assert n[np.searchsorted(t.s, a - 15.0)] < -2.0             # turn-in from the outside
    assert n[np.searchsorted(t.s, b + 15.0)] < -2.0             # track-out to the outside
    assert 1.0 / np.max(np.abs(line.kappa)) > 1.5 * r
    car = Car.from_spec(SPEC)
    t_line = lap_time(line, speed_profile(line, car, v_start=20.0))
    t_centre = lap_time(t, speed_profile(t, car, v_start=20.0))
    assert t_line < t_centre


def test_a_closed_circle_runs_on_its_outer_edge():
    """On a circle every point is the same corner: the least curvature is the largest radius."""
    r = 50.0
    t = road(lambda s: 1.0 / r, 2 * math.pi * r - 1.0, closed=True)
    line, n = min_curvature_line(t, half_width=1.0, margin=0.3)
    wl, wr = free(t, 1.0, 0.3)
    assert np.allclose(n, -wr, atol=0.05)
    assert np.allclose(line.kappa, 1.0 / (r + wr[0]), rtol=0.02)


@pytest.mark.skipif(not os.path.isdir(HARUNA), reason="Haruna's course data is not here")
def test_haruna_line_stays_on_the_road_and_is_faster():
    car = Car.from_spec(SPEC)
    t = Track.from_swb_course(HARUNA, SurfaceTable.from_yaml(CLASSES), car).resampled(2.0)
    line, n = min_curvature_line(t, half_width=0.5 * car.width, margin=0.3)
    wl, wr = free(t, 0.5 * car.width, 0.3)
    assert np.all(n <= wl + 1e-3) and np.all(n >= -wr - 1e-3)
    assert np.all(line.w_left >= 0.5 * car.width) and np.all(line.w_right >= 0.5 * car.width)
    t_line = lap_time(line, speed_profile(line, car, v_start=0.0))
    t_centre = lap_time(t, speed_profile(t, car, v_start=0.0))
    assert t_line < 0.97 * t_centre, (t_line, t_centre)
