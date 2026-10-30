"""The planner's tyre on each of swb's surface classes against swb's surface layer itself (roadmap
061 M2): swb_isaac_world.surfaces and surface_layer, imported from the superproject's checkout
(SWB), are what Isaac runs. For every class in its table, the planner's tyre (track.SurfaceTable
.tyre, model.tyre) has swb's stiffness scales and peak slips, and its force curve is PhysX's tyre
at the stiffness swb writes under the friction swb writes (WheelSurfaces.update, times PhysX's
friction-vs-slip graph, which swb divides out and PhysX puts back): the same peak, where the
class says, the same value and the same fall-off past it.

PhysX's tyre itself is model.tyre with no fall-off (checked against Isaac's sweeps on dry asphalt
in test_model.py). Isaac's sweeps per class are not in test/data: the manoeuvre suite runs each
class only on Isaac, which this host does not run beside CARLA (deferred to 061 M6).
"""
import math
import os
import sys
from types import SimpleNamespace

import casadi as ca
import numpy as np
import pytest

from race_planner import model
from race_planner.car import Car
from race_planner.track import TYRE_FIELDS, SurfaceTable, Track

SWB = os.environ.get("SWB", os.path.expanduser("~/repos/carla-scenario-bridge"))
SPEC = os.path.join(SWB, "src/sim_vehicle_spec/vehicles/drift_mustang.yaml")
CLASSES = os.path.join(SWB, "src/swb_isaac_world/config/surface_classes.yaml")
HARUNA = os.path.join(SWB, "data/courses/haruna")
sys.path.insert(0, os.path.join(SWB, "src/swb_isaac_world"))
swb = pytest.importorskip("swb_isaac_world.surfaces")
layer = pytest.importorskip("swb_isaac_world.surface_layer")

NO_FALL = 1e6          # a peak share so far out that rho never passes 1: PhysX's own tyre


def wheel_surfaces(car, table):
    """swb's WheelSurfaces for `car` on a map that is only its class table."""
    rig = layer.rig_peaks(SimpleNamespace(mass=car.m, long_stiff_value=car.long_stiff_value,
                                          lat_stiff_value=car.lat_stiff_value))
    return swb.WheelSurfaces(SimpleNamespace(table=table), car.mu, car.friction_vs_slip, rig, 1)


def swb_wheel(ws, cls, kappa, tan_alpha):
    """swb's wheel on `cls` at a slip: (stiffness (lat, lon), peaks (kappa, tan), the friction
    PhysX applies there (the value written times its graph), relative to dry asphalt's)."""
    ws.reset()                        # a fresh wheel: the blend snaps to the ground
    # at 1 mm/s: no lag (below LAG_MIN_SPEED) and no Stribeck (no sliding speed)
    w = ws.update(0, swb.Surface(cls), swb.RoadCondition(), 1e-3, kappa, 0.01,
                  slip_angle=math.atan(tan_alpha))
    return ws.stiffness[0], ws.peaks[0], w * swb.graph_at(ws.vehicle_graph, kappa)


def tyre_fn(car):
    k, t, fz, fz0, mu, sl = (ca.SX.sym(n) for n in ("k", "t", "fz", "fz0", "mu", "sl"))
    sf = ca.SX.sym("sf", 4)
    return ca.Function("tyre", [k, t, fz, fz0, mu, sl, sf],
                       list(model.tyre(car, k, t, fz, fz0, mu, sl, sf)))


@pytest.fixture(scope="module")
def setup():
    car = Car.from_spec(SPEC)
    return car, SurfaceTable.from_yaml(CLASSES), swb.load_classes(CLASSES), tyre_fn(car)


def test_the_rig_s_peaks_are_swb_s(setup):
    car, _, _, _ = setup
    rig = layer.rig_peaks(SimpleNamespace(mass=car.m, long_stiff_value=car.long_stiff_value,
                                          lat_stiff_value=car.lat_stiff_value))
    assert model.peaks(car) == pytest.approx(tuple(p * car.mu for p in rig), rel=1e-12)
    assert (model.PEAK_SPAN, model.STIFFNESS_IN_USE) == (layer.PEAK_SPAN, layer.STIFFNESS_IN_USE)


def test_each_class_s_stiffness_and_peaks_are_swb_s(setup):
    """SurfaceTable.tyre against WheelSurfaces: the stiffness swb writes (shares of the rig's)
    and the peaks it takes rho over, for every class in swb's table."""
    car, table, ct, _ = setup
    ws = wheel_surfaces(car, ct)
    kr, tr = model.peaks(car)
    assert set(table.mu_of) == set(ct.classes)
    for cls in ct.classes:
        s_lon, s_lat, a_lon, a_lat = table.tyre(cls, car)
        (w_lat, w_lon), (kp, tp), _ = swb_wheel(ws, cls, 0.0, 0.0)
        assert (s_lon, s_lat) == pytest.approx((w_lon, w_lat), rel=1e-12), cls
        assert (kr * a_lon, tr * a_lat) == pytest.approx((kp, tp), rel=1e-12), cls
        c = ct[cls]
        if c.peak_slip_angle is not None:
            assert math.atan(tr * a_lat) == pytest.approx(c.peak_slip_angle, rel=1e-12)
    # dry asphalt is the rig's own tyre, exactly: the live path's numbers do not move
    assert table.tyre("asphalt", car) == (1.0, 1.0, 1.0, 1.0)
    t = Track.from_swb_course(HARUNA, table, car)
    assert all(np.all(getattr(t, k) == 1.0) for k in TYRE_FIELDS)


def _curve(f, car, cls, table, slips, lateral=True):
    """The planner's force along one slip direction on class `cls` at rest load."""
    fz0 = model.rest_loads(car)[2]
    mu, sl, sf = table.mu(cls, car), table.sliding(cls), table.tyre(cls, car)
    out = []
    for x in slips:
        fx, fy = f(0.0 if lateral else x, x if lateral else 0.0, fz0, fz0, mu, sl, sf)
        out.append(-float(fy) if lateral else float(fx))
    return np.array(out)


def _reference(f, car, cls, table, ct, ws, slips, lateral=True):
    """PhysX's tyre (model.tyre with no fall-off) at swb's stiffness under swb's friction."""
    fz0 = model.rest_loads(car)[2]
    mu_ref = table.mu_of[table.reference]           # PhysX's friction 1.0, absolute
    out = []
    for x in slips:
        k, t = (0.0, x) if lateral else (x, 0.0)
        (w_lat, w_lon), _, mu_w = swb_wheel(ws, cls, k, t)
        fx, fy = f(k, t, fz0, fz0, mu_ref * mu_w, 1.0, [w_lon, w_lat, NO_FALL, NO_FALL])
        out.append(-float(fy) if lateral else float(fx))
    return np.array(out)


def _peak(slips, force):
    """The smallest slip within 0.5 % of the largest force (modes.rear_peak's rule)."""
    return float(slips[int(np.argmax(force >= 0.995 * force.max()))])


@pytest.mark.parametrize("lateral", [True, False], ids=["lateral", "longitudinal"])
def test_each_class_s_force_curve_is_swb_s(setup, lateral):
    """Pure slip, each direction, every class, at rest load: the planner's curve and the
    reference agree within 1 % of the peak force everywhere (measured: 0.6 % at most, ice; the
    planner softens rho's knee at 1 for its optimiser, which is all that differs), the peak (the
    smallest slip within 0.5 % of the largest force) at the same slip within 2.5 % or two grid steps
    (measured 2 %; ice lengthways 0.047 against 0.049 on a 0.001 grid;
    the softened knee: asphalt 8.65 deg against 8.50), its value within 1 % (0.6 %), and the
    fall-off at twice and four times the class's peak within 0.5 %. Where a class falls off
    (sliding 0.8 and less) the force peaks at the class's own peak slip angle: asphalt 8.5 deg,
    packed snow 6.0, ice 3.0; where it barely does (0.9-1.0: gravel, sand, mud, loose snow)
    PhysX's tyre still rises 1.4 % past it (swb's 2.3 mu Fz / C is 98.6 % of its brush's
    saturation), to 1.1-1.3 times the class's angle, in swb's model as in the planner's."""
    car, table, ct, f = setup
    ws = wheel_surfaces(car, ct)
    kr, tr = model.peaks(car)
    if lateral:
        slips = np.tan(np.radians(np.arange(0.05, 40.0, 0.05)))
    else:
        slips = np.arange(0.001, 1.0, 0.001)
    for cls in ct.classes:
        p = _curve(f, car, cls, table, slips, lateral)
        r = _reference(f, car, cls, table, ct, ws, slips, lateral)
        top = r.max()
        assert np.max(np.abs(p - r)) <= 0.01 * top, cls
        at = np.arctan(slips) if lateral else slips
        step = at[1] - at[0]
        assert _peak(at, p) == pytest.approx(_peak(at, r), rel=0.025, abs=2.01 * step), cls
        assert p.max() == pytest.approx(top, rel=0.01), cls
        _, _, a_lon, a_lat = table.tyre(cls, car)
        knee = tr * a_lat if lateral else kr * a_lon
        for m in (2.0, 4.0):
            x = knee * m
            if x >= slips[-1]:
                continue
            i = int(np.searchsorted(slips, x))
            assert p[i] == pytest.approx(r[i], rel=0.005), (cls, m)


def test_a_class_s_peak_moves_the_planner_s_rear_peak(setup):
    """modes.rear_peak, the study's and the chooser's drift line, on the class's tyre: packed
    snow peaks later than the rig's tyre at its friction did (2-3 deg), ice earlier."""
    from race_planner import modes
    car, table, _, _ = setup
    deg = {}
    for cls in ("asphalt", "snow_packed", "ice", "gravel"):
        mu, sl = table.mu(cls, car), table.sliding(cls)
        deg[cls] = (math.degrees(modes.rear_peak(car, mu, sl)),
                    math.degrees(modes.rear_peak(car, mu, sl, table.tyre(cls, car))))
    assert deg["asphalt"][0] == deg["asphalt"][1]
    assert deg["snow_packed"][0] < 3.0 < 5.5 < deg["snow_packed"][1] < 7.0
    assert deg["ice"][1] < deg["ice"][0] + 0.5
    assert deg["gravel"][1] > 10.0
