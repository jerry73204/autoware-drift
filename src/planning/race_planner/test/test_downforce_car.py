"""A downforce car with a load-sensitive, rewritten tyre (car.py's optional `aero` and
`tire_model`): FastSlowRR's generic 2026 F1 car (test/data/generic_f1_2026.yaml, its
`circuitgen spec`, roadmap 003 R1/R2), checked against what its Isaac car does
(FastSlowRR circuits/isaac/f1car.py) and against a car without them."""
import dataclasses
import math
import os

import casadi as ca
import numpy as np

from race_planner import model, qss
from race_planner.car import Car
from race_planner.track import Track

SPEC = os.path.join(os.path.dirname(__file__), "data/generic_f1_2026.yaml")
G = 9.81


def car():
    return Car.from_spec(SPEC)


def test_the_spec_reads_aero_and_tyre():
    c = car()
    # 1/2 rho Cl A: 0.5 x 1.2 x 4.7138 x 1.5
    assert abs(c.downforce - 0.5 * 1.2 * 4.7138 * 1.5) < 1e-9
    assert abs(c.aero_front_share - 0.4669) < 1e-12
    assert abs(c.tyre_peaks[0] - 0.11) < 1e-12
    assert abs(c.tyre_peaks[1] - math.tan(math.radians(9.0))) < 1e-12
    (f1, f2), mf, _ = c.mu_load
    assert (f1, f2) == (2000.0, 8000.0) and abs(mf[0] - 1.0) < 1e-12


def test_downforce_loads_the_axles_by_its_share():
    c = car()
    for v in (0.0, 40.0, 85.0):
        fz = [float(x) for x in model.loads(c, 0.0, 0.0, v)]
        df = c.downforce * v * v
        assert abs(sum(fz) - (c.m * G + df)) < 1e-6
        rest = model.rest_loads(c)
        assert abs(fz[0] + fz[1] - rest[0] - rest[1] - df * c.aero_front_share
                   + c.drag * v * v * (c.drag_height - c.h_cg) / c.wheel_base) < 1e-6
    # no speed given, or no downforce: the loads as before
    nd = dataclasses.replace(c, downforce=0.0)
    assert [float(x) for x in model.loads(nd, 1.0, 2.0, 80.0)] == \
        [float(x) for x in model.loads(nd, 1.0, 2.0)]


def test_mu_falls_with_load_on_the_rear():
    c = car()
    lo, hi = (float(model.mu_share(c, False, f)) * c.mu for f in (2000.0, 8000.0))
    assert abs(lo - 1.9335) < 1e-3 and abs(hi - 1.5132) < 1e-3
    assert float(model.mu_share(c, True, 5000.0)) * c.mu > 1.92


def test_the_rewritten_tyre_peaks_at_its_slips_at_any_load():
    c = car()
    k, t, fz = ca.SX.sym("k"), ca.SX.sym("t"), ca.SX.sym("fz")
    f = ca.Function("f", [k, t, fz], list(model.tyre(c, k, t, fz, 3000.0, c.mu)))
    kp, tp = c.tyre_peaks
    for load in (2000.0, 6000.0, 12000.0):
        # brush: the full mu Fz at the peak slip (flat past it), short of it below
        cap = c.mu * load
        assert abs(float(f(kp, 0.0, load)[0]) / cap - 1.0) < 1e-6
        assert float(f(0.8 * kp, 0.0, load)[0]) / cap < 0.995
        assert abs(-float(f(0.0, tp, load)[1]) / cap - 1.0) < 1e-6
        assert -float(f(0.0, 0.8 * tp, load)[1]) / cap < 0.995


def test_qss_with_downforce_is_finite_and_faster():
    s = np.arange(0.0, 2000.0, 2.0)
    kappa = np.where((s > 900) & (s < 1100), 1 / 150.0, 0.0)
    psi = np.cumsum(kappa) * 2.0
    one = np.ones_like(s)
    tr = Track(s=s, x=np.cumsum(np.cos(psi)) * 2, y=np.cumsum(np.sin(psi)) * 2, z=0 * s, psi=psi,
               kappa=kappa, w_left=6 * one, w_right=6 * one, mu=car().mu * one, closed=False)
    c = car()
    v = qss.speed_profile(tr, c, v_start=50.0)
    v0 = qss.speed_profile(tr, dataclasses.replace(c, downforce=0.0), v_start=50.0)
    assert np.all(np.isfinite(v)) and v.max() <= c.wheel_speed_max + 1e-9
    assert v[(s > 950) & (s < 1050)].min() > 1.5 * v0[(s > 950) & (s < 1050)].min()
