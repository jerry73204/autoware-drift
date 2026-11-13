"""studies/acados_tracker.py: L2's tracker as an acados OCP (race stack phase 1, task 5).

Skipped where acados_template is not importable. acados' hpipm/blasfeo and CasADi's HPIPM plugin
(the CasADi Tracker's QP) share symbol names, so neither is loaded in pytest's own process (the
suite runs test_tracker there): the acados solve and the CasADi Tracker's step each run in a
subprocess of their own."""
import importlib.util
import json
import os
import subprocess
import sys

import numpy as np
import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
STUDIES = os.path.join(os.path.dirname(HERE), "studies")
if importlib.util.find_spec("acados_template") is None:
    pytest.skip("acados_template is not importable", allow_module_level=True)
sys.path.insert(0, STUDIES)
import acados_tracker as at  # noqa: E402  (CasADi only: it loads neither acados nor HPIPM)
if at.acados_dir() is None:
    pytest.skip("no acados install (lib/libacados.so)", allow_module_level=True)
V = 10.0          # m/s, the straight's speed


def _call(expr):
    """json of acados_tracker.<expr>, evaluated in a fresh interpreter."""
    code = ("import json, sys, warnings; warnings.simplefilter('ignore'); "
            f"sys.path.insert(0, {STUDIES!r}); import acados_tracker as at; "
            f"print(json.dumps(at.{expr}))")
    env = dict(os.environ)
    env.pop("PYTHONPATH", None)
    r = subprocess.run([sys.executable, "-c", code], capture_output=True, text=True, env=env,
                       cwd=os.path.dirname(HERE), timeout=900)
    assert r.returncode == 0, r.stderr[-3000:]
    return json.loads(r.stdout.strip().splitlines()[-1])


def test_straight_constant_speed_holds_still():
    """kappa 0, references n 0 and vx V everywhere, from that state: the car is left alone."""
    status, u0 = _call(f"acados_straight({V!r}, 0.0)")
    assert status == 0
    assert abs(u0[0]) < 1e-3, u0          # delta's rate, rad/s
    assert abs(u0[1]) < 0.5, u0           # the request, m/s^2


def test_offset_steers_back():
    """The same references from n = +0.5 m: the first steering rate turns the car back to the
    line, with the sign the CasADi Tracker gives on that state."""
    status, u0 = _call(f"acados_straight({V!r}, 0.5)")
    assert status == 0
    ref = _call(f"casadi_straight_rate({V!r}, 0.5)")
    assert abs(ref) > 1e-3 and abs(u0[0]) > 1e-3, (ref, u0)
    assert np.sign(u0[0]) == np.sign(ref), (u0, ref)
