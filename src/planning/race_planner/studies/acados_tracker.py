"""L2's tracker as an acados OCP: a first model (race stack phase 1, task 5).

The model is tracker._dynamics' f with tyres=False (the ellipse, not L1's PhysX tyres: phase 4
makes the two equal), states (s n xi vx vy r delta), inputs (delta's rate, the request a), as an
explicit ERK model (RK4, one step a stage, as the Tracker's own). The cost is NONLINEAR_LS on
(n xi vx vy r) and the inputs, at the Tracker's scales (tracker.SCALE_X, SCALE_RATE, SCALE_A,
TERMINAL, its line weights), with the references per stage; unscaled by the stage length
(cost_scaling 1), as the Tracker sums its residuals. Hard bounds: the steering lock (on delta)
and its rate (on the input). No edges, grip or actuator rows yet, and no jerk term (the Tracker
carries the last request as an eighth state for it). PARTIAL_CONDENSING_HPIPM under SQP_RTI.

- record: one mode_choice_m4 run (RP_DET=1) with the CasADi Tracker, saving per tick the state
  the Tracker started from, the stations (the guess's s) and the references and stage
  parameters it set there, and its output: the first input (delta_rate, a) and solve_s.
- bench: the acados solver over the recorded ticks in order (one RTI each, warm from the last
  tick's iterate): time_tot percentiles, status counts, and |delta| against the CasADi
  Tracker's first input (reported, not gated: the models differ by the tyres).

acados' hpipm/blasfeo and CasADi's HPIPM plugin share symbol names: `record` (CasADi's HPIPM)
and `bench` (acados) run in separate processes. Generated code goes to
~/.cache/race_planner/acados/<hash of the model and options>/ and is reused when present.

Run:  source ~/repos/AutoSDV/.venv-race/race-env.sh   # ACADOS_SOURCE_DIR, SWB
      env -u PYTHONPATH .venv/bin/python studies/acados_tracker.py record --course c:gravel:15:90:8 --out DIR
      env -u PYTHONPATH .venv/bin/python studies/acados_tracker.py bench --course c:gravel:15:90:8 --out DIR --label x86
"""
import argparse
import dataclasses
import hashlib
import inspect
import json
import os
import sys
import time

import casadi as ca
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from race_planner import tracker as T  # noqa: E402
from race_planner.car import Car  # noqa: E402
from race_planner.ocp import STEER_RATE  # noqa: E402
from race_planner.track import Track  # noqa: E402

from scensim_paths import scensim  # noqa: E402  (scensim from pip, or the $SWB checkout)

SWB = scensim.PLAY_LOG[:-len("/play_log")]
SPEC = scensim.spec("drift_mustang")
CACHE = os.path.expanduser("~/.cache/race_planner/acados")
COURSE = "c:gravel:15:90:8"


def acados_dir():
    """acados' install: $ACADOS_SOURCE_DIR, else $ACADOS_ROOT, else ~/.local/opt/acados (where
    AutoSDV's setup puts it); None when none holds lib/libacados.so."""
    for d in (os.environ.get("ACADOS_SOURCE_DIR"), os.environ.get("ACADOS_ROOT"),
              os.path.expanduser("~/.local/opt/acados")):
        if d and os.path.isfile(os.path.join(d, "lib", "libacados.so")):
            return d
    return None


# acados_template finds its libraries through ACADOS_SOURCE_DIR (never LD_LIBRARY_PATH here:
# acados' hpipm/blasfeo would shadow CasADi's)
if not os.environ.get("ACADOS_SOURCE_DIR") and acados_dir():
    os.environ["ACADOS_SOURCE_DIR"] = acados_dir()

# the Tracker's grid, read from its signature (horizon, dt, tick)
_SIG = inspect.signature(T.Tracker.__init__).parameters
DT = _SIG["dt"].default
TICK = _SIG["tick"].default
N = int(round(_SIG["horizon"].default / DT))
NXA = 7                 # s n xi vx vy r delta
NY_X = 5                # n xi vx vy r: the tracked states (x[1:6])

OPTIONS = {"qp_solver": "PARTIAL_CONDENSING_HPIPM", "nlp_solver_type": "SQP_RTI",
           "integrator_type": "ERK", "sim_method_num_stages": 4, "sim_method_num_steps": 1,
           "hessian_approx": "GAUSS_NEWTON", "qp_solver_warm_start": 1}


def _weights():
    """Stage and terminal weights (diagonals) from the Tracker's scales; the line's (W[0]) is
    set per stage from its line weights (tracker.Tracker._line_weights)."""
    wx = 1.0 / T.SCALE_X[:NY_X] ** 2
    w = np.concatenate([wx, [1.0 / T.SCALE_RATE ** 2, 1.0 / T.SCALE_A ** 2]])
    return w, T.TERMINAL * wx


def _model(car, grip):
    from acados_template import AcadosModel
    f, _, _, _, _ = T._dynamics(car, grip, tyres=False)
    x, u, p = ca.SX.sym("x", NXA), ca.SX.sym("u", T.NU), ca.SX.sym("p", T.NP)
    m = AcadosModel()
    m.name = "race_tracker"
    m.x, m.u, m.p = x, u, p
    m.xdot = ca.SX.sym("xdot", NXA)
    m.f_expl_expr = f(x, u, p)
    m.cost_y_expr = ca.vertcat(x[1:1 + NY_X], u)
    m.cost_y_expr_e = x[1:1 + NY_X]
    return m


def build(car, n=N, dt=DT, code_dir=None, grip=1.0, verbose=False):
    """The acados solver of the first model (module docstring) on `n` stages of `dt`.

    code_dir: where its generated code goes; None: ~/.cache/race_planner/acados/<hash of the
    model's expression, n, dt and the options>/, reused when its library is there."""
    from acados_template import AcadosOcp, AcadosOcpSolver
    import acados_template
    m = _model(car, grip)
    rate = car.steer_rate_max or STEER_RATE
    key = hashlib.sha256("\n".join([
        str(m.f_expl_expr), str(m.cost_y_expr), str(m.cost_y_expr_e), repr(n), repr(dt),
        json.dumps(OPTIONS, sort_keys=True), repr((car.steer_max, rate)),
        getattr(acados_template, "__version__", "?"), os.environ.get("ACADOS_SOURCE_DIR", "")])
        .encode()).hexdigest()[:16]
    if code_dir is None:
        code_dir = os.path.join(CACHE, key)
    os.makedirs(code_dir, exist_ok=True)
    ocp = AcadosOcp()
    ocp.model = m
    ocp.solver_options.N_horizon = n
    ocp.solver_options.tf = n * dt
    for k, v in OPTIONS.items():
        setattr(ocp.solver_options, k, v)
    ocp.solver_options.cost_scaling = np.ones(n + 1)    # the Tracker's sum, not an integral
    ocp.solver_options.print_level = 0
    w, we = _weights()
    ocp.cost.cost_type = ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.cost.W, ocp.cost.W_e = np.diag(w), np.diag(we)
    ocp.cost.yref, ocp.cost.yref_e = np.zeros(len(w)), np.zeros(len(we))
    c = ocp.constraints
    c.x0 = np.zeros(NXA)
    c.idxbx = c.idxbx_e = np.array([6])                 # the lock
    c.lbx = c.lbx_e = np.array([-car.steer_max])
    c.ubx = c.ubx_e = np.array([car.steer_max])
    c.idxbu = np.array([0])                             # its rate
    c.lbu, c.ubu = np.array([-rate]), np.array([rate])
    ocp.parameter_values = np.zeros(T.NP)
    ocp.code_gen_options.code_export_directory = os.path.join(code_dir, "c_generated_code")
    ocp.code_gen_options.json_file = "acados_ocp.json"         # in the export directory
    gen = ocp.code_gen_options.code_export_directory
    have = (os.path.isfile(os.path.join(gen, "acados_ocp.json")) and os.path.isdir(gen)
            and any(f.startswith("libacados_ocp_solver_") for f in os.listdir(gen)))
    solver = AcadosOcpSolver(ocp, generate=not have, build=not have, verbose=verbose)
    solver.code_dir, solver.n, solver.cache_hit = code_dir, n, have
    return solver


# -- one tick ----------------------------------------------------------------------------------
def _stage_refs(tick):
    """yref for stages 0..n-1 and n, and the line's weight at each stage, from a recorded tick:
    the Tracker's REF[:, k] and ISN[k] are at x_(k+1), its AREF[k] on u_k."""
    REF, AREF, ISN, x0 = tick["REF"], tick["AREF"], tick["ISN"], tick["x0"]
    n = REF.shape[1]
    yx = np.hstack([np.asarray(x0[1:1 + NY_X], float).reshape(-1, 1), REF[:NY_X]])  # x_0 .. x_n
    isn = np.concatenate([[ISN[0]], ISN])
    yref = [np.concatenate([yx[:, k], [0.0, AREF[k]]]) for k in range(n)]
    return yref, yx[:, n], isn


def _guess(tick):
    """x along the references from the car (the stations' s, the references' n .. delta)."""
    S, REF = tick["S"], tick["REF"]
    X = np.vstack([S[1:], REF]).T
    return np.vstack([np.asarray(tick["x0"], float)[None, :NXA], X])


def solve(solver, tick, reset=True):
    """One RTI step on a tick (a dict: x0 (7), S (n+1), P (NP x n), REF (6 x n), AREF (n), ISN
    (n)). reset: start from the references (else the solver's last iterate). Returns (status,
    u0, time_tot)."""
    n = solver.n
    w, we = _weights()
    yref, yref_e, isn = _stage_refs(tick)
    P = tick["P"]
    if reset:
        solver.reset()
        X = _guess(tick)
        for k in range(n + 1):
            solver.set(k, "x", X[k])
        for k in range(n):
            solver.set(k, "u", np.array([0.0, tick["AREF"][k]]))
    x0 = np.asarray(tick["x0"], float)[:NXA]
    solver.set(0, "lbx", x0)
    solver.set(0, "ubx", x0)
    for k in range(n):
        solver.set(k, "p", np.asarray(P[:, k], float))
        solver.set(k, "yref", yref[k])
        wk = w.copy()
        wk[0] = isn[k] ** 2            # the line's weight at x_k (1 / its scale squared)
        solver.cost_set(k, "W", np.diag(wk))
    solver.set(n, "p", np.asarray(P[:, n - 1], float))
    solver.set(n, "yref", yref_e)
    wn = we.copy()
    wn[0] = isn[n] ** 2               # ISN's last carries sqrt(TERMINAL) already
    solver.cost_set(n, "W", np.diag(wn))
    status = solver.solve()
    return int(status), np.asarray(solver.get(0, "u"), float), float(solver.get_stats("time_tot"))


# -- a synthetic straight (the tests) ----------------------------------------------------------
def straight_track(length=400.0, ds=4.0, mu=1.12, w=5.0):
    s = np.arange(0.0, length + ds / 2, ds)
    one = np.ones_like(s)
    return Track(s=s, x=s.copy(), y=0 * s, z=0 * s, psi=0 * s, kappa=0 * s, w_left=w * one,
                 w_right=w * one, mu=mu * one, closed=False)


def straight_plan(track, v):
    """A plan on the straight: the line, at v, no slide, no request."""
    s = track.s
    X = np.zeros((6, len(s)))
    X[0] = v
    return {"s": s, "X": X, "U": np.zeros((4, len(s) - 1)), "t": s / v, "time": s[-1] / v,
            "ok": True}


def straight_tick(v, n0, n=N, dt=DT):
    """A tick on the straight from (s 0, n n0, at v): references n 0, vx v at the stations the
    plan's speed reaches, every stage's parameters the straight's."""
    tr = straight_track()
    S = np.arange(n + 1) * v * dt
    kappa, mu, grade, sl, wl, wr = T._track_at(tr, S[:n])
    P = np.vstack([kappa, mu, grade, sl, np.zeros((2, n)), T._tyre_at(tr, S[:n])])
    REF = np.zeros((T.NR, n))
    REF[2] = v
    isn = np.full(n, 1.0 / T.SCALE_X[0])
    isn[-1] *= np.sqrt(T.TERMINAL)
    return {"x0": np.array([0.0, n0, 0.0, v, 0.0, 0.0, 0.0]), "S": S, "P": P, "REF": REF,
            "AREF": np.zeros(n), "ISN": isn}


def acados_straight(v, n0, grip=1.0):
    """(status, first input) of the acados solver on straight_tick(v, n0), from the references.
    Loads acados: never in a process that runs CasADi's HPIPM."""
    status, u0, _ = solve(build(Car.from_spec(SPEC), grip=grip), straight_tick(v, n0))
    return status, [float(x) for x in u0]


def casadi_straight_rate(v, n0, grip=1.0):
    """The CasADi Tracker's first steering rate on straight_tick's state (its sign is what the
    tests compare). Runs CasADi's HPIPM: never in a process that loaded acados."""
    car = Car.from_spec(SPEC)
    tr = straight_track()
    t = T.Tracker(car, tr, grip=grip)
    got = {}
    apply = t.lqr.apply

    def spy(state, delta, rate, a):
        got["rate"], got["a"] = float(rate), float(a)
        return apply(state, delta, rate, a)

    t.lqr.apply = spy
    st = {"s": 0.0, "n": n0, "xi": 0.0, "vx": v, "vy": 0.0, "r": 0.0, "delta": 0.0}
    out = t.step(st, straight_plan(tr, v), 0.0)
    assert out["ok"] and not out["fallback"], out
    return got["rate"]


# -- record (CasADi Tracker) -------------------------------------------------------------------
def _npz_path(out, course):
    return os.path.join(out, "rec_" + course.replace(":", "_").replace("@", "_") + ".npz")


def record(course, out_npz, arm="single"):
    """One mode_choice_m4 run (RP_DET=1) on `course`; per Tracker.step, the tick (see solve)
    and the CasADi Tracker's output, saved to out_npz."""
    os.environ["RP_DET"] = "1"
    os.environ.setdefault("M4_OUT", os.path.join(os.path.dirname(os.path.abspath(out_npz)), "m4"))
    sys.path.insert(0, HERE)
    import mode_choice_m4 as m4             # applies RP_DET at import
    ticks = []
    orig_step, orig_params = T.Tracker.step, T.Tracker._params

    def params(self, X, acc, plan, mu_scale=1.0):
        p = orig_params(self, X, acc, plan, mu_scale)
        self._rec = (np.array(X[:NXA, 0]), np.array(X[0]), p)
        return p

    def step(self, state, plan, plan_t0=None):
        if not getattr(self, "_spied", False):
            apply = self.lqr.apply

            def spy(st, delta, rate, a):
                self._u = (float(rate), float(a))
                return apply(st, delta, rate, a)
            self.lqr.apply, self._spied = spy, True
        self._rec, self._u = None, None
        c = time.perf_counter()
        out = orig_step(self, state, plan, plan_t0)
        wall = time.perf_counter() - c
        if self._rec is not None:
            x0, S, p = self._rec
            n, NP, NR = self.N, T.NP, T.NR
            i = 0
            P = p[i:i + NP * n].reshape(NP, n, order="F"); i += NP * n
            REF = p[i:i + NR * n].reshape(NR, n, order="F"); i += NR * n
            AREF = p[i:i + n]; i += 3 * n
            ISN = p[i:i + n]
            ticks.append({"x0": x0, "S": S, "P": P, "REF": REF, "AREF": AREF, "ISN": ISN,
                          "u": self._u, "solve_s": out["solve_s"], "wall_s": wall,
                          "ok": out["ok"], "fallback": out["fallback"]})
        return out

    car_box = {}
    init = T.Tracker.__init__

    def tinit(self, car, track, *a, **kw):
        init(self, car, track, *a, **kw)
        car_box.update(car=self.car, grip=kw.get("grip", 1.0), N=self.N, dt=self.dt,
                       tick=self.tick)

    T.Tracker.step, T.Tracker._params, T.Tracker.__init__ = step, params, tinit
    try:
        res = m4.run(course, arm=arm, tag="acados_record")
    finally:
        T.Tracker.step, T.Tracker._params, T.Tracker.__init__ = orig_step, orig_params, init
    if not ticks:
        raise RuntimeError(f"no Tracker ticks recorded ({res.get('result')})")
    st = lambda k: np.stack([t[k] for t in ticks])            # noqa: E731
    car = car_box["car"]
    os.makedirs(os.path.dirname(os.path.abspath(out_npz)), exist_ok=True)
    np.savez_compressed(
        out_npz, x0=st("x0"), S=st("S"), P=st("P"), REF=st("REF"), AREF=st("AREF"),
        ISN=st("ISN"), u=np.array([t["u"] for t in ticks], float),
        solve_s=np.array([t["solve_s"] for t in ticks]), wall_s=np.array([t["wall_s"] for t in ticks]),
        ok=np.array([t["ok"] for t in ticks]), fallback=np.array([t["fallback"] for t in ticks]),
        meta=json.dumps({"course": course, "arm": arm, "grip": car_box["grip"], "N": car_box["N"],
                         "dt": car_box["dt"], "tick": car_box["tick"], "result": res.get("result"),
                         "car": dataclasses.asdict(car)}))
    print(out_npz, len(ticks), "ticks", res.get("result"), flush=True)
    return out_npz


def _car(d):
    for f in dataclasses.fields(Car):
        if isinstance(f.default, tuple) and isinstance(d.get(f.name), list):
            d[f.name] = tuple(d[f.name])
    return Car(**d)


# -- bench (acados) ----------------------------------------------------------------------------
def _pct(a):
    a = np.asarray(a, float)
    return {"p50": float(np.percentile(a, 50)), "p95": float(np.percentile(a, 95)),
            "p99": float(np.percentile(a, 99)), "max": float(a.max())}


def bench(npz, solver=None):
    """The acados solver over npz's ticks in order (one RTI each, warm from the last tick's
    iterate; from the references on the first tick and after a failed one)."""
    d = np.load(npz)
    meta = json.loads(str(d["meta"]))
    if solver is None:
        solver = build(_car(meta["car"]), n=meta["N"], dt=meta["dt"], grip=meta["grip"])
    keys = ("x0", "S", "P", "REF", "AREF", "ISN")
    statuses, times, U = [], [], []
    reset = True
    for i in range(len(d["x0"])):
        status, u0, tt = solve(solver, {k: d[k][i] for k in keys}, reset=reset)
        statuses.append(status)
        times.append(tt)
        U.append(u0)
        reset = status != 0 or not np.all(np.isfinite(u0))
    statuses, U = np.array(statuses), np.array(U)
    ref = d["u"]
    good = (statuses == 0) & np.all(np.isfinite(ref), axis=1) & np.all(np.isfinite(U), axis=1)
    nmpc = good & ~d["fallback"]
    diff = np.abs(U - ref)

    def deltas(mask):
        if not mask.any():
            return None
        return {"delta_rate_p95": float(np.percentile(diff[mask, 0], 95)),
                "a_p95": float(np.percentile(diff[mask, 1], 95)), "n": int(mask.sum())}

    vals, counts = np.unique(statuses, return_counts=True)
    return {"ticks": int(len(statuses)), "status_counts": {str(int(v)): int(c)
                                                           for v, c in zip(vals, counts)},
            "time_tot": _pct(times),
            "casadi_solve_wall": _pct(d["wall_s"]),
            "abs_diff_vs_casadi": deltas(good), "abs_diff_vs_casadi_nmpc_ticks": deltas(nmpc),
            "casadi_fallback_ticks": int(d["fallback"].sum()),
            "code_dir": solver.code_dir, "cache_hit": bool(solver.cache_hit),
            "course": meta["course"], "grip": meta["grip"], "N": meta["N"], "dt": meta["dt"]}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("what", choices=("record", "bench"))
    ap.add_argument("--course", default=COURSE)
    ap.add_argument("--out", required=True)
    ap.add_argument("--label", default="x86")
    ap.add_argument("--arm", default="single")
    a = ap.parse_args(argv)
    path = _npz_path(a.out, a.course)
    if a.what == "record":
        record(a.course, path, arm=a.arm)
        return 0
    res = bench(path)
    res["label"] = a.label
    out = os.path.join(a.out, f"acados_{a.label}.json")
    with open(out, "w") as fh:
        json.dump(res, fh, indent=1)
    print(out, json.dumps(res), flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
