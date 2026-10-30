"""Where a live L1 solve spends its time, and whether Fatrop does it cheaper (roadmap 061 M3's
solver checks; docs/mode-choice-m3.md, "The solver's cost").

`collect`: live windows as the Driver solves them, on Haruna (asphalt) and Haruna with mu 0.3
patches (mode_choice_e1's courses): a second of plan at a time from s 60, each re-solve warm
from the last plan with its multipliers (the G seed), and at every hairpin ahead a synthetic
rear-out seed (Ds, no multipliers). Each window's inputs are saved (windows.npz).

`ipopt`: each window on the live Window (compiled, MUMPS) with IPOPT's timing statistics:
the share of the solve in the linear solver (factorisation and back-solves), in the problem's
functions, and the rest (IPOPT's own). `fatrop`: the same windows on Fatrop (CasADi's
interface, structure detected), its functions JIT-compiled; plans, iterations and times
against IPOPT's. Results: swb's play_log/race_planner/mode_choice/m3/solver_*.jsonl.

Run (each on one quiet core): env -u PYTHONPATH taskset -c 14 .venv/bin/python
studies/solver_cost.py collect|ipopt|fatrop|report
"""
import json
import math
import os
import re
import sys
import tempfile
import time

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
sys.path.insert(0, HERE)

import mode_choice_e1 as e1  # noqa: E402
from race_planner import modes  # noqa: E402
from race_planner.car import Car  # noqa: E402
from race_planner.line import min_curvature_line  # noqa: E402
from race_planner.live import warm_from  # noqa: E402
from race_planner.qss import speed_profile  # noqa: E402
from race_planner.window import Window, initial  # noqa: E402

OUT = e1.OUT
WINDOWS = os.path.join(OUT, "solver_windows.npz")
COURSES = ("haruna", "haruna-0.3")
LENGTH = 260.0                       # the Driver's window on Haruna


def _setup(name):
    car = Car.from_spec(e1.SPEC)
    t = e1.course(name, car)
    line, _ = min_curvature_line(t, 0.5 * car.width, 0.3)
    ceil = speed_profile(line, car)
    t = t.extended(2 * LENGTH + 16)
    return car, t, ceil


def collect():
    rows = {}
    k = 0
    for name in COURSES:
        car, t, ceil = _setup(name)
        cold = Window(car, length=LENGTH, grip=e1.GRIP, warm=False)
        w = Window(car, length=LENGTH, grip=e1.GRIP, budget=None)
        cs = modes.corners(t, e1.GRIP)
        s_last = float(e1.course(name, car).s[-1])
        s_c = t.s[:len(ceil)]

        def v_end(i0):
            return float(np.interp(t.s[min(i0 + w.N - 1, len(t.s) - 1)], s_c, ceil))

        i = int(np.searchsorted(t.s, 60.0))
        X, U = initial(cold.section(t, i), car, v_start=15.0)
        x0 = X[:, 0].copy()
        x0[0] = 15.0
        X[:, 0] = x0
        plan = cold.solve(t, i, x0, v_end(i), X, U)
        w.lam, w.lam_x = cold.lam, cold.lam_x
        while True:
            j = max(1, int(np.searchsorted(plan["t"], 1.0)))
            i = plan["i0"] + j
            if t.s[i] > s_last - 20:
                break
            x0 = plan["X"][:, j].copy()
            Xw, Uw = warm_from(w, plan, i, x0, t)      # the multipliers shifted with it
            lam, lam_x = w.lam.copy(), w.lam_x.copy()
            sec = w.section(t, i)
            rows[k] = dict(course=name, kind="G", i0=i, x0=x0, v_end=v_end(i), X=Xw, U=Uw,
                           lam=lam, lam_x=lam_x)
            k += 1
            c = next((c for c in cs if 10.0 <= c.s_in - t.s[i] <= LENGTH - 40.0), None)
            if c is not None and c.hairpin:
                pk = modes.rear_peak(car, c.mu, c.sliding)
                Xd, Ud = modes.seed_rear_out(Xw, Uw, sec, c.s_in, c.s_out, car, pk, c.mu, c.sign)
                rows[k] = dict(course=name, kind="Ds", i0=i, x0=x0, v_end=v_end(i), X=Xd, U=Ud)
                k += 1
            new = w.solve(t, i, x0, v_end(i), Xw, Uw)
            print(name, round(t.s[i]), new["status"], new["iters"], round(new["solve_s"], 3),
                  flush=True)
            if not new["ok"]:
                break
            plan = new
    arrays = {}
    for k, r in rows.items():
        for f, v in r.items():
            arrays[f"{k}/{f}"] = np.asarray(v) if not isinstance(v, str) else np.array(v)
    os.makedirs(OUT, exist_ok=True)
    np.savez_compressed(WINDOWS, n=len(rows), **arrays)
    print(len(rows), "windows")


def _windows():
    d = np.load(WINDOWS)
    out = []
    for k in range(int(d["n"])):
        r = {f.split("/", 1)[1]: d[f] for f in d.files if f.startswith(f"{k}/")}
        r["course"], r["kind"] = str(r["course"]), str(r["kind"])
        out.append(r)
    return out


class _Capture:
    """The process's stdout (fd 1: IPOPT's C printing) into a file while in the block."""

    def __enter__(self):
        sys.stdout.flush()
        self.f = tempfile.TemporaryFile(mode="w+")
        self.saved = os.dup(1)
        os.dup2(self.f.fileno(), 1)
        return self

    def __exit__(self, *a):
        sys.stdout.flush()
        os.dup2(self.saved, 1)
        os.close(self.saved)
        self.f.seek(0)
        self.text = self.f.read()
        self.f.close()


TIMERS = ("OverallAlgorithm", "StdAugSystemSolverMultiSolve", "LinearSystemFactorization", "LinearSystemBackSolve",
          "LinearSystemSymbolicFactorization", "LinearSystemScaling", "Function Evaluations",
          "Objective function", "Objective function gradient", "Equality constraints",
          "Inequality constraints", "Equality constraint Jacobian",
          "Inequality constraint Jacobian", "Lagrangian Hessian", "PDSystemSolverTotal",
          "ComputeSearchDirection")


def _timers(text):
    out = {}
    for name in TIMERS:
        m = re.search(r"^\s*" + re.escape(name) + r"\.*:\s*([\d.]+) \(sys:\s*[\d.]+ wall:\s*([\d.]+)\)",
                      text, re.M)
        if m:
            out[name] = float(m.group(2))
    return out


def ipopt(limit=None):
    rows = []
    cache = {}
    for k, r in enumerate(_windows()[:limit]):
        if r["course"] not in cache:
            car, t, _ = _setup(r["course"])
            w = Window(car, length=LENGTH, grip=e1.GRIP, budget=None,
                       ipopt={"print_level": 3, "print_timing_statistics": "yes"})
            cache[r["course"]] = (t, w)
        t, w = cache[r["course"]]
        w.lam = r.get("lam")
        w.lam_x = r.get("lam_x")
        with _Capture() as cap:
            p = w.solve(t, int(r["i0"]), r["x0"], float(r["v_end"]), r["X"], r["U"])
        st = w.solver.stats()
        tm = _timers(cap.text)
        fn = {key[7:]: st[key] for key in st if key.startswith("t_wall_nlp_")}
        rows.append({"k": k, "course": r["course"], "kind": r["kind"], "iters": p["iters"],
                     "status": p["status"], "solve_s": p["solve_s"], "time": p["time"],
                     "ok": p["ok"], "timers": tm, "casadi": fn})
        print(k, r["course"], r["kind"], p["status"], p["iters"], round(p["solve_s"], 3),
              {a: round(b, 3) for a, b in tm.items() if a in ("OverallAlgorithm",
               "LinearSystemFactorization", "LinearSystemBackSolve", "Function Evaluations")},
              flush=True)
    with open(os.path.join(OUT, "solver_ipopt.jsonl"), "w") as f:
        for x in rows:
            f.write(json.dumps(x) + "\n")


def fatrop_solver(w, variant, fatrop_opts):
    """Fatrop on window w's problem. Its structure detection takes the shooting gaps as
    x_k+1 - F(x_k, u_k) (the window writes F - x_k+1): those rows' sign flipped. `expand`: the
    problem expanded to SX, CasADi's own derivatives in its virtual machine; `compiled`: the
    window's own per-stage derivatives (sign-flipped likewise), C-compiled as IPOPT's are."""
    import casadi as ca
    from race_planner.window import _compiled_nlpsol
    n = len(w.lbg_all)
    sg = np.ones(n)
    for k in range(w.M):
        sg[k * w.per_stage:k * w.per_stage + 6] = -1.0
    D = ca.DM(sg)
    nlp = dict(w.nlp)
    nlp["g"] = w.nlp["g"] * D
    eq = [bool(v) for v in (w.lbg_all == w.ubg_all)]
    opts = {"structure_detection": "auto", "equality": eq, "print_time": False,
            "error_on_fail": False, "fatrop": fatrop_opts}
    if variant == "expand":
        return ca.nlpsol("f", "fatrop", nlp, dict(opts, expand=True))
    W, P = w.nlp["x"], w.nlp["p"]
    g, J = w.own["nlp_jac_g"](W, P)
    Dm = ca.DM(ca.Sparsity.diag(n), sg)
    jac = ca.Function("nlp_jac_g", [W, P], [g * D, ca.mtimes(Dm, J)], ["x", "p"],
                      ["g", "jac_g_x"])
    lf, lg = ca.MX.sym("lam_f"), ca.MX.sym("lam_g", n)
    Ht = w.own["nlp_hess_l"](W, P, lf, lg * D)        # the upper triangle; Fatrop takes it all,
    H = Ht + ca.transpose(ca.triu(Ht, False))          # and the Lagrangian's gradient with it
    gl = lf * w.own["nlp_grad_f"](W, P)[1] + ca.mtimes(ca.transpose(ca.mtimes(Dm, J)), lg)
    hess = ca.Function("nlp_hess_l", [W, P, lf, lg], [gl, H],
                       ["x", "p", "lam_f", "lam_g"], ["grad_gamma_x", "hess_gamma_x_x"])
    grad = ca.Function("nlp_grad_f", [W, P], [w.own["nlp_grad_f"](W, P)[1]], ["x", "p"],
                       ["grad_f_x"])                  # Fatrop asks for the gradient alone
    own = {"nlp_jac_g": jac, "nlp_hess_l": hess, "nlp_grad_f": grad}
    return _compiled_nlpsol(nlp, opts, own, plugin="fatrop")


# Fatrop 1.1.8 (CasADi 3.8.2's) takes no multipliers to start from, so a re-plan's warm start is
# its primal one only; `lowmu`: its barrier started lower, as the Driver's warm IPOPT's is
FATROP_EXTRA = {"lowmu": {"mu_init": 1e-2, "bound_push": 1e-4}}


def fatrop(variant="compiled", limit=None):
    rows = []
    cache = {}
    fopts = {"print_level": 5, "max_iter": 400, "tol": 1e-3, **FATROP_EXTRA.get(variant, {})}
    for k, r in enumerate(_windows()[:limit]):
        if r["course"] not in cache:
            car, t, _ = _setup(r["course"])
            w = Window(car, length=LENGTH, grip=e1.GRIP, budget=None)
            c = time.time()
            fs = fatrop_solver(w, "compiled" if variant == "lowmu" else variant, fopts)
            print("fatrop built", round(time.time() - c, 1), "s", flush=True)
            cache[r["course"]] = (t, w, fs)
        t, w, fs = cache[r["course"]]
        i0 = int(r["i0"])
        P, wl, wr, s = w.params(t, i0)
        half = 0.5 * w.car.width
        Pw = np.vstack([P, wl[:w.M] - half - w.margin, wr[:w.M] - half - w.margin])
        lbx, ubx = w.bounds(wl, wr, r["x0"], float(r["v_end"]))
        U = np.array(r["U"], float)
        x_init = np.clip(w.pack(r["X"], U), lbx, ubx)
        args = {"x0": x_init, "p": Pw.ravel(order="F"), "lbx": lbx, "ubx": ubx,
                "lbg": w.lbg_all, "ubg": w.ubg_all}
        c = time.time()
        with _Capture() as cap:
            sol = fs(**args)
        dt = time.time() - c
        st = fs.stats()
        m = re.search(r"^fatrop\s+([\d.]+)", cap.text, re.M)
        fe = re.search(r"^function evaluation\s+([\d.]+)", cap.text, re.M)
        Xv, Uv = w.unpack(sol["x"])
        p = w._result(P, s, Xv, Uv, bool(st["success"]), st["return_status"],
                      st.get("iter_count", -1), dt, i0)
        rows.append({"k": k, "course": r["course"], "kind": r["kind"], "iters": p["iters"],
                     "status": p["status"], "solve_s": dt, "time": p["time"], "ok": p["ok"],
                     "defect": p["defect"], "violation": p["violation"],
                     "fatrop_s": float(m.group(1)) if m else None,
                     "functions_s": float(fe.group(1)) if fe else None})
        print(k, r["course"], r["kind"], p["status"], p["iters"], round(dt, 3), round(p["time"], 3),
              p["ok"], rows[-1]["fatrop_s"], rows[-1]["functions_s"], flush=True)
    with open(os.path.join(OUT, f"solver_fatrop_{variant}.jsonl"), "w") as f:
        for x in rows:
            f.write(json.dumps(x) + "\n")


def report():
    ip = [json.loads(x) for x in open(os.path.join(OUT, "solver_ipopt.jsonl"))]
    lines = ["# L1's solve: where the time goes (IPOPT, MUMPS, compiled), and Fatrop\n"]
    say = lines.append
    for kind in ("G", "Ds", None):
        xs = [x for x in ip if kind is None or x["kind"] == kind]
        tot = np.array([x["timers"].get("OverallAlgorithm", np.nan) for x in xs])
        # MUMPS factorises inside the augmented system's solve (IPOPT's LinearSystemFactorization
        # stays 0 with it): the linear solver's time is that solve's
        lin = np.array([x["timers"].get("StdAugSystemSolverMultiSolve", 0) for x in xs])
        fe = np.array([x["timers"].get("Function Evaluations", 0) for x in xs])
        it = np.array([x["iters"] for x in xs])
        say(f"- {kind or 'all'}: {len(xs)} windows, iterations p50 {np.median(it):.0f} p95 "
            f"{np.percentile(it, 95):.0f}; wall p50 {np.median(tot):.3f} p95 "
            f"{np.percentile(tot, 95):.3f} s; ms/iteration {1000 * tot.sum() / it.sum():.2f}; "
            f"linear solver {100 * lin.sum() / tot.sum():.1f} %, functions "
            f"{100 * fe.sum() / tot.sum():.1f} %, the rest (IPOPT) "
            f"{100 * (tot.sum() - lin.sum() - fe.sum()) / tot.sum():.1f} %")
        fn = {}
        for x in xs:
            for a, b in x["casadi"].items():
                fn[a] = fn.get(a, 0.0) + b
        say("  functions (CasADi's wall, % of the total): " + ", ".join(
            f"{a} {100 * b / tot.sum():.1f}" for a, b in sorted(fn.items()) if a != "total"))
    for variant in ("expand", "compiled", "lowmu"):
        p = os.path.join(OUT, f"solver_fatrop_{variant}.jsonl")
        if not os.path.exists(p):
            continue
        fa = {x["k"]: x for x in map(json.loads, open(p))}
        say(f"\n## Fatrop ({variant}) against IPOPT, window by window\n")
        both = [(x, fa[x["k"]]) for x in ip if x["k"] in fa]
        same = [abs(a["time"] - b["time"]) / a["time"] for a, b in both if a["ok"] and b["ok"]]
        say(f"- windows {len(both)}; ok IPOPT {sum(a['ok'] for a, _ in both)}, Fatrop "
            f"{sum(b['ok'] for _, b in both)}; window time differs by p50 "
            f"{100 * np.median(same) if same else float('nan'):.3f} %, max "
            f"{100 * max(same) if same else float('nan'):.3f} %")
        for kind in ("G", "Ds"):
            sel = [(a, b) for a, b in both if a["kind"] == kind]
            if not sel:
                continue
            ia, ib = [a["iters"] for a, _ in sel], [b["iters"] for _, b in sel]
            wa, wb = [a["solve_s"] for a, _ in sel], [b["solve_s"] for _, b in sel]
            say(f"- {kind}: iterations p50/p95 IPOPT {np.median(ia):.0f}/{np.percentile(ia, 95):.0f}, "
                f"Fatrop {np.median(ib):.0f}/{np.percentile(ib, 95):.0f}; wall p50/p95 IPOPT "
                f"{np.median(wa):.3f}/{np.percentile(wa, 95):.3f} s, Fatrop "
                f"{np.median(wb):.3f}/{np.percentile(wb, 95):.3f} s; Fatrop statuses "
                f"{sorted(set(b['status'] for _, b in sel))}")
    text = "\n".join(lines) + "\n"
    open(os.path.join(OUT, "solver_report.md"), "w").write(text)
    print(text)


if __name__ == "__main__":
    what = sys.argv[1]
    if what == "fatrop":
        fatrop(sys.argv[2] if len(sys.argv) > 2 else "compiled",
               int(sys.argv[3]) if len(sys.argv) > 3 else None)
    else:
        lim = int(sys.argv[2]) if len(sys.argv) > 2 else None
        {"collect": collect, "ipopt": lambda: ipopt(lim), "report": report}[what]()
