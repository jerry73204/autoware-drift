"""Plan a section of a course: the minimum-time trajectory, saved for a tracker.

    python -m race_planner.plan <map dir> <spec.yaml> <surface_classes.yaml> <out.npz>
        [--s0 20] [--s1 620] [--v-start 1] [--ds 2] [--diff locked|open] [--grip 1]

<map dir> is swb's course generator's (course/centreline.npz, meshes.npz, surfaces.yaml).
--grip, the share of the road's friction the plan uses (a margin for
the tracker). The npz holds, per node along the section: s, t, x, y (map frame), the
path's heading psi_path, yaw (the body's: psi_path - beta), vx, vy, r, delta, beta, speed, n, and
torque (fl, fr, rl, rr, N m, > 0 driving), plus time (s) and the car's name.
"""
import argparse
import dataclasses

import numpy as np

from race_planner.car import Car
from race_planner.ocp import solve_min_time
from race_planner.track import SurfaceTable, Track

FIELDS = ("s", "x", "y", "z", "psi", "kappa", "w_left", "w_right", "mu", "grade", "sliding")


def section(track, s0, s1):
    keep = (track.s >= s0) & (track.s <= s1)
    return dataclasses.replace(track, **{k: getattr(track, k)[keep] for k in FIELDS})


def plan(map_dir, spec, classes, s0=20.0, s1=620.0, v_start=1.0, ds=2.0, diff=None,
         grip=1.0):
    car = Car.from_spec(spec)
    if diff:
        car = dataclasses.replace(car, diff=diff)
    track = section(Track.from_swb_course(map_dir, SurfaceTable.from_yaml(classes), car)
                    .resampled(ds), s0, s1)
    sol = solve_min_time(track, car, v_start=v_start, grip=grip)
    x, y = sol["x"], sol["y"]
    psi_path = np.unwrap(np.arctan2(np.gradient(y), np.gradient(x)))
    out = {k: sol[k] for k in ("s", "t", "x", "y", "vx", "vy", "r", "delta", "beta", "n")}
    out.update(psi_path=psi_path, yaw=psi_path - sol["beta"],
               speed=np.hypot(sol["vx"], sol["vy"]),
               torque=np.vstack([sol["torque"], sol["torque"][-1:]]))
    return sol, out, car


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("map_dir")
    ap.add_argument("spec")
    ap.add_argument("classes")
    ap.add_argument("out")
    ap.add_argument("--s0", type=float, default=20.0)
    ap.add_argument("--s1", type=float, default=620.0)
    ap.add_argument("--v-start", type=float, default=1.0)
    ap.add_argument("--ds", type=float, default=2.0)
    ap.add_argument("--diff", choices=("locked", "open"))
    ap.add_argument("--grip", type=float, default=1.0)
    a = ap.parse_args()
    sol, out, car = plan(a.map_dir, a.spec, a.classes, a.s0, a.s1, a.v_start, a.ds, a.diff,
                         a.grip)
    np.savez(a.out, time=sol["time"], car=car.name, **out)
    worst = max(sol["violation"].items(), key=lambda kv: kv[1][0])
    print(f"{sol['status']}: {sol['time']:.2f} s over {out['s'][-1] - out['s'][0]:.0f} m, "
          f"worst violation {worst[0]} {worst[1][0]:.3g} at s {worst[1][1]:.0f} -> {a.out}")
    return 0 if sol["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
