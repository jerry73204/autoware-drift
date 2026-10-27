#!/usr/bin/env python3
"""A lap's bag through each drift (swb roadmap 057): what the drift did, and what Autoware made of it.

    tools/analyse_lap.py <bag dir or .mcap>      (Autoware 1.5's environment sourced)

Reads, when recorded on the ego's domain:
  /drift/state                                  the controller's phases (JSON)
  /vehicle/status/velocity_status               the car's body velocity from the vehicle bridge:
                                                the simulator's truth (vx, vy, yaw rate)
  /localization/kinematic_state                 the EKF's pose and twist
  /localization/pose_estimator/pose_with_covariance   NDT's pose
  /localization/pose_estimator/transform_probability, .../nearest_voxel_transformation_likelihood
  /api/operation_mode/state, /api/fail_safe/mrm_state, /autoware/state

Prints, per drift (from /drift/state's phase changes): the bend, entry speed, the true sideslip
held (mean and peak, from the velocity report), the controller's own sideslip estimate against
it, the EKF's distance from NDT's pose at each NDT fix (mean, max), NDT's score (min), how the
drift ended (handed back, rejoined, recovered, aborted) and any operation-mode or MRM change in
it; then the lap's mode and MRM timeline.
"""
import json
import math
import sys
from bisect import bisect_left

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamp(msg, t):
    h = getattr(msg, "header", None) or getattr(msg, "stamp", None)
    st = getattr(h, "stamp", h)
    return st.sec + 1e-9 * st.nanosec if st is not None and hasattr(st, "sec") else t


def read(path):
    r = rosbag2_py.SequentialReader()
    storage = "mcap" if path.endswith(".mcap") else ""
    r.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage),
           rosbag2_py.ConverterOptions("", ""))
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    out = {}
    while r.has_next():
        topic, data, t = r.read_next()
        if topic not in types:
            continue
        try:
            msg = deserialize_message(data, get_message(types[topic]))
        except Exception:  # noqa: BLE001 - a type this environment lacks
            continue
        out.setdefault(topic, []).append((t * 1e-9, msg))
    return out


def nearest(series, t):
    ts = [s[0] for s in series]
    i = bisect_left(ts, t)
    best = min((j for j in (i - 1, i) if 0 <= j < len(series)), key=lambda j: abs(ts[j] - t),
               default=None)
    return series[best] if best is not None else None


def main(path):
    b = read(path)
    states = [(t, json.loads(m.data)) for t, m in b.get("/drift/state", [])]
    vel = [(t, m.longitudinal_velocity, m.lateral_velocity, m.heading_rate)
           for t, m in b.get("/vehicle/status/velocity_status", [])]
    ekf = [(t, m.pose.pose.position.x, m.pose.pose.position.y)
           for t, m in b.get("/localization/kinematic_state", [])]
    ndt = [(t, m.pose.pose.position.x, m.pose.pose.position.y)
           for t, m in b.get("/localization/pose_estimator/pose_with_covariance", [])]
    score = [(t, m.data) for t, m in
             b.get("/localization/pose_estimator/nearest_voxel_transformation_likelihood",
                   b.get("/localization/pose_estimator/transform_probability", []))]
    modes = [(t, m.mode, m.is_autonomous_mode_available)
             for t, m in b.get("/api/operation_mode/state", [])]
    mrm = [(t, m.state, m.behavior) for t, m in b.get("/api/fail_safe/mrm_state", [])]

    # drifts: from a phase "brake" to the next "cruise"
    drifts, cur = [], None
    for t, s in states:
        ph = s["phase"]
        if ph == "brake" and cur is None:
            cur = {"t0": t, "bend": s.get("point"), "entry": s.get("speed"), "est": [], "end": None}
        if cur is not None:
            if ph in ("hold",):
                cur["est"].append(s.get("beta_deg"))
            if ph == "cruise":
                cur["t1"], cur["end"] = t, (s.get("log") or ["", ""])[1]
                drifts.append(cur)
                cur = None
    if cur is not None:
        cur["t1"], cur["end"] = states[-1][0], "(bag ended)"
        drifts.append(cur)

    print(f"{len(drifts)} drift(s); {len(states)} state reports, {len(vel)} velocity reports, "
          f"{len(ekf)} EKF, {len(ndt)} NDT poses")
    for d in drifts:
        t0, t1 = d["t0"], d["t1"]
        v = [x for x in vel if t0 <= x[0] <= t1 and x[1] > 1.0]
        betas = [math.degrees(math.atan2(x[2], x[1])) for x in v]
        gaps = []
        for tn, xn, yn in (n for n in ndt if t0 <= n[0] <= t1):
            e = nearest(ekf, tn)
            if e:
                gaps.append(math.hypot(e[1] - xn, e[2] - yn))
        sc = [s for t, s in score if t0 <= t <= t1]
        m_in = [m for m in modes if t0 <= m[0] <= t1]
        r_in = [r for r in mrm if t0 <= r[0] <= t1]
        print(f"- {d['bend']}: {t1 - t0:5.1f} s, entry {3.6 * (d['entry'] or 0):5.1f} km/h, "
              f"true sideslip mean {sum(betas) / len(betas):+6.1f} peak {min(betas, key=lambda b: -abs(b)):+6.1f} deg"
              if betas else f"- {d['bend']}: no velocity reports", end="")
        if d["est"]:
            print(f", controller's estimate mean {sum(d['est']) / len(d['est']):+6.1f}", end="")
        if gaps:
            print(f", EKF-NDT gap mean {sum(gaps) / len(gaps):.2f} max {max(gaps):.2f} m", end="")
        if sc:
            print(f", NDT score min {min(sc):.2f}", end="")
        print(f"; ended: {d['end']}")
        for m in m_in:
            print(f"    mode {m[1]} autonomous_available {m[2]} at {m[0] - t0:+.1f} s")
        for r in r_in:
            print(f"    MRM state {r[1]} behavior {r[2]} at {r[0] - t0:+.1f} s")
    print("modes:", [(round(t - (modes[0][0] if modes else 0), 1), m, a) for t, m, a in
                     [modes[i] for i in range(len(modes)) if i == 0 or modes[i][1:] != modes[i - 1][1:]]])
    print("mrm:", [(round(t - (mrm[0][0] if mrm else 0), 1), s, bh) for t, s, bh in
                   [mrm[i] for i in range(len(mrm)) if i == 0 or mrm[i][1:] != mrm[i - 1][1:]]])


if __name__ == "__main__":
    main(sys.argv[1])
