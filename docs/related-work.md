# Related work: racing and drifting stacks (for option 3)

A survey for option 3 of [autoware-in-a-drift.md](autoware-in-a-drift.md), 2026-10-27. Licenses
from GitHub's API that day ("none": no license file, so all rights reserved: reference only).
Items marked *unverified* were not checked against code.

**The short of it**: no open stack drifts inside Autoware. The racing stacks race at the grip
limit; drifting exists as Stanford DDL and TRI papers and small repositories. Our drift control
comes from the papers; the grip half, the racing line and the estimator have open references.

## Autoware-based

- Autoware Racing working group (F1TENTH, IAC, karts): no published race stack; Autoware on
  F1TENTH exists (TSC update, Sep 2023). Small ports: privvyledge/autoware.f1tenth (Apache-2.0),
  mgmike1011/lqr_lateral_controller (none).
- JSAE AI Challenge, Autoware + AWSIM racing kart: AutomotiveAIChallenge/aichallenge-2024,
  -2025 (Apache-2.0, C++). The closest working example of option 3's seam: own planning and
  control on Autoware's topics, the gate and operation modes kept. Grip only.
- AWSIM + IAC sim races (2023); the IAC car model's availability *unverified*. KAIST's IAC stack
  AndreaFinazzi/nif (C++, none); HaoruXue/spline-trajectory-optimization (Apache-2.0).

## Other ROS 2 racing stacks

- TUM Autonomous Motorsport (Roborace, IAC, A2RL; no full release): global_racetrajectory_optimization
  and trajectory_planning_helpers (LGPL-3.0, Python: minimum curvature / time racing lines);
  GraphBasedLocalTrajectoryPlanner (LGPL-3.0); TUMRT sampling_based_3D_local_planning (GPL-3.0);
  3DVehicleDynamicsStateEstimation (GPL-3.0, C++, ROS 2 Humble: point-mass EKF plus a single-track
  UKF for sideslip); Open-Car-Dynamics (Apache-2.0); TAM__long_acc_control, tam-stability-control
  (Apache-2.0); mod_vehicle_dynamics_control (MATLAB, Roborace). A2RL stack paper arXiv 2602.08571:
  tube MPC over cascaded PID, a 3D EKF, health over ROS 2 diagnostics. GPL / LGPL: write our own
  from the papers, or run theirs as a separate process.
- ForzaETH race_stack (F1TENTH, ros2-humble; none): state estimation, planner, controller, a race
  state machine (pattern for our supervisor; details *unverified*). arXiv 2403.11784.
- F1TENTH: f1tenth_gym, f1tenth_planning (MIT). AMZ Driverless: fssim (MIT); MPCC
  alexliniger/MPCC (Apache-2.0, C++, hpipm), arXiv 1905.05150.

## Drifting, and stacks outside ROS

- Stanford DDL, MARTY: Goh, Goel & Gerdes, JDSMC 2019, "drifting along a general path": path
  tracked through the velocity vector's rotation rate, sideslip through yaw acceleration, model
  inversion and wheel-speed control; entry, hold and exit with no equilibrium. No code; the basis
  for the drift half of a unified controller. Hindiyeh's equilibrium controller in MATLAB:
  mbarnes1/vehicle-drift (none).
- TRI / Stanford Supras: NMPC across grip and drift within road bounds; state from a 250 Hz
  dual-antenna RTK INS; arXiv 2407.13760 (neural front tire in the NMPC), 2306.06330 (learned
  tires). No code. Lesson: their estimate is INS-grade, so ours can start from the simulator's.
- RL drifting: caipeide/drift_drl (MIT, CARLA), zhou-yh19/xcar-rlgpu (MIT), mrsd16teamd/loco_car
  (MIT, RC car). Baselines, not the controller. GT Sophy (Nature 2022): closed.
- RC scale: alexliniger/MPCC (above), urosolia/RacingLMPC (none), MPC-Berkeley/barc (MIT).

## Per component

| component | references | approach |
|---|---|---|
| estimator | TUM 3DVehicleDynamicsStateEstimation; TRI's INS | first the simulator's truth (aib's body velocity) with noise; then a UKF on a single-track Pacejka model (vx, vy, r) fusing NDT, IMU and wheel speeds, filling twist.linear.y |
| global planner | TUM global_racetrajectory_optimization; HaoruXue's spline tool | offline minimum-time line on the ring; per bend a drift segment from the drift equilibrium (beta, r, delta, vx); a file; Autoware's mission planner keeps the route |
| local planner | TUM graph / sampling planners; ForzaETH (pattern) | Frenet sampling to rejoin the line; the trajectory on Autoware's topic, the drift intent (beta, r per point) on a side topic |
| controller | alexliniger/MPCC; Goh/Goel/Gerdes; TRI NMPC | one NMPC (acados or hpipm) on a dynamic bicycle with Pacejka tires, tracking references that include beta and r: grip and drift differ only in the reference; DDL model inversion as a fallback; on /control/trajectory_follower/control_cmd with the gate's limits widened |
| supervisor | TUM A2RL orchestration; ForzaETH state machine | grip / drift entry / hold / exit / abort-to-grip; envelope (beta, r) and track-limit checks reported to the diagnostic graph |
