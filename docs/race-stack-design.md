# The race stack: option 3, designed

A design to review before anything is built (user, 2026-10-27: "Try option 1. If it's not
working, go option 3 and explore the design first"). Decisions (user, 2026-10-28): we build our
own planner stack; the controller is C++; supervision changes Autoware's stock monitoring to
fit racing rather than adding a supervisor of our own. Option 1 ([autoware-in-a-drift.md](autoware-in-a-drift.md))
got laps and drifts on Town05's ring, but each lap found a new way Autoware and a bolted-on
drift law disagree, and the margins are thin; this is the stack that removes the disagreement.
References: [related-work.md](related-work.md); the ring's physics: [ring-envelope.md](ring-envelope.md).

## What option 1 taught (laps 1-28, server-241, 2026-10-27/28)

| lesson | lap | consequence for the design |
|---|---|---|
| Autoware's road checks fire in a drift: lane departure, AEB, control state, trajectory shift, the planner's lateral acceleration (point-to-point curvature reads lanelet joins as R 61 m) | 10-25 | race supervision replaces road supervision; Autoware's MRM stays only for real faults |
| Autoware's planning shapes speed for comfort: 1 m/s^2, a 200 / 180 m horizon that ends in a stop, curvature over 2 m, the gate's 25 m/s | 11-24 | the race planner owns the speed profile; Autoware's planner does not shape it |
| Two controllers that hand over disagree: MPC integrates during the override, the hand-back needs the car back on Autoware's line, an emergency stop holds the wheel straight in a bend | 17-25 | one controller for grip and drift, no hand-over |
| A drift's line depends on entry speed, entry point and a path term; a hold with no path feedback circles 70 m in a 104 m bend | 27-28 | the drift is planned (entry, sideslip, radius, exit) per bend and tracked as a path, not triggered at a point |
| State: the EKF has no lateral velocity; NDT differences are 10 Hz and late; the vehicle report is the simulator's truth | 19-21 | a slip-aware estimator; until it exists, the vehicle report |
| NDT: an oscillating step to the iteration limit, then a cascade of skipped scans | 24, issue 017 | the estimator gives NDT its guess and survives a skipped scan |

## Architecture

Autoware keeps what a race needs from it and gives up what it cannot do. The seams are
Autoware's own, so SSv2, the relay and swb stay as they are.

```
                    Autoware (kept)                              race stack (new, autoware-drift)
 SSv2 -> relay -> ADAPI -> mission planner -- route --------------> race planner
                          (routing, goals)                          | offline: racing line + per-bend drift plan
 sensors -> NDT ------------------------------- pose -------------> race estimator (vx, vy, r, pose)
       ^      '-- initial guess <------------------------------------'   |   (IMU, wheel speeds, NDT)
       |                                                                 v
       |                                    /planning/scenario_planning/trajectory  <- race planner (online:
       |                                                                 |              line + rejoin, with the
       |                                    + /race/plan (sideslip, yaw rate per point)  drift intent)
       |                                                                 v
       |                        /control/trajectory_follower/control_cmd <- race controller (one NMPC:
       |                                                                 |                   grip and drift)
       |    vehicle_cmd_gate (kept, its limits opened) <-----------------'
       |    operation modes, MRM handler, ADAPI (kept)
       |    diagnostic graph: race supervisor's units in place of the road checks
       '--- vehicle (aib on Isaac, its control_topic back to /control/command/control_cmd)
```

- **Kept**: map, localization's NDT (with the estimator's guess), perception (unused on an empty
  ring, needed later), the mission planner and ADAPI routing, the gate, the operation-mode
  transition manager, the MRM handler, the diagnostic graph aggregator.
- **Replaced**: scenario planning's output (the profile's relay `/planning/trajectory ->
  /planning/scenario_planning/trajectory` is the seam: off, and the race planner publishes the
  latter; Autoware's scenario planning can keep running unused, or be cut by a profile
  argument), the trajectory follower (`trajectory_follower_mode:=none`; the race controller
  publishes the follower's topic), the EKF's role as the motion state (it may keep publishing
  `/localization/kinematic_state` for Autoware's own consumers).
- **Gone**: drift_controller's override between the gate and the vehicle; aib's ego config goes
  back to the default `control_topic`.

## Components

### Race estimator (first)

- **State**: x, y, yaw, vx, vy, r (and ax, ay); a dynamic single-track model with the spec's
  tire (sim_vehicle_spec: stiffness, mu) as the process model.
- **Measurements**: NDT pose (with its NVTL as the covariance), IMU yaw rate and specific force,
  the four wheel speeds (the front pair is undriven: vx through a locked rear axle is not), the
  vehicle's steering report.
- **Out**: `/race/state` (nav_msgs/Odometry with twist.linear.y), and NDT's initial guess
  (`/localization/pose_estimator/...` input; or as the EKF's replacement on
  `/localization/kinematic_state`, once proven).
- **Recovery**: after N skipped NDT scans, re-align from the last good pose propagated by the
  model (issue 017's cascade).
- **Stage 0**: the vehicle report (aib's body velocity: what drift_controller uses today), so the
  planner and controller can be built before the estimator; stage 1 the filter, judged against
  that truth on recorded laps.
- **References**: TUM 3DVehicleDynamicsStateEstimation (point-mass EKF + single-track UKF for
  sideslip; GPL-3.0: written anew from the paper, not copied).

### Race planner

- **Offline** (a tool, run once per track; output a file in the package):
  - the track: the ring's road boundaries from the lanelet map (the three lanes plus shoulder,
    less a margin to the poles found inside the SW bend);
  - the racing line: minimum curvature, then a speed profile from the car's g-g diagram (the
    spec's tire and power; ring-envelope.md's numbers), TUM global_racetrajectory_optimization's
    method (LGPL-3.0: run as a separate offline tool or re-implemented);
  - per bend, the drift plan: entry point and speed, the drift equilibrium (sideslip, yaw rate,
    steer, rear slip; Velenis-Frazzoli-Tsiotras balances, ring-envelope.md), the arc it holds
    and the exit point, overlaid on the racing line.
- **Online** (a node at 10-20 Hz): from the estimator's state, the plan ahead
  (`/planning/scenario_planning/trajectory`, Autoware's type: pose and speed per point) and the
  drift intent the type cannot carry (`/race/plan`: sideslip and yaw rate per point, a custom
  message); after a deviation (a wide drift, a recovery), a rejoin to the line within the track
  (Frenet sampling, TUM's graph planner as the pattern).
- The mission planner's route stays the authority on where the car goes (SSv2's goals); the
  race planner follows the route's lanelets around the ring.

### Race controller

- **One controller for grip and drift**: a nonlinear MPC on the dynamic single-track model with
  the spec's tire, tracking the plan's path, speed, sideslip and yaw rate; grip and drift differ
  only in the reference (sideslip 0 or the plan's). Steering and the rear axle's drive
  (acceleration request, which sim_vehicle_ecu turns into torque through the locked axle) are its
  inputs; the steering lock, rate and drive limits its constraints; the track boundaries as soft
  constraints.
- **C++** (user, 2026-10-28), a ROS 2 component like Autoware's own controllers, in this
  repository's `src/control/`.
- **Solver**: acados (BSD-2-Clause, C code generated from the model, real time at 50-100 Hz for
  a 1-2 s horizon; the model and cost are written once in its Python or MATLAB front end and the
  generated C is what the C++ node links) or Liniger's MPCC formulation (Apache-2.0, C++, hpipm)
  for the contouring cost.
- **Fallback**: the existing drift law (law.py) with its path term, for the proving ground and as
  the controller while the MPC is built; Goh/Goel/Gerdes's model inversion as a second option.
- **Out**: `/control/trajectory_follower/control_cmd` (Autoware's Control), so the gate, the
  transition manager and the MRM handler see a normal follower.

### Supervision: Autoware's own monitoring, set for racing

No supervisor of our own (user, 2026-10-28): the stock monitors stay, configured for a race
track instead of a road, and judge the race stack as they judge Autoware's planner and follower.

| stock monitor | on a road | for racing |
|---|---|---|
| diagnostic graph (`autoware_diagnostic_graph_aggregator`) | road checks under each operation mode | a race graph: the race nodes' topic rates and node health in place of the planner's and follower's; the road checks out of the autonomous mode (option 1's `drift.yaml` is its start) |
| lane departure checker (`autoware_lane_departure_checker`) | the footprint against the lane | the footprint against the track's road boundary (its `boundary_types_to_detect` and margins), so leaving the track, not the lane, is the fault |
| control validator (`autoware_control_validator`) | the follower's deviation from the trajectory, 1 m / 1 rad | its thresholds at a drift's: the plan's line with the sideslip and lateral error the plan allows |
| planning validator (`autoware_planning_validator`) | curvature point to point, lateral acceleration 9.8, the trajectory's shift | the race planner's trajectory with the race car's limits (g-g diagram), curvature over the plan's own spacing |
| AEB, obstacle collision checker | obstacles on the path | kept for traffic (the ring is empty today), off until perception is part of the race |
| localization monitors (`ndt_scan_matcher` diagnostics, `localization_error_monitor`) | NDT's state, the EKF's covariance | kept, reading the race estimator's covariance; NDT's exe-time WARN not a fault (option 1's `warn-to-ok`) |
| MRM handler, emergency stop operator | a comfortable or emergency stop | kept; the emergency stop holds the wheel, so the race controller steers it along the plan (as drift_controller does since lap 25) |

What the stock monitors cannot see -- a spin (sideslip and yaw rate outside the recoverable
envelope) -- is the race controller's own diagnostic, published under the control unit of the
race graph, so a spin is an MRM like any fault.
## Interfaces (new)

| topic | type | from -> to |
|---|---|---|
| `/race/state` | nav_msgs/Odometry (+ twist.linear.y) | estimator -> planner, controller, supervisor |
| `/race/plan` | `race_msgs/RacePlan` (per point: s, pose, speed, sideslip, yaw rate, mode grip/drift) | planner -> controller, supervisor |
| `/planning/scenario_planning/trajectory` | autoware_planning_msgs/Trajectory | planner -> Autoware (gate, transition manager, validator) |
| `/control/trajectory_follower/control_cmd` | autoware_control_msgs/Control | controller -> gate |
| diagnostics of the race nodes (rates, the controller's envelope) | diagnostic_msgs | race nodes -> the race graph |

## Milestones

1. **Estimator stage 0 + offline planner**: the racing line and drift plans for Town05's ring
   from the map; plotted against the lane and the poles. No car.
2. **Controller on the model**: the NMPC in Python (acados' Python interface) against model.py's
   single-track car (extended to four wheels' loads and the locked axle), grip laps and drifts.
3. **Proving ground**: the controller in swb's manoeuvre spike (as the `drift` case does today):
   a drift on a circle, then a figure eight (both directions, transitions).
4. **The ring, through Autoware's seams**: planner + controller nodes, `trajectory_follower_mode:=none`,
   the relay off; laps with drifts at every bend; the supervisor's graph in place of option 1's
   pruned one.
5. **Estimator stage 1**: the filter replacing the vehicle report, judged on recorded laps.

## Decided, and still open

- **Decided** (user, 2026-10-28): our own planner stack; the controller in C++; Autoware's stock
  monitoring set for racing in place of a supervisor of our own.
- **TUM's GPL / LGPL code**: what each does and how it may be used is below ("TUM's code").
- **Drift, measured** (`drift-optimality.md`, 2026-10-30): on asphalt a drift is optimal only
  in tight hairpins and worth at most 0.4 % on swb's asphalt; on low friction 1-2 %. Open for
  low friction: the planner's tyre on each surface's own peak slip angle, the open axle in L1,
  and a mode choice (multi-start, or a GPU sampler) over the grip and drift optima.
- **Open**: how much of Autoware to keep long term (perception and NDT are needed only if the
  ring gets traffic or the state must come from sensors).

## TUM's code: what it is for, and how we may use it

TUM's autonomous racing group (Roborace, Indy Autonomous Challenge, A2RL) published parts of
their stack. Each is a working answer to one of our components, so it is the reference to read
first; their licenses decide whether we may run it, link it, or only learn from it.

| repository | license | what it does | our component | how we may use it |
|---|---|---|---|---|
| global_racetrajectory_optimization | LGPL-3.0, Python | offline: from a track's boundaries, the racing line (minimum curvature, iterative minimum curvature, or minimum time) and its speed profile from a g-g-v diagram | global planner | run it as an offline tool on Town05's ring: its output (a line, a CSV) is ours, so no license reaches our code; changes to the tool itself would stay LGPL |
| trajectory_planning_helpers | LGPL-3.0, Python | its library: splines, curvature, the forward-backward speed profile | global planner | as above, through the tool |
| GraphBasedLocalTrajectoryPlanner | LGPL-3.0, Python | online: a lattice of offsets around the racing line, the best collision-free path each cycle (rejoins, overtakes) | local planner | LGPL lets a program use it as a library without becoming LGPL; but our planner node is C++ and it is Python, so it is the design to follow, rewritten |
| sampling_based_3D_local_planning | GPL-3.0, Python | A2RL's local planner: sampled trajectories on a 3D track | local planner | GPL: code taken into ours makes ours GPL; read the paper, write our own |
| 3DVehicleDynamicsStateEstimation | GPL-3.0, C++, ROS 2 | the state estimator: a point-mass EKF for pose and velocity, a single-track UKF for sideslip, an IMU handler | estimator | GPL: it could run unmodified as its own ROS node beside ours (separate programs talking over topics), but linking or copying it makes ours GPL; we write ours from its paper, since the estimator is ours to tune for drifting |

So: the racing line can come from TUM's tool directly (offline, its output ours); the local
planner and the estimator are written by us in C++ following TUM's designs; nothing of TUM's is
linked into this Apache-2.0 repository.
