# Autoware in a drift: what breaks, and a race stack in its place

Written 2026-10-27, after the first Town05 laps (swb roadmap 057). Autoware 1.5.0's planning and
control are built for a road car that stays in its lane and in grip: a kinematic model, a
trajectory along the lane, and monitors that call any large departure from it a fault. A drift
is a large departure from it by design. This note goes through each part of the stack against
a drift, then proposes what to build instead.

## How the project drives today

`drift_controller` sits after Autoware's command gate: the car takes
`/control/command/drift/control_cmd`, which is Autoware's `/control/command/control_cmd`
passed through outside a drift and the drift law's command inside one (BRAKE, TURN_IN, HOLD,
EXIT, RECOVER). Autoware never learns that someone else is driving. During a drift it keeps
planning the lane, keeps running its controller against a car that is not doing what it asks,
and keeps judging the result.

## What a drift does to each part of Autoware

### Localization: the EKF has no lateral velocity

`autoware_ekf_localizer`'s state is (x, y, yaw, yaw bias, vx, yaw rate): it moves the car along
its heading. The vehicle bridge does report the lateral velocity (aib's `VelocityReport`
carries the body's `lateral_velocity`), but the EKF has nowhere to put it. At 29 m/s and 15 deg
sideslip the car moves 7.5 m/s sideways that the prediction ignores: 0.75 m between two NDT
poses at 10 Hz, 1.5 m at 30 deg. Two consequences:

- NDT takes the EKF's pose as its initial guess; a guess that is off by a metre or more is
  where scan matching starts to fail or converge to a wrong local minimum.
- The EKF gates NDT's pose by Mahalanobis distance (`pose_gate_dist` 49.5). With a small
  covariance and a prediction that is wrong in a consistent direction, correct NDT poses can
  be rejected, and then the EKF runs on its own model. The localization diagnostics
  (`/autoware/localization/accuracy`, `scan_matching_status`) are leaves of
  `/autoware/modes/autonomous`.

`/localization/kinematic_state` also says sideslip 0 throughout (which is why
`drift_controller` estimates sideslip from consecutive NDT poses itself). This is the deepest
problem: everything downstream, Autoware's or ours, needs a pose that holds through a slide.

### Planning: a lane-keeping trajectory, and checks against it

Behavior and motion planning produce a trajectory along the lane centre at the smoother's
speed. In a drift the car's heading is 15-30 deg off that trajectory's yaw and its line can be
metres off the centre, so:

- the planning validator's `trajectory_shift` and `yaw_deviation` checks compare consecutive
  trajectories near the ego; a car that has moved off the line makes the planner re-anchor
  and the shift checks fire (`/autoware/planning/trajectory_validation/*`, all leaves of
  autonomous mode);
- a car that leaves the lanelets (onto the shoulder, or spun) is off the route: the behavior
  path planner has no lane to plan in, and the trajectory stops or vanishes;
- obstacle stop treats anything inside the footprint as an obstacle and trims the trajectory
  (this is what held lap 1-5 at a standstill: the Mustang's lidar saw its own hood).

### Control: an MPC on a kinematic model, then validators

- The trajectory follower (MPC lateral, PID longitudinal) keeps running while the drift law
  drives. Its integrators, steering-rate limits and delay compensation build up against a car
  it does not control, and at hand-back its first command comes from that stale state. The
  MPC's vehicle model is kinematic or linear-tire; past the grip limit its predictions are
  wrong, so it cannot catch a car that is still sliding.
- `control_validator` (`max_distance_deviation` 1.0 m, `yaw_deviation_error` 1.0 rad, over-
  velocity 2 m/s + 20 %, rolling back, acceleration error) is a leaf of `/autoware/control`.
  A drift line one metre off the lane centre is an ERROR.
- `lane_departure_checker` (boundary departure on curbstones) and AEB
  (`autonomous_emergency_braking`, which predicts the ego's path from yaw rate and speed and
  brakes for points on it) are leaves too. AEB's predicted path in a slide is a circle the car
  is not on.
- The command gate's limits (lateral acceleration, steering rate, jerk) apply to Autoware's
  own command only; the drift law's command bypasses them, which is intended.

### System: any of the above ends in MRM

Every leaf above is part of `/autoware/modes/autonomous`. When one goes ERROR, autonomous
becomes unavailable and the MRM handler takes over: the profile has `use_comfortable_stop:
false`, so it is an emergency stop, which the gate emits on `/control/command/control_cmd`.
During a drift `drift_controller` ignores that topic (it is overriding), and at hand-back it
forwards an emergency stop at 100 km/h on a car that may still be sideways. The MRM recovers
when the leaves clear (`timeout_emergency_recovery` 5 s), so a lap would survive only if every
fault clears before the drift ends. For a scenario under SSv2 the emergency is also a recorded
failure.

## What to do

Three options, from least to most work. The recommendation is to do 1 now, so laps run and
we see which alarms actually fire, and to build 3 as the project's real stack; 2 is a dead end
because it keeps all of Autoware's assumptions and adds a mode switch on top.

### 1. Keep the architecture, make Autoware tolerate a drift (days)

All in this repository, as a profile overlay passed to `just ego-isaac` (no swb change):

- control: `launch_lane_departure_checker:=false`, `launch_autonomous_emergency_braking:=false`
  (an empty ring has nothing to brake for); `control_validator` thresholds wide enough for a
  drift line (distance 5 m, yaw 1.2 rad, velocity +10 m/s), or the validator off;
- the diagnostic graph: the same graph with the control and planning performance leaves
  (`control_state`, `lane_departure`, `trajectory_validation/trajectory_shift`) as warnings
  that do not remove autonomous mode;
- `drift_controller`: hand back only when the car is within the transition manager's engage
  limits of Autoware's trajectory (1.5 m, 0.52 rad, settled sideslip); otherwise keep a
  rejoin law driving. Forward Autoware's emergency (and its gear and hazard commands) at any
  phase, so a real fault still stops the car;
- localization: measure first. Log the EKF's pose against NDT's and the gate's rejections
  through a drift (`/localization/pose_estimator/*`, the EKF's debug). If NDT holds at the
  lap's sideslip, leave it.

This keeps Autoware's lane trajectory as the line between bends, which is not a racing line,
and leaves the MPC's stale state at hand-back. It is enough to finish laps and measure.

### 2. Hand over through Autoware's operation modes

`drift_controller` would request LOCAL mode for each drift (the ADAPI
`change_to_local`) and publish as the external command source, then request AUTONOMOUS back.
Autoware then knows it is not driving, the gate's external-command limits apply, and LOCAL
mode does not depend on planning and localization accuracy. But the gate's external limits
would have to be opened to a drift, the transition back is checked against Autoware's
trajectory (the same 1.5 m and 0.52 rad), SSv2's concealer sees the mode change, and the MPC
problem is unchanged. Not recommended.

### 3. A race stack in Autoware's planning and control slots (weeks)

Autoware's launch already has the seams: `trajectory_follower_mode` other than
`trajectory_follower_node` launches no follower, so any node that publishes
`/control/trajectory_follower/control_cmd` is the controller; the gate, the operation-mode
transition manager, the MRM handler and ADAPI stay, so SSv2 and the relay work unchanged.
Planning keeps the mission planner (routes, ADAPI routing) and replaces the scenario planning
pipeline with a node that publishes `/planning/scenario_planning/trajectory`. Perception and
the rest of the system stay.

- **State estimator** (first, because both of the others depend on it): an EKF or UKF on a
  dynamic single-track model, state (x, y, yaw, vx, vy, yaw rate), fusing NDT's pose, the IMU
  (yaw rate, ax, ay) and the vehicle's speed, with front (undriven) wheel speeds where
  available. Feed NDT its initial guess from this estimator, and publish its pose and twist as
  a second kinematic state for the race nodes. Leave Autoware's EKF running for Autoware's own
  consumers, or replace it on `/localization/kinematic_state` once the new one is proven.
- **Global planner (offline)**: a racing line on the Town05 ring from the lanelet map: the
  track's boundaries, a minimum-curvature or minimum-time line (the TUM
  `global_racetrajectory_optimization` method), a speed profile from the car's g-g diagram
  (sim_vehicle_spec's tire), and per bend the drift plan: entry point, target sideslip,
  radius, exit point. This replaces `town05_ring.yaml`'s hand-placed points.
- **Local planner (online)**: follows the racing line, and after any deviation (a wide
  drift, a recovery) plans a rejoin within the track's boundaries; it publishes the trajectory
  Autoware expects plus a side topic with the drift intent (target sideslip and yaw rate per
  point), which `autoware_planning_msgs/Trajectory` has no field for.
- **Controller**: one controller for grip and drift, on the same dynamic model: below the
  limit a path tracker (Stanley or MPC on the dynamic model); in a bend the existing law,
  extended to track the path through the drift by steering on lateral and heading error and
  holding sideslip with rear drive (the approach of Goh, Goel and Gerdes's drifting along a
  general path, Stanford DDL). No hand-back between two controllers that disagree about the
  car's state.
- **Race supervisor**, in place of the road monitors: track limits (distance to the track
  boundary, not to the lane centre), spin (sideslip and yaw rate beyond the recoverable
  envelope), localization health (the new estimator's NDT residuals), stale inputs. It
  reports through Autoware's diagnostic graph so MRM still stops the car on a real fault, and
  its own abort is a straight-line brake with countersteer, not Autoware's emergency stop,
  which locks a sliding car's wheels in whatever direction it faces.

Reuse from what exists: `law.py` (the drift sequence and its laws), `model.py` (the
single-track test model, which becomes the estimator's and controller's model), the proving
ground runs (swb `spikes/manoeuvres.py`, `drift` and `power_over`) as the controller's test
bench before Town05.

## Order

1. Option 1's overlay and the hand-back rule; run laps; record which diagnostics fire and the
   EKF against NDT through each drift.
2. The state estimator, tested on recorded laps against the simulator's ground truth (aib's
   body velocity).
3. The global planner and the racing line for Town05; then the local planner.
4. The unified controller on the proving ground, then on the ring.
5. The race supervisor and its diagnostic graph; retire option 1's widened thresholds.
