# A track-agnostic minimum-time planner (the race stack's planner, first)

Date: 2026-10-29. Status: decided by the user in conversation ("Go"); the calls marked *ours*
are the builder's, open to change.

## Intent

The drift on Haruna's H1 (2026-10-29) came from settings tuned by hand for that hairpin. The
user wants no per-route tuning: a method that takes any race track -- a Tsukuba-style circuit,
another touge pass -- and the car, and works out how to drive it. Driving happens mostly in
simulation (Isaac, swb), with the physics modelled realistically alongside; the controller may
one day run on a small real car, AutoSDV.

The target is **lap time** (user, after weighing D1-style scoring). Drifting is not asked for;
it should appear where, and only where, the physics makes it faster: low friction, very tight
hairpins, or a car whose layout favours rotating on the rear (drift_mustang's locked axle). On
dry asphalt with an open differential the plan should stay near grip.

## Goals

1. From a track's map and attributes plus a car's spec, with no per-track parameters, compute
   the minimum-time trajectory: the line, the speed, and the car's full state along it --
   longitudinal and lateral velocity (so sideslip), yaw rate, steering, drive and brake.
   Braking points and turn-in points are read off it (where the deceleration and the steering
   build), not set.
2. Physics the plan can be trusted on: the model is the car's spec (sim_vehicle_spec) and the
   track's surface friction and grade; it is checked against Isaac's car on manoeuvres before
   it plans a lap.
3. Proof that it is track- and car-agnostic: Haruna's downhill section and a second track of a
   different kind (Tsukuba circuit, from open data with the existing course generator) with the
   same code and no tuning; and the expected sideslip pattern across conditions on Haruna --
   locked rear axle on asphalt, open differential on asphalt, and snow.
4. The planned trajectory driven in Isaac by a tracking controller, its lap time compared with
   the plan's.

## Non-goals (this design)

- D1-style scoring (line through zones, angle, speed): a different objective on the same
  machinery, later.
- The online re-planner, the race estimator, Autoware integration of the new controller, and
  AutoSDV hardware: the race stack's later milestones (race-stack-design.md).
- Traffic, overtaking, tyre wear, fuel/energy.

## Anti-goals

- Any parameter that names a track, a corner or a drift point.
- A drift forced by a rule ("hairpin under R x: drift"). Sideslip is a free variable of the
  optimisation.
- A plan from a model that has not been compared with the simulator's car.

## Inputs

- **Track**: the Lanelet2 map (the road's boundaries: centreline, left and right widths; a
  closed circuit or a point-to-point section), the map's z for grade and banking, and the
  surface class per area (swb's `surfaces.yaml`: class -> mu from `surface_classes.yaml`).
  A track attribute file may add what a map lacks (a safety margin to walls or guardrails,
  the direction of travel, start and finish), never a per-corner setting.
- **Car**: sim_vehicle_spec's `vehicles/<car>.yaml`: mass, CoM, inertia, wheelbase, track,
  tyre (friction, stiffness, friction-vs-slip), powertrain (motor torque and power, gear,
  drive axle, differential), brakes, steering lock, drag. AutoSDV later gets its own spec.

## Method (*ours*, from the literature)

1. **Track model**: the reference line and its curvature, the left/right free width less the
   car's half width and a margin, grade and mu along s; curvilinear (Frenet) coordinates.
2. **Quasi-steady-state baseline**: a minimum-curvature line and a forward-backward speed
   profile from the car's g-g-v envelope (the tyre's mu, drive and brake limits, drag,
   grade) -- fast, robust, and the warm start for 3.
3. **Minimum-time optimal control** (the plan): direct collocation over s on a dynamic
   two-track model -- states vx, vy, r, lateral offset n, heading error, front steer, wheel
   speeds of the rear axle (so a locked axle and wheelspin are modelled), per-wheel load
   transfer; combined-slip tyres fitted to the spec's PhysX tyre (and to Isaac measurements, 4);
   controls steering rate, drive and brake torque -- minimising time, with track width, steering,
   torque, power and brake limits as constraints. Sideslip is free. CasADi + IPOPT (Python,
   offline). References: Tavernini et al. 2013 (minimum-time cornering, surface and layout),
   Velenis et al. (drift equilibria), TUM global_racetrajectory_optimization (run as a tool or
   re-implemented; its output is ours).
4. **Model check against Isaac**: steady-state circles (grip and drift), step steer and a
   braking test with swb's manoeuvre spikes on drift_mustang; the model's tyre and drivetrain
   fitted until it predicts Isaac's car (yaw rate, sideslip, speed) within stated tolerances.
5. **Tracking in Isaac**: the existing course probe drives the planned trajectory (state
   feedback on the plan's line, speed, sideslip and yaw rate); the race stack's NMPC
   controller replaces it later.

## Outputs

A trajectory file per (track, car, surface): s, x, y, heading, vx, vy, sideslip, yaw rate,
steering, drive/brake, lap time; plus derived braking and turn-in points and plots against the
map.

## Success (*ours*)

- Same code, no per-track parameters, on Haruna and Tsukuba.
- On Haruna: locked axle on asphalt -- small sideslip (a few to ~15 deg) in the tight hairpins
  if faster; open differential on asphalt -- near-grip; snow -- large sideslip broadly. If the
  optimiser says otherwise, the result stands and is explained, not tuned away.
- Tracked in Isaac within ~5 % of the planned time on a section, with no contact.

## Where

autoware-drift (the race stack's home): a `race_planner` Python package (offline tool) and its
tests; tracks generated by swb's course generator (`swb_isaac_world.course`).
