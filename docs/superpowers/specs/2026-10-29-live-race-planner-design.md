# A live race planner and tracker (revises the minimum-time planner's method 5)

Date: 2026-10-29. Status: **proposal**, for the user's review. The user decided the shape
("a live planning/tracking system that runs in real time: seconds to compute a medium-range
plan, and another part to update the route and the control when the vehicle drifts"); the
rest is *ours* and open to change. It revises
[2026-10-29-min-time-planner-design.md](2026-10-29-min-time-planner-design.md) (method 5,
"tracking in Isaac") and fills in the online planner and the controller of
[race-stack-design.md](../../race-stack-design.md).

## Why

The minimum-time optimiser plans a whole track in minutes (Haruna s 60-1520: 74.8 s of driving,
about 30 min to solve), and a fixed-gain tracker then follows it. Driving that plan in Isaac
(2026-10-29, `track7`-`track16_full` in swb's `play_log/race_planner/`) showed both halves
are wrong for a car that slides:

- **The car cannot wait** for a plan that takes minutes, and a plan made once is stale as soon
  as the car is off it.
- **The tracker fights the plan.** Its path feedback steers against the rotation the plan asks
  for on a trail-braked entry, so the car does not rotate, ends 1.5 m inside the line at the
  apex, drops its inner front wheel onto the grass and loses 1.1-1.4 s per hairpin.
- **Once off the plan there is no plan.** On the full run (s 935) a braking slide made the
  tracker release the brakes; the car reached the next hairpin 6 m/s fast, the emergency brake
  locked the rear (no ABS) and it spun. A planner re-solved from where the car was would have
  braked for the hairpin from there.
- **What the plan assumed the car could do was not what the car can do**: it drove the motor
  against the brakes to move the brake balance, which the car's one acceleration request cannot
  ask for (fixed, autoware-drift 959600b). The live system must plan with the car's real inputs.

## What the field does (2026-10-29 survey; details in related-work.md)

| pattern | who | lesson for us |
|---|---|---|
| one receding-horizon problem, re-solved from the measured state every 20 ms: maximise progress along the track inside its bounds (MPCC) | Liniger, Domahidi, Morari, 1:43 cars, 50 Hz, drifting with saturated rear tyres ([arXiv 1711.07300](https://arxiv.org/pdf/1711.07300)); AMZ Driverless with a learned model error (Kabzan et al., RA-L 2019) | a car pushed off its line has no line to return to; it re-optimises from where it is |
| global line offline, a local planner every 150 ms, a tube MPC tracking with margin for model error | TUM, Roborace / Indy / A2RL ([arXiv 2205.15979](https://arxiv.org/pdf/2205.15979), [graph planner](https://www.github.com/TUMFTM/GraphBasedLocalTrajectoryPlanner)) | the layered split we want; their low layer assumes grip |
| offline time-optimal line + acados NMPC, real-time iteration, ms per solve | Li, Chen, Ren, Sensors 2024 ([PMC11174940](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11174940/)); Zanon et al. 2014 time-optimal RTI | a full nonlinear tracker is affordable at 50 Hz |
| drifting along a general path: the velocity vector's turn rate follows the path, yaw acceleration holds the sideslip | Goh, Goel, Gerdes, JDSMC 2019 ([Stanford DDL](https://ddl.stanford.edu/publications/journal/towards-automated-vehicle-control-beyond-stability-limits-drifting-along)) | exactly our tracker's conflict: path and rotation need separate handles |
| planned sideslip in the feedforward, not the feedback, for stability at the limit | Kapania, Gerdes, VSD 2015 ([Stanford DDL](https://ddl.stanford.edu/publications/journal/design-feedback-feedforward-steering-controller-accurate-path-tracking-and)) | the tracker follows the plan's sideslip as feedforward |
| linear models around a drift trajectory are enough: LQR / linear MPC tracking to centimetres | Peterson, Gerdes, thesis 2023, Supra "Takumi" ([purl vg668bz0151](https://purl.stanford.edu/vg668bz0151)) | a time-varying LQR along the plan is a cheap, strong fallback tracker |
| NMPC through transient, non-equilibrium drift and back to grip, 50 solves a second, road bounds as constraints; a learned tyre | TRI, GR Supra; tandem drift with Stanford 2024 ([TRI](https://pressroom.toyota.com/?p=72269), [paper](https://www.emergentmind.com/papers/2405.10859)) | the target capability; their state comes from an RTK INS |
| online friction estimate feeding the MPC, or friction kept uncertain in it | Dallas et al. UKF + NMPC at 95 % of friction ([arXiv 2304.12263](https://arxiv.org/pdf/2304.12263)); stochastic / risk-averse MPC ([2305.03798](https://arxiv.org/pdf/2305.03798v1.pdf), [2410.17183](https://arxiv.org/pdf/2410.17183.pdf)) | the grip share we use (0.92) becomes a measured margin |

**Drivers.** Racing coaches teach trail braking as a tool for rotation: more into slow, tight
corners, less into fast ones, released early when the car oversteers on entry
([Speed Secrets](https://speedsecrets.com/when-should-i-trail-brake-when-shouldnt-i/),
[Driver61](https://driver61.com/uni/trail-braking/)). Rally and drift drivers start a slide on
purpose with a feint (pendulum turn: away, then in), a lift, a brake stab, the handbrake or a
clutch kick, and hold it with throttle and countersteer
([Team O'Neil](https://press.teamoneil.com/blog/how-to-do-a-pendulum-turn),
[Jalopnik](https://jalopnik.com/1829051043)). Velenis, Tsiotras and Lu found trail braking and
the pendulum turn as minimum-time solutions of an optimal control problem on loose surfaces
([EJC 2008](https://dblp.dagstuhl.de/rec/journals/ejcon/VelenisTL08.html)), which is why ours
plans them rather than scripting them. A driver also never commits to one line: braking points
move with grip and entry speed, and a driver who runs wide re-plans the next corner from
where the car is. That is the behaviour wanted here.

## Goals

1. **Live**: from the estimated state, a plan of the next stretch (about 6-10 s, 150-300 m)
   re-solved about once a second, and a control command at 50 Hz, both in a fixed time budget.
   No whole-track solve on the car.
2. **Recovers by construction**: after a slide, a wide line or a lost brake, the next plan
   starts from where the car is, inside the track, and stays feasible (it can still brake for
   what is ahead).
3. **Plans only what the car can do**: the car's real inputs (steering, one acceleration
   request that the ECU turns into drive or a brake split; never both), its limits, the
   track's bounds with a margin.
4. **Drift where it is faster, not by rule** (unchanged): sideslip free in the planner; grip
   and drift differ only in the plan.
5. **Proof** (Isaac, Haruna then Tsukuba, no per-track parameters): the full Haruna run
   completes without contact within 5 % of the offline optimum's time (74.8 s), and finishes
   after a disturbance (a kick in yaw rate, a patch of low grip) where the current tracker
   spins.

## Non-goals

- The C++ port and Autoware's seams (race-stack-design.md's milestone 4): the layers are built
  and proven in Python against Isaac first; the interfaces are chosen so the port is
  mechanical.
- Learned models (Kabzan, TRI's neural tyre), traffic, D1 scoring, AutoSDV hardware.

## Anti-goals

- Any parameter that names a track, a corner or a drift point (unchanged).
- A tracker that pulls the car back onto a stale line through a wall, or onto the grass.
- A layer that silently assumes another succeeded: each must have a defined behaviour when the
  one above is late or failed.

## The design (*ours*)

Three layers, each re-planning from the current state, slower and longer above, faster and
shorter below. The ECU (sim_vehicle_ecu) stays below them as the car's own.

```
 track + car spec ------------------------------------------------------------.
   | offline (minutes, once per track and surface; optional)                   |
   v                                                                           |
 L0  global plan: the minimum-time OCP we have (ocp.py)                        |
     -> a speed ceiling v_max(s) and a line hint n(s) for the windows' ends    |
   |                                                                           |
   v              state (estimator: x, y, yaw, vx, vy, r, wheel speeds, mu)   |
 L1  medium-range planner, ~1 Hz, budget <= 1 s: the same two-track model,    |
     min time over the next 150-300 m from the current state, warm-started    |
     from the last plan shifted; terminal speed <= v_max(s_end) (or able to  |
     stop in the remaining width); event-triggered re-solve after a slide    |
   |   plan: x, y, s, vx, vy, r, beta, steer, accel request per 0.1 s        |
   v                                                                          |
 L2  tracker, 50 Hz, budget <= 10 ms: NMPC on the dynamic single- or two-    |
     track model, horizon 1-1.5 s, inputs = the car's (steer rate, accel     |
     request -> ECU split); cost: contouring + lag error to the L1 plan,     |
     its sideslip and yaw rate (Kapania: sideslip in the reference, not as   |
     path feedback), track bounds soft; acados RTI                           |
     fallback: time-varying LQR along the L1 plan (Peterson)                 |
   |   steer, accel request                                                  |
   v                                                                          |
 ECU (sim_vehicle_ecu): drive / brake split, locked axle, steering actuator <-'
```

### L0: the global plan (offline, optional)

The existing `race_planner.ocp.solve_min_time` over the whole track. Its job changes: not a
rail to follow but the long view a 300 m window lacks: a ceiling `v_max(s)` (with the grip share)
that L1's window must end under, so a window never ends at a speed the next hairpin cannot
take, and a line hint for warm starts. Without it (a new track, no time to solve), L1 ends each
window at a speed from the quasi-steady-state profile (`qss.py`, seconds), which is
conservative but safe.

### L1: the medium-range planner

- **Problem**: today's spatial OCP (two-track, PhysX tyre, load transfer, the brake system,
  drive xor brake, slip bounds), over a window `[s0, s0 + W]` with W about 150-300 m (more than
  the braking distance from top speed: 37 m/s at 8 m/s^2 is 85 m), started at the estimated
  state (n, xi, vx, vy, r, delta, wheel speeds), ended at `v <= v_max(s0 + W)`.
- **Inputs it plans in**: the ones the car has. Today's OCP plans four wheel torques under
  constraints that make them the ECU's split. L1 should plan steer rate and one acceleration
  request, the ECU's map to torques written into the model (sim_vehicle_ecu's own functions), so
  "never drive against the brakes" is structural, not a penalty.
- **Speed**: warm start from the previous plan shifted by the distance driven (the solution
  barely changes between solves), a coarser grid (ds 2-4 m), a fixed iteration cap; a solve that
  hits the cap still returns its best feasible iterate. **This is the main risk**: today a cold
  400 m solve takes 200 s; whether a warm 200 m window converges in under a second has to be
  measured first (milestone 1). If not, the window shrinks, the model drops to a single track
  for L1 (keeping load transfer), or the solver changes (acados SQP with a horizon in s).
- **Re-solve**: every 0.5-1 s on a timer, and at once when the car leaves the plan's envelope
  (lateral error > 0.5 m, sideslip off by > 5 deg, speed off by > 2 m/s: *ours*, to tune once on
  the proving ground, not per track).
- **Out**: the plan resampled in time (0.1 s), with the state and inputs per point, and the
  solve's status and age.

### L2: the tracker

- **Problem**: NMPC (acados, real-time iteration) on the dynamic single-track model with the
  PhysX tyre and load transfer (two-track if the budget allows), horizon 1-1.5 s at 50 Hz;
  inputs steer rate and acceleration request, with the ECU's split in the model; cost:
  contouring and lag error to L1's plan, its speed, sideslip and yaw rate as references (grip
  and drift differ only in these), input rates; track bounds as soft constraints.
- **Why this fixes today's failures**: the tracker predicts what its steering and braking will
  do over 1 s, so it does not steer against a planned rotation (the plan's sideslip is a
  reference, not an error to remove) and does not release the brakes to stop a slide when the
  hairpin needs them (the bounds and the speed ahead are in its cost).
- **Fallback**: a time-varying LQR along L1's plan (linearised at each plan point; Peterson's
  finding that linear models suffice near a drift trajectory), used if the NMPC misses its
  budget or fails. The existing course_probe tracker stays only as a comparison.

### When something goes wrong

| event | behaviour |
|---|---|
| L1 late or failed | L2 keeps tracking the last plan; past its end, L2 tracks a stop inside the bounds |
| the car leaves the plan's envelope | L1 re-solves now, from the measured state |
| L2 misses its budget / fails | the LQR fallback for that tick; three in a row: a stop along the plan |
| a spin (sideslip and yaw rate outside the recoverable set) | brake to a stop, steering into the slide; reported as the race controller's diagnostic (race-stack-design.md's supervision) |

### State and grip

Stage 0 is the simulator's state (aib's body velocity, as today). The grip share (0.92 today)
stays as a margin until an online friction estimate (Dallas et al.'s UKF on the same model)
gives L1 and L2 the road's mu and its uncertainty; that is part of the race estimator, not of
this design.

## Where and how it runs

- `race_planner` grows `window.py` (L1: a window of the OCP from a state) and `tracker/` (L2:
  the acados model and solver, generated from the same CasADi model code; the LQR fallback).
- Proven in Isaac through swb's `course_probe` (it already runs a PLAN): a mode that runs L1
  and L2 in the probe's loop, L1 on its own thread or process with its time budget enforced, so
  the timing is real; its frames logged as today (`plan_track.jsonl` with the wheels).
- Later (race-stack-design.md): L1 and L2 as ROS 2 nodes, L2 in C++ linking acados' generated
  C.

## Milestones

1. **L1 speed spike** (throwaway): a warm-started 200 m window of today's OCP on Haruna,
   stepped along the track; measure solve times. Decides L1's window, grid and model.
2. **L1 in the loop** with today's tracker in Isaac: re-planning every second from the state;
   the full Haruna run completes (today's s 935 failure) within the stated margin.
3. **L2 NMPC** (acados) offline against the model, then in Isaac in place of today's tracker;
   the hairpins within 5 % of the plan.
4. **Disturbances**: a yaw kick and a low-grip patch on Haruna; the run finishes.
5. **Tsukuba** with no changes.

## Success

- Full Haruna run, no contact, within 5 % of L0's 74.8 s; L1's solves within budget over the run
  (p99), L2's within 10 ms.
- The run finishes after the disturbances of milestone 4.
- Tsukuba runs with the same code and settings.

## Calls made after the builder check (*ours*, open to change)

1. **Solver and the ECU in the model.** acados is not installed (2026-10-29); milestone 3
   builds it (BSD-2, CMake, its Python `acados_template` in race_planner's venv). The ECU's map
   from one acceleration request to torques is piecewise linear for drift_mustang (no ABS, no
   TC, a static 50:50 brake split, drive on the rear, the locked axle): drive =
   max(a, 0) and brake = max(-a, 0), written with a smooth max (softplus, as model.py's
   knee) so L1 and L2 share one CasADi model. Cars with ABS / TC on: those stay outside the
   model, as ECU behaviour the margin covers.
2. **L1's bar, if the spike misses 1 s.** Live means the plan arrives while the car is still
   inside its window's first quarter: solve time x top speed < W / 4 (W 200 m at 37 m/s: under
   1.3 s). Up to 3 s is acceptable with W grown to keep that ratio. L1 need not be L0's
   formulation: a time-domain MPCC or a coarser single-track OCP are allowed if they keep
   free sideslip and the car's inputs.
3. **Hand-over.** L1 solves from the state predicted (by the model, under the current plan) to
   solve start + its budget, and stamps the plan with that time; L2 switches to the new plan at
   its first tick after that time, with no blending: L2's cost refers to the plan by s
   (contouring), so a switch moves the reference, not the steering, and its input-rate cost
   smooths the rest.
4. **Sensing and time.** Stage 0: the simulator's state, no noise, no latency (the estimator is
   a separate milestone). course_probe steps Isaac synchronously in simulated time; L1 runs in
   its own process, and its plan becomes available at solve start + its measured wall-clock
   time, mapped to simulated time, so the latency the car would feel is real even though the
   sim waits. L2 runs inline at 50 Hz (sim time); the ECU stays at physics rate, as now.
5. **Bounds and contact.** The lateral limits are the course's road edges (centreline +/-
   half width, 4 m on Haruna) less the car's half width and 0.3 m (L1 hard, L2 soft). Failure:
   any guardrail contact, or any wheel's raycast on Terrain (off the asphalt) for more than
   0.2 s (course_probe logs both: `contacts`, the wheels' ground shape).
6. **Window end and L0.** v_max(s) is L0's speed times 1.0 (L0 already has the grip share);
   without L0, `qss.py`'s profile. The proof run (success 1) uses L0, as precomputed per track
   is not per-track tuning; the same run without L0 is reported alongside.
7. **Disturbances** (milestone 4; course_probe gains an injection hook): a yaw-rate kick of
   30 deg/s applied over 0.1 s mid-straight before a hairpin; a 20 m patch at mu 0.6
   (swb's surface layer, a class override by s) at a hairpin's entry.
8. **Compute.** 32 cores, one RTX 5090 (Isaac). L2 calls acados' generated C through its
   Python wrapper (the solve is C; the loop's Python overhead is well under the 10 ms budget);
   L1 in its own process (no GIL contention).

## Left to the builder

The window length and grid, L1's model fidelity and solver after the spike; the envelope
thresholds; acados options; the LQR's weights; logging. Each is set once on the proving ground
or the first track and not per corner.
