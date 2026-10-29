# A live race planner and tracker (revises the minimum-time planner's method 5)

Date: 2026-10-29. Status: **proposal, revision 2**, for the user's review. The user decided the
shape ("a live planning/tracking system that runs in real time: seconds to compute a
medium-range plan, and another part to update the route and the control when the vehicle
drifts"); everything else is *ours*, argued below, and open to change. It revises
[2026-10-29-min-time-planner-design.md](2026-10-29-min-time-planner-design.md) (method 5,
"tracking in Isaac") and fills in the online planner and the controller of
[race-stack-design.md](../../race-stack-design.md). Revision 2 adds the reasoning behind each
decision and the changes from a design review (listed at the end).

## 1. Why change

The minimum-time optimiser plans a whole track in one solve (Haruna s 60-1520: 74.8 s of
driving, about 30 min to solve), and a fixed-gain tracker follows it. Driving that plan in
Isaac (2026-10-29, swb `play_log/race_planner/track7`-`track16_full`) gave four lessons. Each
one is a requirement below.

| what happened | lesson | requirement |
|---|---|---|
| the car cannot wait 30 min, and a plan made once is stale as soon as the car leaves it | planning must run while driving | **R1** plans arrive within a fixed budget, from the current state |
| at s 935 a braking slide made the tracker release the brakes; with no plan from where the car was, it reached the hairpin 6 m/s fast, locked the rear and spun | after a deviation the car needs a new plan *from where it is*, not a pull back to the old one | **R2** recovery is re-planning, not path correction |
| at every hairpin the tracker's path feedback steered against the rotation the plan asked for; the car ended 1.5 m inside the line and dropped a wheel on the grass (1.1-1.4 s lost each) | a tracker that only sees errors against a line cannot hold a planned slide | **R3** the tracker predicts and follows the plan's sideslip and yaw rate, not just its line |
| the plan drove the motor against the brakes to move the brake balance, which the car's one acceleration request cannot ask for (fixed in 959600b) | a plan in inputs the car does not have is not a plan | **R4** every layer plans in the car's real inputs and limits |

## 2. What the field does, and what we take from it

| pattern | who | what we take |
|---|---|---|
| one receding-horizon problem re-solved from the measured state every 20 ms: maximise progress along the track inside its bounds (MPCC) | Liniger, Domahidi, Morari, 1:43 cars, 50 Hz, drifting with saturated rear tyres ([arXiv 1711.07300](https://arxiv.org/pdf/1711.07300)); AMZ Driverless with a learned model error (Kabzan et al., RA-L 2019) | R2: no line to return to; the tracker's cost is progress along the track (contouring), not distance to a line |
| global line offline; a local planner every 150 ms; a tube MPC tracking it, its constraints tightened by a margin for model error | TUM, Roborace / Indy / A2RL ([arXiv 2205.15979](https://arxiv.org/pdf/2205.15979), [graph planner](https://www.github.com/TUMFTM/GraphBasedLocalTrajectoryPlanner), [tube MPC](https://portal.fis.tum.de/en/publications/a-tube-mpc-approach-to-autonomous-multi-vehicle-racing-on-high-sp/)) | the layered split; the planner plans with less grip than the tracker may use |
| offline time-optimal line + acados NMPC with real-time iteration, a few ms per solve | Li, Chen, Ren, Sensors 2024 ([PMC11174940](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC11174940/)); Zanon et al. 2014; acados RTI racing at 3.4 ms ([arXiv 2604.19452](https://arxiv.org/pdf/2604.19452)) | a nonlinear tracker is affordable at 50 Hz |
| drifting along a general path: the velocity vector's turn rate follows the path, yaw acceleration holds the sideslip | Goh, Goel, Gerdes, JDSMC 2019 ([Stanford DDL](https://ddl.stanford.edu/publications/journal/towards-automated-vehicle-control-beyond-stability-limits-drifting-along)) | R3: path and rotation need separate handles |
| the planned sideslip in the feedforward, not as path feedback, for stability at the limit | Kapania, Gerdes, VSD 2015 ([Stanford DDL](https://ddl.stanford.edu/publications/journal/design-feedback-feedforward-steering-controller-accurate-path-tracking-and)) | R3: the plan's sideslip is a reference, never an error to remove |
| linear models about a drift trajectory suffice: LQR / linear MPC to centimetres | Peterson, Gerdes, thesis 2023, Supra "Takumi" ([purl vg668bz0151](https://purl.stanford.edu/vg668bz0151)) | a time-varying LQR along the plan as the fallback tracker |
| NMPC through transient drift and back to grip, 50 solves a second, road bounds as constraints | TRI, GR Supra; tandem drift with Stanford 2024 ([TRI](https://pressroom.toyota.com/?p=72269), [paper](https://www.emergentmind.com/papers/2405.10859)) | the target capability |
| friction estimated online and fed to the MPC, or kept uncertain inside it | Dallas et al., UKF + NMPC at 95 % of friction ([arXiv 2304.12263](https://arxiv.org/pdf/2304.12263)); stochastic / risk-averse MPC ([2305.03798](https://arxiv.org/pdf/2305.03798v1.pdf), [2410.17183](https://arxiv.org/pdf/2410.17183.pdf)) | the grip margins become measured, later |
| trail braking and the pendulum turn come out of a minimum-time optimal control problem | Velenis, Tsiotras, Lu, EJC 2008 ([dblp](https://dblp.dagstuhl.de/rec/journals/ejcon/VelenisTL08.html)) | drift techniques are planned, never scripted |

**Drivers.** Coaches teach trail braking as a tool for rotation: more into slow, tight
corners, less into fast ones, released early when the car oversteers on entry
([Speed Secrets](https://speedsecrets.com/when-should-i-trail-brake-when-shouldnt-i/),
[Driver61](https://driver61.com/uni/trail-braking/)). Rally and drift drivers start a slide on
purpose (a feint away then in, a lift, a brake stab, the handbrake, a clutch kick) and hold it
with throttle and countersteer ([Team O'Neil](https://press.teamoneil.com/blog/how-to-do-a-pendulum-turn),
[Jalopnik](https://jalopnik.com/1829051043)). Three habits matter here. A driver commits to a
corner's plan (brake point, entry, rotation) about one corner ahead, not one lap ahead. A
driver who runs wide re-plans the next corner from where the car is. And a driver never drives
the whole lap at the grip limit everywhere: the margin is larger where the cost of a mistake is
a wall than where it is a run-off. The design copies all three.

## 3. Goals, non-goals, anti-goals

**Goals.**
1. R1-R4 above, measured: plans within budget (p99), control at 50 Hz within 10 ms.
2. Drift where it is faster, not by rule (unchanged): sideslip is free in the planner.
3. Proof in Isaac, Haruna then Tsukuba, no per-track parameters: the full Haruna run completes
   without a failure (section 7) within 5 % of the offline optimum's time (74.8 s), and still
   completes after a disturbance (a yaw kick, a low-grip patch) where today's tracker spins.

**Non-goals.** The C++ port and Autoware's seams (race-stack-design.md, milestone 4): built
and proven in Python first, interfaces chosen so the port is mechanical. Learned models,
traffic, D1 scoring, AutoSDV hardware. The race estimator (separate; stage 0 is the
simulator's state).

**Anti-goals.** Any parameter that names a track, a corner or a drift point. A tracker that
pulls the car back onto a stale line through a wall or onto the grass. A layer that silently
assumes another succeeded.

## 4. The design: why layers, and why these

### 4.1 The constraint that forces layers

Two needs pull in opposite directions.

- **Seeing far enough.** A corner's decisions (where to brake, the line, whether to slide) depend
  on the whole corner and what follows: braking from 37 m/s at 8 m/s^2 takes 85 m, and a
  hairpin's entry, apex and exit span another 50-100 m. The car must look **150-300 m (5-10 s)**
  ahead to decide the next corner well.
- **Reacting fast enough.** A car sliding on saturated rear tyres is unstable: its sideslip
  diverges in a few tenths of a second (before the s 935 spin the sideslip doubled, 6.5 to
  13.5 deg, in 0.6 s under braking, and the car spun within 0.2 s of the last frame it held). Feedback
  must run at **20-50 Hz** with a model that predicts at least that far, about **1 s**.

One optimisation cannot do both on our model. A problem that sees 300 m ahead with the
two-track model and its sliding tyres is a large non-convex problem: it needs many iterations
to converge (today: 200 s cold for 400 m), and the converged answer matters, because a
half-solved one can pick the wrong corner plan. A problem solved 50 times a second can afford
about one iteration (real-time iteration), which is excellent at correcting around a good
reference and poor at discovering a new one. So the work is split by what each needs: **a slow
layer that converges over a long horizon, and a fast layer that iterates once over a short
one.** That is the medium-range planner and the tracker. This matches the hierarchical
racing stacks (TUM; Liniger's hierarchical variant; Sensors 2024) and how drivers plan a corner
ahead while correcting every instant.

Why not one fast layer with a long horizon (Liniger's single MPCC)? It works for 1:43 cars at
3 m/s, where 40 steps cover the next corner. At 37 m/s, 300 m is 8 s: 400 steps at 20 ms, two
orders of magnitude more than one RTI tick can carry on a two-track model with sliding tyres.
We keep its idea (contouring cost, re-solve from the measured state) in the tracker.

Why not a sampled or graph local planner (TUM)? Fast and robust for grip, but a drift's
states (sideslip, yaw rate, rear slip) are not points on a lattice of lateral offsets; sampled
paths would need the dynamics afterwards anyway. The optimiser plans them directly.

### 4.2 The layers

```
 L0  offline, once per track and surface (minutes; optional)
     whole-track minimum-time OCP (today's ocp.py)
     -> v_max(s): the long view a window lacks          (why: 4.3)
            |
            v           state (stage 0: simulator; later the race estimator)
 L1  medium-range planner, ~1 Hz, budget <= 1 s
     min time over the next 150-300 m from the predicted state, the car's inputs,
     grip share g1 < 1; several warm starts; the window's end bounded by v_max
     -> a plan: state and inputs per 0.1 s, stamped    (why: 4.4)
            |
            v
 L2  tracker, 50 Hz, budget <= 10 ms
     NMPC (acados RTI), horizon ~1 s: contouring + lag to L1's plan, its speed, sideslip
     and yaw rate as references; the car's inputs; grip up to 1; track bounds soft
     fallback: time-varying LQR along L1's plan         (why: 4.5)
            |  steer, one acceleration request
            v
 ECU (sim_vehicle_ecu, the car's own): drive or brake split, locked axle, steering actuator
```

### 4.3 L0: the long view (offline, optional)

**Why it exists.** A 200 m window cannot see past its end, so it could end at a speed the next
hairpin cannot take; the car would arrive too fast and the next window could not save it. The
window's end needs a speed it must not exceed. The whole-track optimum supplies exactly that,
`v_max(s)`, and it is computed once per track and surface, offline: preparation, not tuning,
like the map.

**Why optional.** A new track, or a surface the car has not seen, should still drive.
Without L0, the window ends under `qss.py`'s speed profile (the car's g-g envelope along the
track, seconds to compute), which is conservative.

**Revision: the end condition must stay feasible.** If the car is far off L0's line at the
window's end, L0's speed there may not apply. The end speed is bounded by the lower of L0's
`v_max(s)` and the quasi-steady speed the car could still brake to from that lateral offset;
a window whose end is infeasible is shortened.

### 4.4 L1: the medium-range planner

**Why it exists.** R1 and R2: it makes the corner-scale decisions (brake point, line, rotation
and slide) from where the car actually is, again every second and immediately after the car
leaves the plan. It is the layer that would have re-planned the s 935 approach at the car's
real speed and braked from there.

**Why ~1 Hz and 150-300 m.** At 37 m/s the car covers 37 m between plans, a sixth of a 200 m
window, so each plan is used for its first part and replaced while the corner is still ahead;
one second leaves room for a solve that converges. "Live" is defined as: the plan arrives
while the car is still in its window's first quarter (solve time x speed < W / 4); with W 200 m
at 37 m/s that is 1.3 s, and up to 3 s is acceptable with W grown to keep the ratio.

**What it solves.** Today's spatial OCP (two-track, PhysX tyre, load transfer, locked axle,
slip bounds) over `[s0, s0 + W]` from the predicted state, ending under the bound of 4.3, in the
car's inputs (R4): steer rate and one acceleration request, with the ECU's map to torques in
the model (for drift_mustang piecewise linear: drive = max(a, 0) on the rear, brake =
max(-a, 0) split 50:50; written with a smooth max, as model.py's knee), so driving against the
brakes is impossible by construction rather than penalised.

**Revision: grip margin by layer.** L1 plans with a grip share g1 < 1 (0.92 today); L2 may
use the full grip. The difference is the tracker's authority to correct: a plan at the
limit leaves the tracker nothing, as the runs showed (the car slid and every correction made it
worse). This is TUM's constraint tightening in its simplest form. g1 is one number for the car,
later replaced by a margin from the friction estimate's uncertainty.

**Revision: margin by edge type, not by corner.** Leaving the road is not equally costly
everywhere: a guardrail ends the run, grass 8 cm below the asphalt locks a wheel, a run-off
costs time. The track margin comes from the map's edge type (guardrail, terrain, run-off),
which the course generator already knows, not from any corner's name. Today's 0.3 m everywhere
put the inner wheels on the grass at Haruna's apexes.

**Revision: several warm starts, because a slide is a choice.** Grip and drift through a
corner are different local optima; a solver warm-started from the last plan stays in the
mode it started in and never discovers the other. L1 solves from two or three starts in
parallel (the last plan shifted; the grip line from L0 or `qss.py`; a drift seed from the
steady drift states `steady.py` computes) and keeps the fastest feasible result. This is what
"drift only where it is faster" needs online; the cores are there (32).

**How it meets its budget.** Warm start from the last plan shifted by the distance driven; a
coarser grid (ds 2-4 m); an iteration cap that returns the best feasible iterate. **This is the
main risk**: a cold 400 m solve takes 200 s today; whether a warm 200 m window converges in
about a second must be measured first (milestone 1). Fallbacks, in order: a shorter window, a
single-track model with load transfer, acados' SQP in the spatial domain.

**When it re-solves.** On a 1 s timer, and at once when the car leaves the plan's envelope
(lateral error > 0.5 m, sideslip off by > 5 deg, speed off by > 2 m/s: set once on the proving
ground, not per track).

**Hand-over.** It solves from the state predicted (by the model, under the current plan) to
solve start + budget, and stamps the plan with that time; L2 switches at its first tick after it,
without blending. L2 follows the plan by s (contouring), so a switch moves the reference, not
the steering, and its input-rate cost smooths the rest.

### 4.5 L2: the tracker

**Why it exists.** R3: between plans, the car is pushed off by model error, bumps and its own
instability in a slide, faster than a plan can come. Something must correct at 50 Hz, and in a
slide it must not undo the slide the plan asked for.

**Why an NMPC, not today's gains or an LQR.** Today's tracker reacts to errors against a line;
it cannot know that steering out of a planned rotation makes things worse a second later, or
that releasing the brakes now means arriving too fast. An NMPC predicts 1 s ahead with the same
car model: the plan's sideslip and yaw rate are references in its cost (Kapania), the track
bounds and input limits are constraints, and the progress along the track is part of the cost
(Liniger), so it trades a little line for a lot of stability. An LQR is linear and has no
constraints; it is the fallback, valid near the plan (Peterson), cheap enough never to miss a
tick.

**Why acados with real-time iteration.** One SQP iteration per tick on a warm start is what
50 Hz allows, and what the literature runs in a few ms; acados generates C from the same CasADi
model code L1 uses, and the C is what the later C++ node links.

**Revision: the spatial formulation does not survive a spin.** L1 plans in s (distance along
the track), which divides by the car's progress rate; in a spin, progress stops and the
formulation breaks. L2 runs in time and keeps working; if the car leaves the recoverable
envelope (section 5), L1 is not asked for a plan until the car is moving along the track again.

## 5. When something goes wrong

| event | behaviour | why |
|---|---|---|
| L1 late or failed | L2 keeps tracking the last plan; past its end, a stop inside the bounds | the last plan is the best information there is |
| the car leaves the plan's envelope | L1 re-solves now, from the measured state | R2 |
| L2 misses its budget or fails | LQR for that tick; three in a row: a stop along the plan | the LQR never misses |
| a spin (sideslip and yaw rate outside the recoverable set) | brake to a stop, steering into the slide; reported as the race controller's diagnostic | the spatial planner cannot plan it (4.5) |

## 6. How it is built and tested

**Revision: a model-in-the-loop simulator first.** Each Isaac run takes about five minutes;
tuning a live system there is slow. race_planner's own two-track model, integrated in time,
is a plant that runs faster than real time; the layers are developed against it, with a
deliberately wrong plant (grip and load transfer off by 10 %) to test robustness, and then
taken to Isaac. Isaac remains the judge.

- `race_planner` grows `window.py` (L1), `tracker/` (L2: acados model and solver from the
  same CasADi code; the LQR fallback) and `sim.py` (the model-in-the-loop plant).
- In Isaac through swb's `course_probe`, which already runs a PLAN: a mode that runs L1 and L2
  in its loop. Isaac steps in simulated time; L1 runs in its own process and its plan becomes
  available at solve start + its measured wall-clock time, so the car feels the real latency.
  L2 runs inline at 50 Hz, calling acados' generated C. Stage 0 state is the simulator's.
- acados is not installed yet (BSD-2, CMake; its Python `acados_template` in race_planner's
  venv).

## 7. Milestones and success

1. **L1 speed spike** (throwaway): a warm-started 200 m window of today's OCP stepped along
   Haruna; solve times, with one and with three warm starts. Decides W, the grid and the model.
2. **Model-in-the-loop**: L1 re-planning against `sim.py` with today's tracker; then L2 (acados)
   in its place; with the wrong-plant variant.
3. **Isaac, Haruna full run** with L0, L1, L2. Failure: any guardrail contact, or any wheel off
   the asphalt for more than 0.2 s (course_probe logs both).
4. **Disturbances** (course_probe gains an injection hook): a yaw-rate kick of 30 deg/s over
   0.1 s before a hairpin; a 20 m patch at mu 0.6 at a hairpin's entry. The run completes.
5. **Tsukuba**, same code and settings.

Success: milestone 3 within 5 % of 74.8 s, L1 within budget (p99) and L2 within 10 ms over the
run; milestones 4 and 5 complete. The same run without L0 is reported alongside.

## 8. Left to the builder

W and the grid after the spike; L1's model fidelity and solver; the envelope thresholds; g1;
the margin per edge type; acados options; the LQR's weights; logging. Each is set once on the
proving ground or the first track, and never per corner.

## 9. Revision 2: what changed from the first proposal, and why

| change | why |
|---|---|
| reasoning for each layer (4.1, 4.3-4.5) and the requirements R1-R4 traced to the Isaac runs | the first version stated the layers without arguing them |
| grip margin by layer: L1 plans at g1 < 1, L2 may use full grip | a plan at the limit leaves the tracker no authority; seen in every slide |
| several warm starts in L1 | grip vs drift is a choice between local optima; one warm start never changes mode |
| window end bounded by what the car can still reach from its actual offset | L0's speed at s is for L0's line, not for wherever the car is |
| track margin by edge type from the map | a wall, a grass drop and a run-off cost different things; today's uniform 0.3 m dropped wheels on the grass |
| the spatial planner stands down in a spin | its formulation divides by progress along the track |
| model-in-the-loop simulator before Isaac | five minutes per Isaac run is too slow to develop a live system in |
| why not one long-horizon MPCC, why not a sampled planner | alternatives argued, not just dropped |
