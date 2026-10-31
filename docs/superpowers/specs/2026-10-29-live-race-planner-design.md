# A live race planner and tracker (revises the minimum-time planner's method 5)

Date: 2026-10-29. Status: **revision 3** (2026-10-30): built and driven in roadmap 060;
revision 3 (section 4.4a, listed in section 10) replaces "several warm starts" with starts
chosen from what lies ahead, from roadmap 061's measurements. Revision 4 (2026-10-31,
section 11) adds a driving-style layer above the planner (fastest lap, drift when cheap,
drift everywhere), designed in
[2026-10-31-drift-style-layer-design.md](2026-10-31-drift-style-layer-design.md). The user decided the
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

## 1a. System requirements (user, 2026-10-29)

- **SR1, speed: at least 110 km/h (30.6 m/s), and as fast as the car and the road allow.**
  110 km/h is Taiwan's highway limit and the floor of the envelope the system must handle; it
  is not a cap. There is no speed limit in the plan: on Haruna's straights the optimum reaches
  134 km/h (37.2 m/s), which is fine.
- **SR2, the fastest lap**: minimum time is the objective; the car's power, grip and the bends
  set the speed. On a mountain road that is mostly well under 100 km/h; on straights higher.

- **SR3, the car's own capability is respected** (user, 2026-10-29): its top speed, its
  acceleration and its deceleration, and its steering, as the spec and the ECU make them. No
  layer plans or commands what the car cannot do (R4 generalised). Each limit comes from the
  spec (sim_vehicle_spec `vehicles/<car>.yaml`), never from a planner constant; where the spec
  declares an explicit limit (a speed limiter, a request range of the vehicle interface), it
  wins over the derived one.

  | limit | from the spec (drift_mustang) | value | modelled today (race_planner) |
  |---|---|---|---|
  | top speed | motor_max_rpm 16000 through gear 6.0 on R 0.32 m; and power 330 kW against drag (0.5 rho Cd A = 0.51) and rolling | 89 m/s by the motor, about 86 m/s by power and drag: the lower one, about 310 km/h | power and drag yes; the motor's rpm limit **no** (add as `|w_rear| <= rpm_max / gear`) |
  | acceleration | motor peak torque 700 N m x 6.0 to base speed, then 330 kW; rear grip | 13 kN at the axle (9 m/s^2), grip-limited lower (about 6 m/s^2 on dry asphalt) | yes (torque, power, tyre) |
  | deceleration | max_brake_torque 2500 N m per wheel, static 50:50; regen 150 N m / 60 kW at the motor; grip | 31 kN of brake (21 m/s^2), grip-limited at about 11 m/s^2 | brakes yes; the regen limits **no** (add: regen within its torque and power, the rest friction) |
  | steering | max_steer_angle 1.01 rad (58 deg); steering rate_limit, time_constant | rate_limit 0 = none in the spec | lock yes; the rate is a planner constant (2 rad/s) **instead of** the spec's: use the spec's when set, and the ECU actuator's real rate otherwise |
  | jerk / request range | the vehicle interface (Autoware's gate: its acceleration and jerk limits, opened for racing) | race-stack-design.md sets them | **no**: L2 plans its inputs inside the gate's limits, or the gate clips them and the plan is wrong |

  Where each is enforced: L0 and L1 as constraints of the OCP (a plan never asks for more); L2
  as hard input constraints of the NMPC and a clamp in the LQR fallback (a command never asks
  for more); the ECU and the gate keep their own limits as the last guard, not as the place
  the limit is first met. A test per limit checks the plan against it (section 7).

What follows from them: every rate, horizon and budget is sized for the fastest the car will
go on the track, `v_top`, which is at least 110 km/h by SR1 and in practice what L0 (or
`qss.py`) computes for the track (37.2 m/s on Haruna; drift_mustang's spec allows far more on a
long straight: 330 kW, a 16000 rpm motor through 6:1). So `v_top` comes from the car and the
track's geometry, never from a setting.

| quantity | at 110 km/h (SR1 floor) | at Haruna's 134 km/h | sized by |
|---|---|---|---|
| braking distance from v_top at 8 m/s^2 (dry asphalt) | 58 m | 86 m | v^2 / 2a |
| on packed snow (3 m/s^2) | 160 m | 230 m | the same, mu from the surface |
| look-ahead the corner decisions need (braking + the corner, 50-100 m) | 150-200 m | 150-250 m (asphalt), 300+ m (snow) | 4.1 |
| distance driven per L1 re-plan (1 s) | 31 m | 37 m | v_top |
| distance driven per L2 tick (20 ms) | 0.6 m | 0.7 m | v_top |
| distance driven per 10 Hz NDT scan | 3.1 m | 3.7 m | the race estimator must hold up to v_top (swb issue 017: NDT lost the car at 30 m/s on Town05, laps held to 28 m/s): a risk for the estimator, outside this design |

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
  on the whole corner and what follows: braking from Haruna's 134 km/h at 8 m/s^2 takes 86 m
  (from 110 km/h, 58 m; on snow several times more), and a hairpin's entry, apex and exit span
  another 50-100 m. The car must look **150-300 m (5-10 s)** ahead to decide the next corner
  well (1a).
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

**Why ~1 Hz and 150-300 m.** At Haruna's top speed (37 m/s) the car covers 37 m between
plans, a sixth of a 200 m window, so each plan is used for its first part and replaced while the corner is
still ahead; one second leaves room for a solve that converges. "Live" is defined as: the plan
arrives while the car is still in its window's first quarter (solve time x speed < W / 4);
with W 200 m at 37 m/s that is 1.35 s, and up to 3 s is acceptable with W grown to keep the
ratio. W is the braking distance from `v_top` on the surface plus a corner (1a), so it is
longer on snow and on fast tracks; it comes from the car's spec, the surface's mu and L0's
`v_top`, never per track.

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

**Revision: several starts, because a slide is a choice.** Grip and drift through a corner
are different local optima; a solver warm-started from the last plan stays in the mode it
started in and never discovers the other. Revision 3 decides which starts, when, and how one
is chosen: section 4.4a.

**How it meets its budget.** Warm start from the last plan shifted by the distance driven; ds
4 m; IPOPT's wall-time cut. Measured (060, the live model): p50 0.27 s, p95 0.91 s, p99 1.04 s,
5.9 ms an iteration (the linear solve 57 %, the model's derivatives 34 %, IPOPT 9 %). Levers
if it is short, in order (user, 2026-10-30: HSL skipped, free alternatives first): fewer
iterations (each start keeps its own plan), Fatrop (in CasADi; 35-44 % faster at p95 but 17 % of
its plans differ beyond the noise, so not adopted yet), CasADi's SQP on HPIPM, MUMPS tuning. A
GPU does not help a solve this size (roadmap 061).

**When it re-solves.** On a 1 s timer, and at once when the car leaves the plan's envelope
(lateral error > 0.5 m, sideslip off by > 5 deg, speed off by > 2 m/s: set once on the proving
ground, not per track).

**Hand-over.** It solves from the state predicted (by the model, under the current plan) to
solve start + budget, and stamps the plan with that time; L2 switches at its first tick after it,
without blending. L2 follows the plan by s (contouring), so a switch moves the reference, not
the steering, and its input-rate cost smooths the rest.

### 4.4a L1's starts: chosen from what lies ahead (revision 3)

**What was measured** (roadmap 061; autoware-drift `drift-optimality.md`,
`mode-choice-m1.md`, `mode-choice-m3.md`, `mode-choice-methods.md`, `preconditions.md`):
- a drift is the optimum only in tight hairpins on asphalt (at most 0.4 %, an earlier
  rotation, not more speed), and, on the planner's tyre per surface class (061 M2), in every
  corner on loose ground that keeps its grip past the peak (mud, sand, loose snow: ~1.2 %) and
  in hairpins on other loose ground and packed snow (0.3-0.6 %); never on ice;
- the right starts reach it (offline 100 %), the 1 s budget cuts them (live 73-87 %);
- choosing between them gained no lap time in the receding loop (the gate decided 3 times in
  18 runs): the extra starts' value is robustness, a plan when the grip start fails (18 % of
  re-plans at mu 0.3; the single start stopped the car, multi-start never did);
- nothing on the car sees friction 200-260 m ahead (cameras to ~30 m, slip estimators 0 m), and
  nothing senses the fall-off past the peak: over the window the surface is the map's.

**Why choose the starts before the solver.** A gradient sees only the slope where it stands;
grip and drift are valleys with a slower half-slide ridge between them. Racing stacks make such
discrete choices before the solver, never inside it (overtaking side, homotopy class); a
sampler's weighted mean lands on the ridge; mixed-integer is too slow for 1 Hz. So a cheap rule
on the window ahead decides which starts run, and the derivative solver refines each.

**The re-plan, step by step:**

```
 map (zones: class + confidence; geometry, grade, width)
 off-board alerts (only downgrade)  camera 0-30/60 m (confirm, veto)  ECU estimate (only lowers)
        │
        ▼
 1. read the window ── corners (turn, least R, distance ahead); per node class probabilities
        │              -> mu at a lower quantile, fall-off; grade; width
        ▼
 2. schedule starts ── G always; H, D where a drift can pay or a fallback is needed; F rarely
        ▼
 3. solve in parallel ─ one core each, each cut at ~0.9 s, each warm from its own last plan
        ▼
 4. choose ─────────── feasible only; switch mode only if > 0.3 % faster on two re-plans,
        │              grip on a tie, camera veto; G failed: any feasible plan
        ▼
 5. adopt (4.4 hand-over) -> L2 -> ECU;  all failed past the stopping distance -> stop (5)
        ▼
 6. after a pass ───── the car's estimate corrects the map for the next lap
```

**The starts.** G: the last adopted plan shifted (060's only start). H: grip held at the rear
tyre's peak slip angle (`rear_alpha_max`), so a grip plan always exists (a free G slides by
itself on low friction): the fallback, adopted only when no free start has a plan (weighed by
the gate it held grip on its slower plan wherever G slid by itself: Tsukuba +0.55 %, M3b). D: rear out at turn-in, from its own last
plan, else the nearest corner archetype in `corner_library.npz` (solved offline by fall-off
continuation), else a synthetic profile. F: a pendulum flick.

**The schedule** (thresholds from the drift-optimality map, never per track):

| the window ahead holds | starts |
|---|---|
| dry asphalt, no corner past ~120 deg tighter than R 25 m | G (as 060) |
| a dry asphalt hairpin whose drift gain in the map clears the switch threshold (0.3 %: today R <= ~9.5 m, >= 150 deg; on a class that falls off, no wider than the measured cell, 061 M4b) | G, H, D |
| the same at R <= 9 m on a surface without fall-off | + F |
| a hairpin on loose ground falling off past the peak (gravel, dirt, grass) or packed snow, its gain in the map >= 0.3 % (today R <= ~9.5 m; gravel also 15 m-wide R 15) | G, H, D (H on low friction anyway, as the fallback) |
| any corner on loose ground that does not fall off (mud, sand, loose snow), with a probability above p_add (~0.1-0.2), on a road up to ~8 m wide; from 10 m its hairpins and tightest 90s only | G, H, D (+ F into hairpins) |
| ice | G, H (no drift pays; H is the fallback) |
| a steep downhill (> 5 %) on low friction | + D |
| the drift corner under ~25 m ahead | no new D or F |

Width (roadmap 061 M3b's arm, 6-15 m): it decides the 90 deg corners on loose ground without
fall-off (a drift pays 0.6-1.3 % at 6-8 m, 0.05-0.2 % at 10 m, nothing at 15 m) and leaves
hairpins alone (every width pays); asphalt and gravel 90s never pay. So the drift map is keyed on
width: a corner reads the widest measured width no wider than its road, the narrowest if its
width is unknown. The table runs from that map (`schedule.py`, `drift_map.json`), applied to the
corner the seeds go into.

**The slide a plan may ask for** (061 M4b, issue 002). On a surface whose friction does not
fall past the peak (mud, sand, loose snow) a saturated tyre keeps its whole force however deep
it slides, so the free plan plowed the front axle at 9-21 times its peak to brake into the mud
hairpin: a slide with every tyre saturated, which no controller holds (the car spun at 78-80
deg against plans of 52-65). Every L1 window, the seeds' too, keeps the front axle's slip angle
within its peak and the rear's within three times it, on such surfaces only (`window.slip_rows`;
none on asphalt, ice or packed snow, where the tyre's own fall-off keeps a plan out of the
slide). The first plan, before the car moves, is rescued when G fails: more iterations, then H
and G from H, then D and F (issue 001).

**Why loose in, strict out.** Leaving out D costs at most 0.4 % on asphalt, but on low friction
1-2 % and the fallback; carrying it costs a core. So uncertain surface adds starts. Committing
to a drift on a gain the size of the noise costs more in tracking than it gains, so the switch
is strict, and a drift plan must still win when both are scored at the lower friction quantile.
This is motorsport's split: options kept open on a cheap prior (rally's tyres, hours ahead),
committed on a measured crossover (F1's lap-time rule).

**Where the surface comes from.** The map's class per zone with a confidence and an age is the
prior; an off-board alert only downgrades, until it expires; the camera confirms or vetoes in
the last 30-60 m (too late to add a start, in time to veto a commit); the car's own estimate
only lowers friction, holds low through a slide, and writes the map after a pass. The
fall-off always comes from the class's tyre curve (the planner's tyre per class, roadmap 061
M2). L1 plans at the class mixture's lower quantile (mean - k sigma).

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
| G fails, another start succeeds | that start's plan is adopted, whatever its mode | M3: the single start stopped where multi-start went on |
| the surface is worse than the map | the car's estimate lowers friction at once and holds it through the slide; it never raises it mid-slide | 060 M6: an estimate that rose mid-slide crashed the 0.7x patch |
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
2. **SR3 in the model**: the motor's rpm limit, regen within its torque and power, the
   steering rate from the spec (or the ECU's actuator), each with a test that a plan on a long
   straight, a full stop and a quick direction change stays within it; L0 re-solved.
3. **Model-in-the-loop**: L1 re-planning against `sim.py` with today's tracker; then L2 (acados)
   in its place; with the wrong-plant variant.
4. **Isaac, Haruna full run** with L0, L1, L2. Failure: any guardrail contact, or any wheel off
   the asphalt for more than 0.2 s (course_probe logs both).
5. **Disturbances** (course_probe gains an injection hook): a yaw-rate kick of 30 deg/s over
   0.1 s before a hairpin; a 20 m patch at mu 0.6 at a hairpin's entry. The run completes.
6. **Tsukuba**, same code and settings.

Success: milestone 4 within 5 % of L0's optimum (74.8 s), L1 within budget (p99) and L2
within 10 ms over the whole speed range the run covers (to 134 km/h on Haruna, so SR1's
110 km/h is exercised);
milestones 5 and 6 complete. The same run without L0 is reported alongside.

## 8. Left to the builder

W and the grid after the spike; L1's model fidelity and solver; the envelope thresholds; g1;
the margin per edge type; acados options; the LQR's weights; logging; p_add, the quantile's k
and the schedule's thresholds (from the drift-optimality map). Each is set once on the
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
| SR1 / SR2 (1a): at least 110 km/h, no cap, the fastest lap; horizons and budgets sized from v_top (L0's top speed for the track, 134 km/h on Haruna) | user's system requirements |
| SR3 (1a): the car's top speed, acceleration, deceleration and steering limits from its spec, in every layer; three gaps in today's planner named (motor rpm, regen, steering rate from the spec) and the gate's limits | user's system requirement |

## 10. Revision 3: what changed after roadmap 061's measurements, and why

| change | why |
|---|---|
| L1's starts chosen per re-plan from the window ahead (4.4a), not always two or three | asphalt rarely pays for a drift (at most 0.4 %); the cores and the budget go where a drift or a fallback can pay |
| a held grip start H beside the free G | on low friction a free G slides by itself, so there was no grip plan to weigh against |
| D from its own last plan, else a corner library solved offline by continuation, else synthetic | a drift start from scratch takes 230-280 iterations at p95; the library's starts reach the optimum most often |
| the switch: > 0.3 % on two re-plans, grip on a tie, scored at the lower friction quantile | 0.3 % is the solver's noise (p75 at mu 0.45); a 0.1 % gate took a noise-sized gain and gave it back |
| any feasible start's plan when G fails | the extra starts' measured value is robustness, not lap time |
| the surface from the map with a confidence; alerts only downgrade, the camera vetoes, the car's estimate only lowers and holds | no sensor sees 200-260 m ahead, none senses the fall-off; 060 M6's estimate rose mid-slide |
| the planner's tyre peaks at each surface class's own slip; the schedule keyed on class, not on mu | swb's surface layer does (061 M2); on it drift-optimal low-friction cases fell from 53 to 32 of 80 and the gain depends on the class's fall-off, not its friction (ice: none) |
| width keys the drift map (M3b's arm); H is the fallback, not a competitor; G alone is solved as 060's, in the driver's own process | narrow loose roads drift in 90 deg corners, wide ones do not, hairpins at every width; H weighed in the gate cost Tsukuba 0.55 %; a G-only re-plan is then 060's exactly (R6) |
| on a class that falls off past the peak (asphalt, gravel, dirt, grass, packed snow) a D start only where the map's gain for the corner clears the switch threshold (0.3 %), not the noise floor (0.05 %); classes without fall-off unchanged (user, 2026-10-30: "we don't pursue drift start for most dry asphalt roads") | a D whose gain the gate can never take is a core spent for nothing where G does not fail; Haruna's chain: G alone on 51-59 % of re-plans (was 43 %), 1.73-1.81 solves a re-plan (2.13), the same time; Tsukuba G alone throughout (061 M4) |
| on a class that falls off, D only up to the measured paying cell's radius (no 1.25 x stretch) | D into Haruna's R 9.9-11.2 m hairpins, inside the stretch, cost 0.09 s on the plant in three of three; without it Haruna's scheduled arm is 060's plans bit for bit (0.000 %, 061 M4b) |
| L1 keeps the front axle within its peak and the rear within three times it, on surfaces without fall-off | the free plan plowed the front at 9-21 times its peak into the mud hairpin, every tyre saturated, a slide no controller holds: every arm stopped there (6 of 6); bounded, none (0 of 6), the car's largest sideslip below its plans' (23-32 against 27-36 deg; 061 M4b, issue 002) |
| the first plan rescued from more starts when G fails (3000 iterations, H then G from H, D, F) | two tight low-friction hairpins had no first plan in any arm; offline both had plans from H or past 400 iterations (061 M4b, issue 001) |
| solver levers: fewer iterations, then Fatrop, SQP on HPIPM, MUMPS tuning; no HSL, no GPU under the solver | user's ruling; the measured cost split; a GPU loses at ~1000 variables |

## 11. Revision 4: a driving style above the planner (roadmap 065)

The user, 2026-10-31: "the behavior should be adjustable. The solver can go fastest lap, drift
for little trade off or always making drift if possible even on straight roads", and "The style
or behavior is like a layer we can decide the combinations". The layer is specified in
[2026-10-31-drift-style-layer-design.md](2026-10-31-drift-style-layer-design.md), from the
evidence in [drift-style.md](../../drift-style.md); built and measured in roadmap 065.

| change | why |
|---|---|
| a style record (S1 `fastest`, S2 `drift_budget`, S3 `drift_max`) chooses the formulation each piece runs: offline data, schedule (4.4a), L1 rows, gate, L2 weights; L1's objective stays minimum time in every style | a drift reward in the cost cannot reach every trade-off on a non-convex problem, holds no single weight across re-plans, and pays the plan to sit on the slip cap (issue 002); every source that defines a drift well defines it as constraints chosen upstream |
| S1 is this design unchanged, bit for bit | the fastest lap must not pay for the layer |
| S2: the drift map stores signed losses; a lap budget (seconds) spent offline by a knapsack; a lex-epsilon gate (plans within 1 + eps of the fastest, the one that drifts most) on classes without fall-off | on asphalt the cheapest drift (Haruna's hairpins, 0.09 s a lap) is per window the size of the 0.3 % noise, so the live gate cannot price it |
| S3: an offline zone plan (where, which way, switch points); elastic L1 rows in zones (rear-slip floor, speed floor, swing rate, no-return); a manji seed M on straights; heavier L2 weights on sideslip and yaw rate; fallback to grip, then stop | drift direction is discrete and a gradient cannot choose it; D1 scores speed x angle, so minimum time inside floors is D1's own task |
| section 5's failure table, the grip share, edge margins, slip caps, ice rule, friction quantile and estimator rules are not style fields; a style may only tighten them, and a record that breaks one is refused by name | no style buys drift with safety |

