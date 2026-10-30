# L1's starts chosen from what lies ahead (roadmap 061 M3b)

Written 2026-10-30. M1-M3 showed that a start in the drift valley is worth little lap time on
dry asphalt (at most 0.4 %, tight hairpins only) and a lot of robustness on low friction (a plan
when the grip start fails). M3b decides, before each re-plan, which starts run, from what the
window ahead holds: its corners, the surface class each node is (from the map, with a
confidence), the grade and the width. Design: the live planner's design, section 4.4a
(revision 3). The surface's sources follow `preconditions.md`'s recommendation.

Code (autoware-drift `src/planning/race_planner`): `race_planner/schedule.py` (the scan, the
drift map, the table), `race_planner/surface.py` (the map's zones, perception, the car's
estimate), `race_planner/drift_map.json` (the table's thresholds, from the study),
`multistart.py`'s scheduled mode, `live.Driver(estimator=...)`; `studies/drift_optimality.py`
grids `m3b-width`, `m3b-width-report`, `drift-map`; `studies/mode_choice_m3b.py` (the chain
check); `test/test_schedule.py`. Results: swb's `play_log/race_planner/drift_study/m3b-width.*`,
`m3b_width.md`, and `play_log/race_planner/mode_choice/m3b/`.

## 1. The width arm: does the road's width move the drift optimum?

The study's sections were all 8 m wide (Haruna's). The arm re-runs the corners where a drift
pays after M2 at 6, 8, 10 and 15 m: R 9, 15, 25 m at 90 and 180 deg, on asphalt (swb's: mu
1.12, sliding 0.8), mud_wet and snow_loose (no fall-off past the peak) and gravel (falls off);
96 cases, L1's window at study settings, free against the rear held at its peak, as M2.

Each cell: the drift's gain, % of the section's time (how far the free plan's rear axle runs past its
peak, deg); bold where a drift is optimal.

| class | R | turn | 6 m | 8 m | 10 m | 15 m |
|---|---|---|---|---|---|---|
| asphalt | 9 | 90 | 0.00 (+1.5) | -0.00 (+1.5) | 0.00 (+1.7) | 0.00 (+1.5) |
| asphalt | 15 | 90 | -0.01 (+1.5) | 0.00 (+1.5) | 0.01 (+1.6) | 0.00 (+1.4) |
| asphalt | 25 | 90 | -0.00 (+1.4) | -0.00 (+1.5) | -0.01 (+1.5) | -0.00 (+1.3) |
| asphalt | 9 | 180 | **0.48 (+6.9)** | **0.40 (+6.5)** | **0.39 (+6.3)** | **0.32 (+5.5)** |
| asphalt | 15 | 180 | **0.21 (+4.5)** | **0.23 (+4.5)** | **0.27 (+4.7)** | **0.29 (+4.8)** |
| asphalt | 25 | 180 | **0.08 (+3.2)** | **0.09 (+3.2)** | **0.10 (+3.1)** | **0.12 (+3.3)** |
| mud_wet | 9 | 90 | **1.19 (+11.2)** | **1.21 (+11.5)** | **0.17 (+3.5)** | 0.05 (+3.0) |
| mud_wet | 15 | 90 | **1.30 (+10.7)** | **0.64 (+5.2)** | **0.10 (+3.2)** | 0.04 (+2.8) |
| mud_wet | 25 | 90 | **0.16 (+3.8)** | **0.12 (+3.1)** | **0.05 (+3.1)** | 0.01 (+2.6) |
| mud_wet | 9 | 180 | **1.43 (+30.3)** | **1.19 (+31.1)** | **1.05 (+28.3)** | **0.96 (+25.8)** |
| mud_wet | 15 | 180 | **0.67 (+23.5)** | **0.61 (+23.3)** | **0.68 (+24.2)** | **0.70 (+23.6)** |
| mud_wet | 25 | 180 | **0.37 (+12.2)** | **0.35 (+12.9)** | **0.39 (+14.0)** | **0.46 (+16.8)** |
| snow_loose | 9 | 90 | **0.65 (+7.9)** | **0.73 (+8.0)** | **0.19 (+3.4)** | 0.03 (+2.7) |
| snow_loose | 15 | 90 | **0.60 (+6.5)** | **0.62 (+7.3)** | 0.08 (+3.0) | 0.02 (+2.6) |
| snow_loose | 25 | 90 | **0.65 (+6.4)** | 0.11 (+2.9) | 0.02 (+2.8) | 0.01 (+2.4) |
| snow_loose | 9 | 180 | **1.17 (+25.0)** | **1.11 (+27.8)** | **0.97 (+25.9)** | **0.94 (+23.4)** |
| snow_loose | 15 | 180 | **0.53 (+19.2)** | **0.58 (+21.4)** | **0.65 (+22.1)** | **0.70 (+21.7)** |
| snow_loose | 25 | 180 | **0.31 (+11.2)** | **0.25 (+11.9)** | **0.32 (+12.9)** | **0.36 (+16.0)** |
| gravel | 9 | 90 | 0.01 (+1.6) | 0.01 (+1.7) | 0.01 (+1.9) | 0.02 (+2.0) |
| gravel | 15 | 90 | -0.01 (+1.4) | 0.00 (+1.8) | 0.00 (+1.8) | 0.01 (+1.9) |
| gravel | 25 | 90 | 0.00 (+1.5) | 0.01 (+1.7) | 0.00 (+1.7) | 0.00 (+1.5) |
| gravel | 9 | 180 | **0.56 (+9.3)** | **0.61 (+9.5)** | **0.50 (+8.5)** | **0.44 (+7.8)** |
| gravel | 15 | 180 | **0.18 (+5.4)** | **0.15 (+5.4)** | **0.25 (+5.6)** | **0.35 (+6.4)** |
| gravel | 25 | 180 | 0.04 (+3.6) | -0.09 (+3.5) | **0.05 (+3.6)** | **0.18 (+5.0)** |

**Verdict: width moves the optimum, but only in 90 deg corners on loose ground without
fall-off.**
- There the drift is a narrow road's: 0.6-1.3 % at 6-8 m, 0.05-0.2 % at 10 m (inside the noise
  at R 15-25), none at 15 m. The slide shrinks with the road: the rear runs 8-11 deg past its
  peak at 6-8 m, 3 deg at 10-15 m (most likely a wider road lets a grip line open the corner,
  which is what the rotation buys on a narrow one; not examined further).
- Hairpins (180 deg) pay at every width on every class tested (asphalt 0.08-0.48 %, gravel
  0.15-0.61 % where it pays, mud and loose snow 0.3-1.4 %), the slide as deep (mud R 9: 26-31
  deg past the peak). The gain falls a little with width at R 9 (asphalt 0.48 -> 0.32 %) and
  rises a little at R 15-25 (asphalt 0.21 -> 0.29 %, 0.08 -> 0.12 %; gravel R 25 pays only from
  10 m): a wider hairpin has room for the rotation.
- Asphalt and gravel 90 deg corners never pay, at any width (|gain| <= 0.02 %).

**The rule it gives**: the drift map is keyed on width as well as class, radius and turn; a
corner reads its class's cells at the widest measured width no wider than its road (narrower
drifts more: loose in), the narrowest where its road is narrower than all of them or its width
is unknown. Radii the arm did not run (40, 60 m) keep their 8 m cells at every width. In the
table of 4.4a this changes one row: "any corner on loose ground without fall-off" holds on roads
up to about 8 m; from 10 m only its hairpins and its tightest 90s (R 9) pay.

## 2. What was built

**The scan** (`schedule.scan`): the corners whose exit is ahead of the window's start and whose
turn-in is inside it (`modes.corners`: curvature tighter than R 150 m smoothed over 12 m, same-
hand stretches under 12 m apart joined, turning 30 deg or more), each with its distance ahead,
least radius, turn, its classes' largest probabilities over the corner and the 60 m before it,
the steepest descent there and the least width; the window's own class probabilities; with a
surface map, each node's friction mean, sigma and lower quantile.

**The drift map** (`schedule.DriftMap`, `drift_map.json` built by `drift_optimality.py
drift-map`): per class, the cells (R, turn, gain, width) where the free plan's rear axle ran 3
deg or more past its peak and beat the held plan beyond the noise: asphalt from the rear grid
(mu 1.12, sliding 0.8), the other classes from `m2-classes` (8 m), the width arm's classes at
every width. Ice has none. A corner pays for a class if a cell that pays (gain >= 0.05 %) is no
tighter (R <= 1.25 R_cell) and turns no further (turn >= turn_cell - 30 deg). Aliases: concrete
-> asphalt, mud_soft -> mud_wet, sand_loose -> sand; a class in neither is seeded as the most
drift-friendly (every class's cells).

**The table** (`schedule.Scheduler.starts`, its thresholds in `schedule.RULES`), applied to the
corner the choice is about (multistart's `corner_ahead`: the first whose turn-in is 10 m or more
ahead, the corner the seeds go into), the union over every class at least `p_add` (0.15)
probable:

| start | when |
|---|---|
| G | always (060's start, solved in the driver's own process) |
| D | the drift map says a drift pays in this corner, or a steep downhill (grade < -5 %) on low friction (a class with mu <= 0.45) |
| H | with D; and whenever the window holds low friction (the fallback when G fails) |
| F | with D, into a hairpin (>= 150 deg) on a class without fall-off (sliding >= 0.95), R <= 9 m on asphalt-like friction (mu >= 0.9), R <= 15 m elsewhere |
| no new D, F | the corner's turn-in under 25 m ahead (a D or F already sliding in it keeps its plan) |

On Haruna this asks for H and D at all seven hairpins (R 10-13 m, 152-185 deg) and G alone
elsewhere; on Tsukuba at three of its seven corners.

**H** (`multistart`): grip held at the rear tyre's peak, `Window(rear_alpha_max=...)`, in its own
process; one compiled window per bound in whole degrees (`held_bound`: the degree below the
peak plus 1, the study's margin), every bound the track needs compiled when the driver starts;
the window's bound is its least peak's. Its start: its own last plan shifted, else the grip
base with the rear's slip clipped into the bound. A held window has more rows, so a plan's
multipliers are handed to another window only when they fit (`multistart._fitting`).

**G in the driver's process** (scheduled mode): solved as 060 solves it, by `Driver.warm`, its
multipliers kept as 060 keeps them while the adopted plan is G's. A re-plan that runs G alone is
060's re-plan exactly (tested: the same plans, bit for bit, on an asphalt bend); its wall time is
G's solve time.

**The gate** (unchanged from M3: 0.3 % of the window's time, two re-plans in a row, grip on a
tie; G failed: any feasible plan) now scores plans at the lower-quantile friction, because every
start solves the window on the planner's track, whose friction is the quantile.

**The surface** (`surface.py`):
- `SurfaceMap`: zones (s0, s1, declared class, confidence, true class, alternatives) over a base
  class. Each node's class probabilities (the declared class at its confidence, the rest over
  the alternatives, every other class by default); the friction planned with, mean - k sigma
  over the mixture (k 1, each class's own spread `sd` 0 by default), never below the lowest class
  of probability 0.15 or more; `planner_track` (that friction, the most probable class's
  sliding share and tyre at it: swb's rule) and `true_track` (each node's true class: what
  `sim.Plant` drives on). A map can be wrong on purpose (declared != true). A certain map gives
  the planner exactly the Track `from_swb_course` gives (tested on Haruna).
- `Perception`: the true class of each patch within 60 m, right with probability 95 % within
  30 m, falling linearly to chance (1 in 12 classes) at 100 m, otherwise one of the others at
  random; one read per patch per call; seeded. In the scheduled driver it lowers the planned
  friction where it reads a class with less friction within 30 m (never raises it) and vetoes a
  gate switch into a drift when every class it reads in the corner is one where no drift pays.
  It never adds a start.
- `FrictionEstimate`: the car's estimate as a scale on the map: a reading only lowers it, it
  holds while the car slides (|beta| >= 5 deg, or past its plan's by 2.5 deg) and 1 s after,
  then rises at 0.05 a second; its log `(s, friction)` and `SurfaceMap.correct` write a lower
  class into the map after a pass (only lower). `live.Driver(estimator=...)`, used with
  `use_estimate`; both off by default (060's).

## 3. The receding-chain check

As E1: `live.Driver` re-plans along a course while the car follows the adopted plan exactly
(`sim.PlanFollower`: no plant, no tracker, no model error); one run each, cores 8-10 for the
seeds, host load 7-15. Courses: Haruna and Tsukuba (one lap) dry; `zones`, E1's chain of the
study's ten corners (R 9-60 m, 90 and 180 deg, 8 m wide) with each corner in a zone of one of
swb's classes (60 m before its turn-in to 20 m past its exit), the map right, one zone declared
asphalt at 0.85 with mud_wet the alternative. Configs: `single` (060), `sched` (M3b), `always`
(G, H, D every re-plan, F into hairpins). `studies/mode_choice_m3b.py`, results swb's
`play_log/race_planner/mode_choice/m3b/` (report.md).

| course | config | result | time (s) | vs single | re-plans | solves / re-plan | G only | with H | with D | with F | wall p50/p95/max | overruns > 1.0 s |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| haruna | single | done | 76.90 |  | 75 | 1.00 | 75 (100 %) | 0 | 0 | 0 | 0.252/0.701/0.90 | 0 |
| haruna | sched | done | 76.85 | -0.065 % | 75 | 2.13 | 32 (43 %) | 43 | 42 | 0 | 0.32/0.667/0.83 | 0 |
| tsukuba | single | done | 72.65 |  | 71 | 1.00 | 71 (100 %) | 0 | 0 | 0 | 0.264/0.9/1.01 | 3 |
| tsukuba | sched | done | 72.75 | +0.138 % | 71 | 1.51 | 53 (75 %) | 18 | 18 | 0 | 0.352/0.931/1.01 | 3 |
| zones | single | stopped: solves failed | 10.40 |  | 10 | 1.00 | 10 (100 %) | 0 | 0 | 0 | 1.004/1.005/1.01 | 9 |
| zones | sched | done | 144.95 |  | 140 | 2.30 | 20 (14 %) | 120 | 52 | 10 | 0.794/1.02/1.03 | 18 |
| zones | always | done | 146.50 |  | 142 | 2.92 | 0 (0 %) | 142 | 111 | 20 | 0.795/1.02/1.02 | 19 |

**Dry asphalt.** The table asks for H and D at the hairpins and for G alone elsewhere: G alone on
43 % of Haruna's re-plans (seven hairpins in 1.5 km: 2.13 solves a re-plan) and 75 % of
Tsukuba's (1.51). Not "G only on nearly all re-plans": these two tracks are hairpin tracks, and
the asphalt hairpin row (0.1-0.4 %) fires as written. Plan time against single: Haruna -0.065 %,
Tsukuba +0.138 % (one run each; the course time is read on a 0.05 s step, 0.07 %). Every G-only
re-plan is 060's exactly; the difference comes from the hairpins, where a D plan that fell back
into the grip valley (a few ms faster) or a G plan sliding past the 3 deg line is adopted. R6
(0.1 %) holds on Haruna and is missed by 0.04 % on Tsukuba: inside the measurement's step and
M3's run-to-run spread, but not shown to be noise (no repeats).

**Class zones.** The single start stopped at s 191 ("solves failed": every re-plan from s 118
cut at the 1.0 s budget once the window reached the mud hairpin's zone; a first run, before the
spin limit was raised, stopped "spin" at s 248); the scheduled and always-on starts finished. The starts
fired where the table says, corner by corner:

| s_in | R | turn | classes (p) | table | why | ran |
|---|---|---|---|---|---|---|
| 84 | 25.0 | 86 | asphalt 0.85, mud_wet 0.15 | GHD | mud_wet pays | GH x1, GHD x2 |
| 248 | 9.0 | 179 | mud_wet 1.0 | GHDF | mud_wet pays | GHDF x9, GHF x1 |
| 408 | 60.0 | 86 | snow_loose 1.0 | GH |  | GH x15 |
| 628 | 15.0 | 177 | gravel 1.0 | GHD | gravel pays | GHD x10 |
| 804 | 40.0 | 178 | ice 1.0 | GH |  | GH x12 |
| 1056 | 9.0 | 87 | snow_loose 1.0 | GHD | snow_loose pays | GH x1, GHD x11 |
| 1200 | 25.0 | 176 | snow_packed 1.0 | GHD | snow_packed pays | GH x1, GHD x13 |
| 1408 | 60.0 | 178 | asphalt 1.0 | G |  | G x5, GH x8 |
| 1720 | 15.0 | 90 | mud_wet 1.0 | GHD | mud_wet pays | GHD x7 |
| 1876 | 40.0 | 87 | asphalt 1.0 | G |  | G x5, GH x7 |

The extra H on the two asphalt corners (1408, 1876) is the window's low-friction rule (mud and
snow zones inside the 260 m window), not the corner's; "GH x1" before a hairpin is the re-plan
where the corner was under 25 m ahead and no D was running. Solves per re-plan: scheduled 2.30
against always-on 2.92 (-21 %; G alone on 14 % of the scheduled re-plans, fewer than the dry
corners because of the window rule); course time 144.95 s against 146.50 s (-1.1 %, one run). Overruns past
1.0 s: 18 of 140 (scheduled), 19 of 142 (always-on), max 1.03 s, on low friction where seeds run
into their 0.9 s cut; the R3 budget needs the quiet cores M3 ruled for.

Before the H ruling (section 4) a first run had H weigh in the gate: Haruna -0.07 %, Tsukuba
+0.55 % (H's slower held plan adopted for a re-plan or two at each hairpin while the gate waited
to confirm a G that slid by itself); `play_log/.../m3b/` keeps only the second run.

## 4. Rulings

- **Width: the drift map is keyed on width; a corner reads the widest measured width no wider
  than its road, the narrowest if unknown.** Why: the arm shows width decides 90 deg corners on
  loose ground without fall-off (narrow 0.6-1.3 %, 15 m none) and leaves hairpins alone. Cost if
  wrong: a 9 m road reads 8 m cells and seeds a 90 deg mud corner whose gain is ~0.1 % (one core
  for nothing); between measured widths the rule is a step, not an interpolation.
- **The table is applied to the corner the choice is about** (the first whose turn-in is 10 m or
  more ahead), not to every corner in the window. Why: the seeds go into that corner (M3's D and
  F are built for it). Cost if wrong: a paying corner second in the window is seeded only once it
  is first, still 25 m or more ahead on several re-plans at 30 m/s; a window with two paying
  corners seeds them one after the other.
- **G is solved in the driver's process, H, D and F in theirs.** Why: a re-plan that runs G alone
  is then 060's exactly (tested bit for bit), so R6 holds wherever the table asks for G alone.
  Cost if wrong: the driver's process needs a core of its own while the others solve (four busy
  processes at most per re-plan).
- **H is the fallback, not a competitor (default; `h_competes=True` makes it one).** Why: in the
  first chain run H competed, and wherever the free G slid by itself (asphalt hairpins: its
  optimum is 3-6 deg past the peak) the gate held grip on H's slower plan until the slide was
  confirmed: Tsukuba +0.55 % against single, past R6's 0.1 %. The extra starts' measured value is
  robustness. Cost if wrong: where G slides by itself on low friction, the gate never weighs the
  slide against a held grip plan; M4 can measure the competing arm.
- **H's bound is the window's least rear peak, rounded to the whole degree below plus 1, its
  windows compiled when the driver starts.** Why: the bound is compiled into the window (about
  30 s a bound, cached on disk); whole degrees need six windows for the zones course. Cost if
  wrong: on a window mixing classes H is held to the lowest peak everywhere (slower on the
  grippier part); it is a fallback, so feasibility matters more than its time.
- **The planned friction is the class mixture's mean less one sigma, never below the lowest class
  at 0.15 or more; each class's own spread is 0 by default.** Why: a certain map then plans as 060
  (R6 by construction); only map uncertainty costs time. Cost if wrong: the within-class spread
  (preconditions: about 0.1) is not planned in; `SurfaceMap(sd=...)` carries it for M4 to sweep.
  A 15 % chance of mud on an asphalt corner plans it at 0.70 instead of 1.12: uncertainty is
  expensive, by design (loose in, strict out).
- **p_add 0.15, gain_min 0.05 %, R x 1.25 and turn - 30 deg tolerances** are starting points
  from the study's grid spacing and preconditions' expected-value estimate, not measured optima.
  Cost if wrong: M4's sweep moves them; the tolerances decide the 90-150 deg corners the grid
  never ran.
- **Perception vetoes only the gate's switch into a drift, never a forced change, and lowers
  friction only within 30 m.** Why: a forced change has no other plan; beyond 30 m its accuracy
  falls toward chance. Cost if wrong: a wrong veto keeps grip for one corner (at most the
  drift's gain); untested in the loop (M4).
- **The estimate's hold (|beta| >= 5 deg, or 2.5 deg past the plan; 1 s; release 0.05/s) and the
  map correction (only lower, 0.05 below, 10 m, confidence 0.9)** are unit-tested only. Cost if
  wrong: M4 tunes them; off by default.
- **The chain check stops at 89 deg of sideslip, not 60.** Why: with no plant the car is its plan,
  and on mud M2's tyre plans the R 9 hairpin past 60 deg (the single start stopped there at s
  248); the spin stop is the plant's rule. Cost if wrong: the zones course's mud hairpin plans
  are slides a real car may not hold; M4 with the plant judges them.
