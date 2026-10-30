# Grip or drift chosen live: the multi-start driver without a plant (roadmap 061 M3, E1)

Written 2026-10-30 (v3 the same day). M1 found that a grip start and a stored rear-out start together reach the
oracle in most drift-optimal corners, offline and one window at a time. M3 builds that into the
live loop (`live.Driver(multistart=...)`) and asks what it does over a whole course of re-plans:
how often the mode a corner is planned in changes, whether it flips on asphalt, whether the
re-plan keeps its 1 s budget, how many iterations each seed needs re-plan after re-plan, what
the threshold changes, and what it is worth against 060's single start. "Without a plant": the
car follows the adopted plan exactly, so only the planning is measured. It also folds in the
solver-cost checks the user asked for in place of HSL (where IPOPT's time goes; Fatrop).

Code: `race_planner/multistart.py`, `race_planner/modes.py` (seeds, corners, the corner
library `corner_library.npz`), `live.py` (`multistart` option, `warm_from`), `sim.py`
(`PlanFollower`, `NoTracker`), `serve.py` (the init option); the experiment
`studies/mode_choice_e1.py`, the solver checks `studies/solver_cost.py`. Results (json per run,
logs, `report.md`, `solver_report.md`): swb's `play_log/race_planner/mode_choice/m3/`. Three
passes, the code changing between them (see *Rulings*): v1, a plan's mode over its whole window
(`m3/window_mode/`); v2, the mode decided in the corner ahead and, with none ahead, over the
window (`m3/corner_mode/`); v3, the final code: with no corner ahead every plan keeps the current
mode (`m3/`, the single-start baselines shared with v2).

## Verdict

**The gate as written fails, on two counts; with the counting ruled below it passes at the 0.3 %
threshold, and the chooser still has almost nothing to choose.**

v3 ran on a host far busier than v2's (load 38-60 against about 20; the seeds' cores shared with
another user's niced work): 6.7 ms an iteration over all seed solves against v2's 4.9, and 40 %
of seed solves cut at 0.9 s against 18 %. Its mode-change counts are the final code's; its timing
is the host's, and v2's is the better measure of the budget.

- *At most one mode change per corner*: counted as written (every change of the chooser's mode
  while a corner is the one ahead), v3 has 10 of 18 runs within it (v2 14). Three kinds of change
  make up the rest, none a flip between two valleys:
  - the previous corner's slide ending: the chooser enters a corner still in the last one's
    drift, and the drift-to-grip as that slide ends is booked on the next corner;
  - *forced by the budget*: the current mode's only plans were cut at 0.9 s, so the other
    mode's plan was the only one (v3: 51 changes, v2: 20). This is load, and it belongs to the
    overrun criterion;
  - *forced at the line*: every plan in the other mode, often a slide at the 3 deg line that
    defines a drift (the same plan reading drift one re-plan and grip the next).
  Counting the first as the previous corner's and the second as the budget's (*Rulings*), v2 has
  no exception in 18 runs and v3 one: Haruna with mu 0.3 patches at the 0.1 % threshold, its
  last hairpin (s 1411), where the gate took the flick's drift at a 0.11 % gain and the next
  re-plan's flick fell back into grip, so the drift was forced off. At 0.3 % (the threshold M1's
  ruling chose) and 0.5 % no corner has more than one change.
- *No grip-drift flip-flop on asphalt*: the adopted plans never flipped in a corner on asphalt
  (0 of 6 runs, v2 0; single start: 3 on Haruna), and the chooser's own mode did not either (v2:
  twice, both at the 3 deg line on Haruna's first hairpin, which the v3 rule removed).
- *Budget overruns at most 1 % of re-plans*: v3 **25 of 1,680 (1.5 %)**, wall p95 0.94 s, max
  1.18 s, all from runs on the loaded cores (chain mu 0.45 at 0.1 %: 6.2 %); v2 **0 of 1,969**
  (p95 0.92 s, max 0.93 s). Each seed is cut at 0.9 s, so a re-plan over 1.0 s is a seed process
  not scheduled in time, not a solve too long: the criterion passes on a quiet core (v2) and
  fails on a saturated one (v3).
- *Three v3 runs stopped* ("solves failed": chain mu 0.3 at 0.1 %, at 45 s; Haruna mu 0.45 at
  0.3 % and 0.5 %, at 9 s) where all three seeds were cut at 0.9 s re-plan after re-plan (87 %
  of seed solves cut in the Haruna runs, 60-130 iterations done in the 0.9 s, 7-15 ms each). The
  same courses finished in v2 at 4.5-5.4 ms an iteration. Multi-start's robustness (below) holds
  only while an iteration costs what the budget was sized for. Re-run once the load had fallen
  to about 15 (`m3/v3_rerun_stopped/`, the same code: chain mu 0.3 at 0.1 and 0.3 %, Haruna mu
  0.45 at 0.1 and 0.3 %), all four finished, 0 re-plans over 1.0 s, 5.3-6.2 ms an iteration,
  no corner with more than one change under the counting ruling; course time vs single +0.94,
  -0.27, 0.00, 0.00 %.

v2 to v3: with no corner ahead, v2 classified the plans over the window, so a slide carried on
into the straight, or a corner just behind, could force the mode on and off between corners; v3
keeps the current mode there. The chooser's changes fell from 136 in 1,969 re-plans (6.9 %) to
83 in 1,680 (4.9 %); the gate decided 3 times (v2: 2).

**What the chooser does** (v2's numbers, the quieter host's; v3 agrees where the budget let it):
in the receding loop G (060's warm start) and the rear-out seed almost always land in the same
mode in the corner ahead: in 71-87 % of the re-plans where D ran the two agree, one or both
failing is the next most common outcome (8-29 %; v3, with 40 % of seed solves cut, 57 % agree
over all runs), and both modes had a feasible plan in only 0-11 re-plans per run (v3 0-7). On
low friction G's own plan slides into the corner by itself (060 already drives 7-9 of 10 chain
corners and every Haruna patch hairpin in a drift); on asphalt D's seed falls back into grip or
fails. So **the gate decided 2 times in v2's 18 runs and 3 times in v3's** (Haruna at 0.1 %
twice, Haruna mu 0.3 patches at 0.1 % once; v2: Haruna and Haruna 0.45 at 0.1 %): always at the
0.1 % threshold, never at 0.3 or 0.5 %; every other change followed the plans. The threshold
sweep therefore moves nothing systematically: course times differ by -0.81 to +0.61 % from the
single start's in v2 and -0.59 to +0.88 % in v3, in no order of threshold, which is the receding
loop's own path dependence (a plan a few ms faster or slower moves every later window).

**What multi-start is worth here**: not lap time -- chains and Haruna within +-0.9 % of the
single start, either sign -- but **robustness, while the iteration cost holds**: when G's solve
fails (v2: 5 % of re-plans on mu 0.45, 18 % on mu 0.3, 0.4 % on asphalt) a seed's plan is
adopted instead, and Haruna with mu 0.3 patches is finished (all six multi-start runs, v2 and
v3) where 060 stopped at 10 s. But on v3's saturated cores three multi-start runs stopped the
same way: when every seed is cut, more seeds do not help.

**D does not converge warm across re-plans on low friction** (v2). D's own plan shifted needs
p50 96-106 iterations against a fresh Dc's 114-146 (mu 0.45, 0.3), p95 172-176 against 183-187,
and 65-110 at the median even four or more re-plans after its seed. On asphalt it does (own 46
against a seed's 96-165, as G's 48-50). G itself needs 90-134 iterations at the median on low
friction (single start: 90-146), against 50 on asphalt: low friction
is costly for every start, so continuation in time does not make the drift seed cheap there.

**Solver cost.** IPOPT spends **57 % of a live solve in the linear solver** (MUMPS, factorising
inside the augmented system's solve), 34 % in the problem's functions (the Hessian 18 %, the
Jacobian 9 %) and 9 % in itself (279 Haruna windows, G and Ds, 5.9 ms an iteration on a quiet
core). **Fatrop** (CasADi's, the window's own compiled per-stage derivatives, structure
detected) costs 3.3 ms an iteration (its own algorithm 0.6 ms, the functions 2.2 ms) and takes
somewhat more iterations (G p50 94 against 69, Ds 210 against 179); wall p95 drops **44 % for
G (1.50 -> 0.84 s) and 35 % for Ds (2.23 -> 1.45 s)**, and Ds within 0.9 s goes from 30 % to
73 %. But the plans do not match within the noise: 44 of 263 windows (17 %) differ by more than
0.3 % (Fatrop slower in 28, faster in 16; p50 0.07 %), Fatrop fails 12 windows IPOPT solves
(IPOPT 3 of Fatrop's), and Fatrop 1.1.8 takes no multipliers to warm-start from. **Not adopted
for live** (the rule: same plans within noise and p95 materially lower); reported, with a
recommendation below.

## Method

### What was built

- **`live.Driver(multistart=...)`** (opt-in; `None`, the default, is 060's one start: the warm
  start moved into `live.warm_from` unchanged, the solve and its adoption as before). With a dict
  of options it builds a `multistart.MultiStart`: one process per seed (`spawn`, one OpenMP
  thread, optionally pinned to `cpus`), each with its own compiled `Window` and its own budget
  (`seed_budget`, IPOPT's `max_wall_time`, by default 0.9 of the re-plan's). Every seed's start
  is sent at once; the re-plan's latency is the slowest answer's wall time. Adoption is 060's:
  at the fixed latency (the budget), later if the answers overran it, and only once the car
  reaches the plan's first node. `serve.py` takes `"multistart": {...}` in its init line.
- **Seeds**:
  - *G*: the adopted plan shifted while it is a grip plan (060's start, its multipliers with
    it); while a drift is adopted, G's own last plan shifted, so the grip valley stays seeded
    (the adopted plan if G's own last solve failed);
  - *D*: its own last plan shifted while that still slides in the corner ahead (its multipliers
    with it: continuation in time); otherwise a fresh seed for the corner ahead: *Dc* from the
    corner library where an archetype is near, *Ds* (M1's synthetic) where none is; idle when
    no corner is ahead;
  - *F* (`flick: True`): as D, only when the corner ahead is a hairpin (R <= 15 m, >= 150 deg).
- **The corner library** (`modes.Library`, `corner_library.npz`, 120 kB): the drift study's
  archetypes (R 9-60 m x 90/180 deg x mu 0.45/0.3/1.12 x fall-off 1.0/0.8/0.6), each case's
  fall-off continuation plan from M1's study-setting grid (Dc), kept where it slides: 60 of 70.
  Built by `studies/drift_optimality.py library`. Mapped onto a live corner aligned on its
  turn-in and exit (the arc stretched), mirrored for a right turn: the archetype's rear-axle
  slip (its excess past the peak kept, the peak the live corner's), its line (within the road)
  and its request (times the friction ratio) put into G's warm start from 40 m before the
  turn-in to 20 m past the exit (M1's `reshape_slip`, now in `modes.py`). Nearest by log R,
  turn, mu and fall-off; none within the tolerance: Ds.
- **Corners** (`modes.corners`): stretches tighter than 150 m (curvature smoothed over 12 m),
  one hand's joined across gaps under 12 m, turning 30 deg or more.
- **A plan's mode** is what it does in the corner the re-plan is about (the first whose turn-in
  is 10 m or more ahead): drift if its rear axle runs 3 deg or more past the tyre's peak (the
  planner's tyre at the planner's friction) from 30 m before the turn-in to the exit; not the
  seed it came from. With no corner ahead (the car in the corner it chose for, or past it, and
  the next turn-in less than 10 m ahead or outside the window's first 220 m) there is nothing to
  choose: every feasible plan keeps the current mode, and the fastest is adopted. (v2 classified
  such plans over the window beyond its first 12 m, so a slide carried into a straight, or a
  corner just behind, forced the mode on and off between corners.)
- **The gate** (`MultiStart.choose`): the fastest feasible plan of each mode competes; the mode
  changes when the other's is faster by more than `threshold` (0.3 %) of the current one's
  window time on `confirm` (2) re-plans in a row, to drift only if its slide starts 25 m or
  more into the window. The car starts in grip, so a tie stays grip. If the current mode has no
  feasible plan (the slide is over, or every plan slides), the mode follows the plans at once:
  a *forced* change.

### E1

- **Courses** (the planner's friction: grip 0.92 times the road's, as live): `chain-1.03`,
  `chain-0.45`, `chain-0.3` -- the study's ten corners (R 9, 15, 25, 40, 60 m at 90 and 180 deg)
  left and right in turn, 120 m straights between, 8 m wide, fall-off 0.8 -- and Haruna (1.03),
  and Haruna with patches of 0.45 and of 0.3 over s 220-560, 840-1060 and 1160-1300 (seven of
  its ten corners, six of them hairpins).
- **Configs**: `single` (060) and `ms0.1`, `ms0.3`, `ms0.5` ({G, D, F}, the threshold in % of
  the window's time), each seed under 0.9 s, its three processes pinned to quiet cores
  (8-10, 11-13); ds 4, the Driver's own window (260 m), budget 1.0 s.
- **The car**: `sim.PlanFollower`, on the adopted plan exactly, stepped at 20 Hz; it starts at
  s 10 at 12 m/s. The course time is the time along the adopted plans (their own prediction).
- **Metrics**: per corner, the mode of every adopted plan that covers it before the car reaches
  its turn-in (changes, flip-flops: a mode left and taken again); the chooser's own switches,
  gate or forced, put on the corner the re-plan was about; the course time against the single
  start; re-plans over 1.0 s; every seed's iterations by its start, and D's along a run of its
  own warm re-plans.

The host was shared (another user's CARLA); v2's seeds ran pinned to cores that were quiet when
measured, at 4.9 ms an iteration over all seed solves (single start 4.2-5.4). v3 ran pinned to
the same cores while others' niced work filled them (load 38-60 on 32 cores): 6.7 ms an
iteration, 5.9-8.9 by run. The single-start baselines are v2's, not re-run.

## Results

v3 (the final code), course time against the single start's, and re-plans over 1.0 s:

| course | single: time (s), overruns | ms0.1 | ms0.3 | ms0.5 |
|---|---|---|---|---|
| chain 1.03 | 90.90, 0 % | +0.61 %, 1.1 % | +0.61 %, 0 % | +0.39 %, 0 % |
| chain 0.45 | 135.85, 7.7 % | -0.26 %, 6.2 % | +0.11 %, 1.5 % | +0.00 %, 0.8 % |
| chain 0.3 | 165.40, 28.9 % | stopped at 44.9 s | +0.88 %, 0.6 % | +0.33 %, 3.8 % |
| Haruna | 76.90, 0 % | -0.07 %, 0 % | -0.07 %, 0 % | -0.59 %, 0 % |
| Haruna 0.45 patches | 99.40, 1.1 % | -0.50 %, 0 % | stopped at 9.3 s | stopped at 9.3 s |
| Haruna 0.3 patches | stopped at 10.4 s (solves failed) | 114.15 s, 0 % | 113.45 s, 1.8 % | 113.60 s, 1.8 % |

Seed solves cut at 0.9 s, v3: 14-23 % on asphalt, 25-61 % on low friction, 78-87 % in the three
stopped runs. The chooser's changes, v3: 83 over the 18 runs (51 forced by the budget, 29 at the
line, 3 by the gate). Corners with more than one change counted as written: chain 0.45 at 0.1 %
(three corners, up to 3), at 0.3 % (one, 2), chain 0.3 at 0.1 % (3), 0.3 % (4), 0.5 % (2),
Haruna at 0.1 % (2), Haruna 0.3 patches at 0.1 % and 0.3 % (3); after the counting ruling only
Haruna 0.3 patches at 0.1 % (s 1411: the previous slide's end forced by the budget, the gate into
the flick's drift at 0.11 %, forced back to grip as the flick's plan fell into grip).

v2 (a corner's mode over the window where none was ahead; the quieter host):

| course | single: time (s), overruns | ms0.1 | ms0.3 | ms0.5 |
|---|---|---|---|---|
| chain 1.03 | 90.90, 0 % | +0.00 % | +0.61 % | +0.61 % |
| chain 0.45 | 135.85, 7.7 % | -0.81 % | -0.44 % | -0.07 % |
| chain 0.3 | 165.40, 28.9 % | +0.33 % | -0.27 % | -0.27 % |
| Haruna | 76.90, 0 % | -0.07 % | -0.07 % | -0.07 % |
| Haruna 0.45 patches | 99.40, 1.1 % | -0.40 % | -0.05 % | -0.05 % |
| Haruna 0.3 patches | stopped at 10.4 s (solves failed) | 115.20 s | 114.50 s | 114.05 s |

Every v2 multi-start run: done, 0 re-plans over 1.0 s.

The chooser's switches over all 18 v2 runs: 2 by the gate (both grip to drift, at 0.1 %), 134
forced (65 drift to grip as slides end, 69 grip to drift where every plan slid). Corners with
more than one change of the chooser's mode: chain 0.3 at 0.1 % (R 60 x 90 deg: grip, drift,
grip), chain 0.45 at 0.3 % (R 15 x 90: two), Haruna's s 276 hairpin at 0.1 and 0.3 % (drift,
grip, drift).

The seeds against G (all v2 multi-start runs; "agree": the same mode in the corner ahead):

| | D ran | agree | D drift, G grip | D grip, G drift | D failed | G failed, D ok | both failed |
|---|---|---|---|---|---|---|---|
| asphalt (chain, Haruna) | 359 | 288 | 24 | 0 | 46 | 0 | 1 |
| mu 0.45 | 517 | 447 | 3 | 1 | 36 | 19 | 11 |
| mu 0.3 | 628 | 444 | 1 | 1 | 65 | 52 | 65 |

(F, into hairpins only, likewise: 70 % agree, 12 % F failed; on asphalt it slid where G did
not in 23 of 166, never faster beyond the threshold twice.)

Iterations (v2), median/p95, every re-plan of the three thresholds pooled (solves in brackets):

| start | asphalt | mu 0.45 | mu 0.3 |
|---|---|---|---|
| G from the adopted plan | 50 / 134 (348) | 94 / 181 (222) | 134 / 180 (314) |
| G its own | 48 / 120 (143) | 90 / 171 (458) | 108 / 177 (502) |
| D fresh seed, Dc | 96 / 186 (104) | 114 / 187 (119) | 146 / 183 (208) |
| D fresh seed, Ds | 165 / 202 (125) | 182 / 205 (30) | 183 / 194 (35) |
| D its own (warm) | 46 / 104 (130) | 96 / 172 (368) | 106 / 176 (385) |
| F fresh / its own | 180 / 202, 46 / 102 | 175 / 209, 90 / 170 | 170 / 203, 97 / 182 |
| single start (060) | 50 / 130 (161) | 90 / 183 (225) | 146 / 194 (169) |

The library's Dc needs fewer iterations than the synthetic Ds wherever both ran (96-146
against 165-183 at the median).

D along its own warm re-plans (the n-th since its seed; median iterations, all thresholds):
asphalt 135-145 at the seed, then 35-60 from the first re-plan on; mu 0.45 110-170 at the
seed, 85-140 at the first to third, 70-100 from the fourth; mu 0.3 140-175, 110-175, 50-110.

## Solver cost (279 live windows on Haruna and Haruna with mu 0.3 patches)

The windows: 060's single start re-solving a second of plan at a time (G: shifted, its
multipliers with it) and, at each hairpin ahead, M1's synthetic rear-out seed from it (Ds, no
multipliers); `studies/solver_cost.py`, each solve on one quiet core.

| | IPOPT (MUMPS, compiled) | Fatrop (compiled, structure detected) |
|---|---|---|
| ms an iteration | 5.9: linear solver 57 %, functions 34 % (Hessian 18, Jacobian 9, gradient 6), IPOPT 9 % | 3.3: Fatrop 0.6, functions 2.2 |
| G iterations p50/p95 | 69 / 241 | 94 / 249 |
| G wall p50/p95/p99 (s) | 0.37 / 1.50 / 2.22 | 0.27 / 0.84 / 1.37 |
| Ds iterations p50/p95 | 179 / 357 | 210 / 379 |
| Ds wall p50/p95/p99 (s) | 1.10 / 2.23 / 2.62 | 0.65 / 1.45 / 1.71 |
| within 0.9 s, G / Ds | 78 % / 30 % | 96 % / 73 % |
| feasible | 275 of 279 | 266 of 279 |
| same plan | | 219 of 263 within 0.3 %; 28 slower, 16 faster beyond it |

Fatrop needed: the gap rows written x_k+1 - F (the window writes F - x_k+1: those rows' sign
flipped, in the problem and in its Jacobian and Hessian), the full Hessian with the
Lagrangian's gradient (IPOPT's library gives the upper triangle), the objective's gradient
alone. `window._compiled_nlpsol` now takes the solver's plugin. Fatrop's expanded SX form (its
own derivatives in CasADi's virtual machine) took 24 ms an iteration (95 % in the functions); a
lower initial barrier (`mu_init` 1e-2) made it slower and failed 2 of 8. Fatrop 1.1.8 has no
multiplier warm start (no such option; CasADi passes none).

## Rulings

- **A plan's mode is decided in the corner ahead, not over its window.** Ruling: classify each
  plan by its slide in the first corner whose turn-in is 10 m or more ahead (from 30 m before
  the turn-in to the exit). Why: the first pass
  classified the whole window (beyond its first 12 m), and on low friction a 260 m window holding
  two or three corners slid in one of them from every start, so both modes had a plan in 0-10
  re-plans a run and the gate never once decided (`m3/window_mode/`). Cost if wrong: a choice
  made per corner can trade a later corner's time for this one's; the window time still
  decides, so only the label is per corner.
- **With no corner ahead, the mode is kept (v3).** Ruling: when no corner's turn-in is 10 m or
  more ahead inside the window, every feasible plan takes the current mode and the fastest is
  adopted; no change, gate or forced, is possible there. Why: v2 classified those plans over the
  window, and a slide carried out of a corner (or into one already begun) read as a mode of its
  own, forcing changes mid-corner and on straights: 136 changes in v2's 1,969 re-plans, 83 in
  v3's 1,680, and Haruna's first-hairpin flip (drift, grip, drift) gone. Cost if wrong: a drift
  whose slide ends badly is held to the end of its corner rather than abandoned at once; the
  plans are still re-solved each second from every start and the fastest feasible one adopted,
  so only the label waits.
- **How changes are counted against "at most one mode change per corner".** Ruling: (1) the
  drift-to-grip as the previous corner's slide ends is that corner's, not the next one's, even
  when the re-plan that makes it is already about the next corner; (2) a change *forced by the
  budget* (the current mode's only plans cut at 0.9 s, so the other mode's was the only plan)
  counts against the budget criterion, not against this one; (3) a change *forced at the line*
  (every feasible plan in the other mode, none cut) counts, as does every gate change. Why: the
  criterion is about the chooser flipping between valleys; (1) is one change per corner booked
  on the wrong corner, and (2) is the host's load (v3 had 51 such changes on saturated cores,
  v2 20). Counted so, v2 passes in all 18 runs and v3 in 17, the exception at the 0.1 % threshold
  only (the gate taking a 0.11 % gain, then forced back at the line); at 0.3 % and 0.5 % both
  pass. Cost if wrong: (2) hides changes a slow host really makes; on such a host the overruns
  and stops already fail the budget criterion, so the gate as a whole still fails there. The
  line-forced changes stay counted, and the classification band (below) is their fix.
- **The budget criterion is judged on v2's quiet cores; v3's timing is reported, not judged.**
  Why: v3's seeds shared their pinned cores with others' work (6.7 ms an iteration against
  v2's 4.9 and the solver study's 5.9 on a quiet core), and M1 ruled timing gates measured by
  iterations or on a quiet core. v2: 0 of 1,969 overruns; v3: 1.5 % and three runs stopped.
  Cost if wrong: the 1 % criterion holds only with an iteration near 5 ms; on a car's computer
  shared with perception, three seeds on three cores at 0.9 s each may not get that, and M4 must
  measure it on the target's load.
- **G stays free (060's planner), so a "grip mode" is G's plan only while G does not slide.**
  Ruling: no held grip seed in M3. Why: the brief's G is the shifted adopted plan, and holding
  it to the tyre's peak is another problem (per-surface bound, a held window per process).
  Cost if wrong: where G slides there is no grip alternative to weigh, which is most of what E1
  saw on low friction; a held seed H (`Window(rear_alpha_max=...)`, per surface) is the fix if
  M4 shows drifts that cost time.
- **Forced changes are not the gate's.** When the current mode has no feasible plan the mode
  follows the plans at once (a slide that is over must end; a grip mode with only sliding plans
  must take one). Why: the alternative is no plan. Cost if wrong: forced changes are where the
  per-corner gate failed (a slide at the 3 deg line read as drift, then grip); a band (drift at
  3 deg, grip below 1.5, the current mode kept between) would remove them, at the cost of a
  second threshold.
- **Each seed is cut at 0.9 s of wall time, an overrun is a re-plan over 1.0 s.** Why: the
  re-plan's latency is the budget (1.0 s); 0.1 s covers the answers' transport (about 20 ms
  measured) and the choice. Cost if wrong: on a loaded core the 0.9 s cut drops seeds (18 % of
  seed solves were cut, most still feasible), which E1's numbers include.
- **"Mu 1.03" is the planner's friction on Haruna's asphalt** (1.12 x grip 0.92), and the
  patches 0.45 and 0.3 likewise the planner's (the road's mu scaled to 0.45/0.92 and 0.3/0.92
  there). Why: the brief's "1.03" is the grip share as live. Cost if wrong: the patches are 8 %
  grippier than M1's mu 0.45 and 0.3 cases.
- **The chain is one course per friction, fall-off 0.8 everywhere.** Why: swb's fall-off; three
  courses and four configs already make 24 runs. Cost if wrong: the 1.0 and 0.6 fall-offs (most
  of M1's drift cases at mu 0.3) are not in E1.
- **F runs in E1** (into hairpins only). Why: half of Haruna's corners are hairpins and M1 found
  the flick the oracle in a tight asphalt hairpin. Cost if wrong: a third core used; F agreed
  with G as D did and was cut more often (fresh flicks 150-210 iterations).
- **The library is M1's Dc plans, not new solves.** Why: they are the fall-off continuation the
  brief asks for, at the study's settings, over its archetypes. Cost if wrong: grip 1.0 and the
  study's entry speed, not the live window's; the mapping corrects the peak and the friction.
- **Fatrop is not adopted for live.** Why: the rule asks for the same plans within noise; 17 %
  of windows differ beyond 0.3 % and Fatrop fails 3 % more, though its p95 is 35-44 % lower.
  Cost if wrong: a faster solver left unused; see below.

## What M4 should take from this

- **Run multi-start for its robustness, not its lap time**: {G, D} (F only into hairpins, or
  not at all) keeps every course going where one start stops, and lands inside the budget, on
  cores that give it about 5 ms an iteration; on saturated cores (v3) it stops as one start
  does. On asphalt it changes nothing measurable; on low friction its gain in E1 is within the
  receding loop's own noise (+-0.9 %), well short of M1's 1-2 % oracle gains. Run M4 at the
  0.3 % threshold: the 0.1 % gate is the only one that took a noise-sized gain and gave it back.
- **To make the choice real, the grip seed must stay in grip**: a held seed (the tyre's peak
  as a bound) beside the free G, so a grip plan always exists to weigh against the slide; and a
  classification band so that slides at the 3 deg line stop flipping the label.
- **D on low friction costs as much warm as cold** (85-120 iterations): budget it as a cold
  solve. On a loaded host, run it every second re-plan or only into corners of R <= 25 m.
- **Fatrop for the seeds, not for G yet**: a fresh seed has no multipliers to lose, and Fatrop
  gets 73 % of them inside 0.9 s against IPOPT's 30 %. Before that, find why 17 % of its plans
  differ (a tolerance mismatch: IPOPT's acceptable level is 1e-3 as its tolerance, Fatrop has
  none; or other local optima), and whether `tol_acceptable` brings them together.
