# Grip or drift, model in the loop: ice's two remaining losses (roadmap 061 M4f)

Written 2026-11-01. M4e (`docs/mode-choice-m4e.md`) left ice, with issues 007 and 008 fixed, at
9.55 of 12 corners (single start) and two losses: R 9 x 180 at 6 m (L1: windows from the car's
state needing 260-1200 IPOPT iterations against the budget's ~170; 0 of 30 with issue 008's
rows) and the 6 m roads' L2 losses (the car leaving the road 0.4-3.8 m off its plan). M4f's
task: root cause both, fix without regressing ice's other corners. Results: swb
`play_log/race_planner/mode_choice/m4f/` (`rec/` recorded windows, `trace/` traced runs,
`ecu/`, `offroad/`, `front_low/`, `regen_l1_only/` the experiments, `tools/` the scripts and
`m4f-experiments.patch`, the experiments' planner switches, not shipped).

Load: 4-236 from other users through the session (teacup1's CARLA and Python, Rust builds).
Every closed-loop run is repeatable (`RP_DET=1 RP_ITERCAP`), so no number here depends on it.
Every run's configuration (`det`, `itercap`, `jit`, tag) was checked in its result JSON; the
traced re-runs of M4e's losses reproduce M4e's plant digests.

## Result: one cause, the car's braking on ice (issue 009)

Both losses are the same thing (`docs/issues/009-ice-brakes-through-the-rear-axle-only.md`):

1. **The car brakes on ice through its rear axle only.** Its ECU regenerates first, up to
   2812 N at the rear tyres, and adds the friction brakes only beyond that; the rear carries
   about 1630 N on ice. The spec has ABS off, and regen is not cut when the rear locks (the
   plant's rear wheels turn backwards at a -2.4 m/s^2 request). L1 models exactly that, so it
   plans at most 1.1-1.3 m/s^2 of braking against the 2.39 the road allows, 0.46 of mu g, where
   the corners' start speeds assume 0.6.
2. **L1 brakes with the front tyres instead**: slip angles of 30-58 deg on the 6 m roads,
   10-19 times ice's 3 deg peak, steered left and right in turn on the straight (ice's tyre
   keeps 0.7 of its peak sliding, mostly drag there). Issue 002's front rows apply only where a
   tyre does not fall off past its peak; ice does, so nothing bounds it.
3. **L1's loss (R 9 x 180 at 6 m)**: from its start speed the corner is past the car even
   scrubbing. The first plan's rescue ends off the road in 5 of 5 starts (8-15 m outside); IPOPT
   lands in a far (8-12 m) or a near (0.3-0.5 m) compromise of the soft edges by the last bits
   (the first plan from the same start: with issue 008's rows far, without them near; with the
   edge weight at 1000 the other way round: likely M4e's 0 of 30 against 21 of 30, not
   measured further), and the loop adopts the far one (Solve_Succeeded). Every window after
   it is warm-started 8.6 m past the edge rows: 250-1230 iterations, most ending off the road
   again or locally infeasible at the first stages' slope and front-balance rows.
4. **L2's losses**: 15 of the 17 M4e lost with 3 or fewer failed re-plans are on 6 m roads, and
   in each the plan's front slip angle in the last 3 s was 11-58 deg. L2 follows a front far
   past its peak, where steering barely changes its force, and has no lateral authority left:
   its own prediction is 0.6-3.0 m past its edge rows (weighted 1e4 a m^2) when the car leaves.
   Not an edge margin and not LINE_MU's line weight (the edge rows dominate). On 15 m roads the
   same plans' errors fit.

### Measurements asked for (L2)

| | |
|---|---|
| L2's edge | soft, at the road's edge less half the car and 0.1 m (`EDGE_MARGIN`), 1e4 a m^2 plus 1e3 a m; L1's plans keep 0.3 m inside that (`margin`), the run ends 0.5 m past it |
| how close plans run | at the 1.75 m bound in 3 of the 15 losses, past it in 3 (2.1-2.9 m), 0.9-1.6 m from the centre in 9 |
| n error at the loss | 0.4-3.3 m, 7 inside, 8 outside; the car leaves where the plan's front is at 11-58 deg |
| the loss | ice:15:180:6: plan 54-57 deg on the straight, car 1.3 m inside and off the inside edge, L2 at the lock; ice:25:180:6: 37-39 deg, then the rear 12 deg past the plan's 2 |

## Candidates

All repeatable runs of ice's 12 corners, single start; M4e's both-fixes cell (`78`) on the same
replicates (budget in iterations, last-bit draw `--jit`) for comparison.

| candidate | ice, finished | the 6 m roads | target corners | |
|---|---|---|---|---|
| M4e `78` (main), 170 x2, 150, 200 | 9.50 of 12 (10, 9, 9, 10) | 14 of 24 | R 9 x 180 / R 15 x 180 at 6 m: 0 / 0 of 4 | |
| **the car's regen within its rear's grip** (plant ECU capped at 1100 N, L1 told: a car change) | **10.50** (12, 11, 10, 9): +1.0, se 0.71 | **22 of 24** | **3 / 4 of 4** | 15 m roads 20 of 24 (against 24); front 4-18 deg in its plans |
| a plan off the road not adopted (`Off_Road_Plan`: its car > 0.5 m past the edge after 3 nodes), 170 x2 | 19 of 24 (M4e 19) | | R 9 x 180: 0 of 2 (the car then starts 10 % slower: still fails in the hairpin) | 20 of 24 runs bit-identical to M4e's, the other 4 the same outcome |
| the front's slip rows on ice (1 or 2 peaks), two corners | 1 of 8 | | 0 / 1 of 4 | re-plans fail 7 of 7: no brake left |
| L1 told regen stops at the rear's grip, the car not (0.7, 0.9), R 9 x 180 at 6 m | 1 of 8 | | | the plant's rear locks (ABS off): spins on the straight |

- **The lever is the car's braking**: with regen within what the rear carries, the plans stop
  scrubbing and the 6 m roads go from 14 to 22 of 24, both target corners from 0 to 3-4 of 4.
  Overall +1.0 corner of 12 is 1.4 se (the crude cap costs 4 of 24 on the 15 m roads' R 9 and
  R 15 x 180): the cap is a size, not a design.
- **Nothing in the planner fixes it.** Bounding the front takes away the plans' only brake;
  rejecting off-road plans is right in principle but the corner's windows fail anyway.

## Gate

| criterion | M4f | |
|---|---|---|
| ice single and sched not worse pooled, the target corners better | nothing shipped to the planner; the car experiment: single +1.0 (se 0.71), target corners 0 -> 3 and 4 of 4 | **not met** (the fix is not the planner's) |
| Haruna / Tsukuba `RP_DET` digests 76.28 / 74.33 s | planner unchanged (main 95b902e) | met trivially |
| packed / loose snow, mud not worse | planner unchanged | met trivially |
| suite | planner unchanged | not re-run |

**Verdict: root caused, not fixed.** Both of ice's remaining losses are issue 009: the drift
car brakes on ice through its rear only (regen first, ABS off), L1 brakes with its front tyres
far past their peak instead, and on 6 m roads neither L1's windows nor L2 can live with that.
The fix is the car's or the study's (ruling below).

## Rulings

- **No planner change shipped.** Every planner-side candidate was worse or equal; the one that
  works changes the car. Cost if wrong: an L1 change that both keeps a brake and bounds the
  front (e.g. a front bound at several peaks only on the straight) is untried.
- **The off-road plan check not shipped** although right in principle (an adopted plan 8-12 m
  off the road is a plan to crash): 19 of 24 as M4e's, the target corner 0 of 2; it also
  makes the study's 10 % slower restart the usual outcome there, which changes the experiment.
  Cost if wrong: a loop that adopts a plan off the road elsewhere (not seen on asphalt).
- **The ECU experiment as evidence, not a fix**: a fixed 1100 N cap tuned to ice; a real one
  would follow the friction (the ECU's estimate) or let ABS work the motor. Its 15 m losses
  (4 of 24) are not explained.
- **One draw per budget** for the car experiment (4 replicates, 48 runs) where M4e drew 20: the
  6 m roads' 22 against 14 of 24 is far beyond the draws' spread; the overall +1.0 is not.

## Needs a ruling (user)

1. **The car**: regen limited on low friction (sim_vehicle_ecu: by its friction estimate, or
   ABS acting on the motor's torque; and `abs_enabled` in the drift spec), with `axle.brakes`
   following it. Roadmap 055's repository, and Isaac's car runs the same ECU.
2. **Or the study**: ice's start speeds from the car's own braking (0.46 mu g, rear-only)
   instead of `v_start`'s 0.6. Changes every ice comparison since M4.
3. **Or accept**: the 6 m ice roads as beyond this car at these speeds (ice single 9.55 of 12).

## Left

- Ice with the car's braking fixed (1 or 2 above): re-measure single/sched/always, then the 15 m
  R 9 / R 15 x 180 losses the crude cap brought.
- The hairpin's windows from the car's state at its rear peak (177-1549 iterations even started
  slower): M4d's "left", still open.
