# Grip or drift, model in the loop: ice with both fixes (roadmap 061 M4e)

Written 2026-11-01. M4d (`docs/mode-choice-m4d.md`) measured issue 008's fix (L1's slip-ratio
rows in the surface's own peak) on L2 before issue 007's fix (L2's line weight on low friction),
and found ice's single start at 32 of 60 with it against 40 without. M4e: one ice batch with
both fixes, 2 x 2 (007 on/off x 008 on/off), to decide (a) whether 008's fix stays and (b) how
much of ice's gap is L2's tracking. Results: swb `play_log/race_planner/mode_choice/m4e/`
(`det/` the repeatable runs, `wall/` the wall-clock runs, `digest/` Haruna and Tsukuba).

## Method

- **Cells.** `00` neither fix (M4c's code), `07` issue 007's only, `08` issue 008's only (M4d's
  shipped form), `78` both (main). One tree (main, 6cf5108); the fixes turned off at import by a
  study-only patch, not shipped: `tracker.LINE_MU = 0` (the line weight then equals the constant
  one and L2 runs the constant-weight QP, as before 14a79e7) and `Window(kappa_by_peak=False)`
  (the rows absolute, as before 9771e96). Checked: `00` and `08` reproduce M4d's M4c-code and
  issue-008 runs to the result (6 corners at 170 iterations), and both fixes on gives main's
  Haruna and Tsukuba `RP_DET` digests (76.28, 74.33 s, plan and plant digests equal to M4d's).
- **Repeatable runs** (`RP_DET=1 RP_ITERCAP`), ice's 12 corners, the budget as 150, 160, 170,
  185 and 200 iterations a second, as M4d.
- **Draws.** Ice's corners are chaotic: M4d's two compiled forms of the same rows finished 19
  and 28 of 36. So each budget's run is one draw, and M4e draws more: the start speed times
  (1 + k x 1e-9), k = 1, 2, 3 (now `--jit` in `studies/mode_choice_m4.py`). Every such run's
  plant digest differs from the k = 0 run's, and 18 % of outcomes flip. A replicate is the
  12 corners at one (budget, k); single: 20 replicates for `07` and `78` (k 0-3), 15 for `00`
  and `08` (k 0-2); sched: 10 (k 0-1) and 5 (k 0); always: `78` only, 5.
- **Noise.** Reported per cell: corners finished per replicate (mean, sd, range); differences
  with the standard error of the difference of the means (Welch). A difference within 2
  standard errors is read as noise. Replicates are not independent in every respect (the same
  12 corners; budgets near each other), so the standard error is if anything optimistic.
- **Cause of a lost corner.** As M4d: a run lost with 4 or more failed re-plans is L1's (its
  plans failed); with 3 or fewer, L2's (the plans were there, the car left them or the road).
  Every run's configuration (`det`, `itercap`, `line_mu`, `kappa_by_peak`, `jit`, `arm`) was
  checked in its result JSON by the aggregation before it was counted.

## Results

**Single start** (finished of 12 a replicate):

| | 008 off | 008 on | 008's effect |
|---|---|---|---|
| **007 off** | `00` 6.87 (sd 1.19, 5-9; 103 / 180); failed re-plans 28 % | `08` 6.27 (sd 1.28, 4-8; 94 / 180); 24 % | -0.60, se 0.45 |
| **007 on** | `07` 9.70 (sd 1.26, 8-12; 194 / 240); 22 % | `78` 9.55 (sd 0.69, 8-11; 191 / 240); 19 % | -0.15, se 0.32 |
| 007's effect | +2.83, se 0.42 | +3.28, se 0.36 | |

**Sched** (multi-start, scheduled):

| | 008 off | 008 on | 008's effect |
|---|---|---|---|
| **007 off** | `00` 7.60 (sd 1.52, 5-9; 38 / 60); 27 % | `08` 6.40 (sd 0.55, 6-7; 32 / 60); 24 % | -1.20, se 0.72 |
| **007 on** | `07` 10.40 (sd 0.97, 9-12; 104 / 120); 17 % | `78` 9.80 (sd 0.92, 8-11; 98 / 120); 15 % | -0.60, se 0.42 |
| 007's effect | +2.80, se 0.74 | +3.40, se 0.38 | |

**Always**, both fixes: 9.80 (sd 0.84, 9-11; 49 / 60), failed 15 % (M4d at 170 iterations,
without 007: 6 of 12 with either rows).

- **Issue 007's fix is ice's lever**: +2.8 to +3.4 corners of 12 in every cell and arm, 7-9
  standard errors. Ice single 9.55 of 12 with both fixes, sched 9.80, always 9.80, against
  M4c's 8, 6, 6.
- **Issue 008's fix is not worse beyond noise** with 007's on: single -0.15 (se 0.32), sched
  -0.60 (se 0.42). Without 007's it was -0.60 (single) and -1.20 (sched), also within 2 se:
  M4d's 32 against 40 of 60 was the draw (with more draws `00` is 6.87, not 8.0, of 12). With
  both fixes the finishes vary less (sd 0.69 against 1.26, single) and fewer re-plans fail
  (19 % against 22 %; sched 15 against 17 %).
- **One corner is not chaos.** R 9 x 180 at 6 m: with 008's rows 0 of 20 (single) and 0 of 10
  (sched) with 007's fix, 0 of 15 without; without the rows 12 of 20, 9 of 10 (5 of 15 without
  007). Every loss there is L1's (every re-plan from the start fails at the budget). Replayed
  (one run's 11 windows, unbudgeted to 3000 iterations): with either rows the windows need
  263-1221 iterations where they converge (5 of 11 either way), the rest end infeasible or in
  restoration; none within 170. The corner is beyond L1's budget with both forms; without
  008's rows a few re-plans land just under it (one run: 3 of 12 at 152-162 iterations), and
  some runs finish on those. Not 008's bug; the same windows as M4d's "left" (non-converging
  from the car's state). 008's rows win back as much elsewhere: R 9 x 90 at 6 and 15 m (17 and
  19 of 20 against 14 and 16), R 9 x 180 at 15 m (17 against 9), R 15 x 180 at 6 m (15
  against 11); it loses some on R 15 x 90 at 6 m (11 against 17) and R 25 x 180 at 6 m (13
  against 17).

**Lost corners by cause** (single; L1: 4 or more failed re-plans, L2: 3 or fewer):

| cell | lost of runs | L1 | L2 | lost a run (of 12): L1 + L2 | on the 6 m roads |
|---|---|---|---|---|---|
| `00` | 77 / 180 | 42 | 35 | 2.80 + 2.33 | 40 of 77 |
| `07` | 46 / 240 | 25 | 21 | 1.25 + 1.05 | 31 of 46 |
| `08` | 86 / 180 | 38 | 48 | 2.53 + 3.20 | 37 of 86 |
| `78` | 49 / 240 | 32 (20 of them R 9 x 180 at 6 m) | 17 | 1.60 + 0.85 | 44 of 49 |

- **How much of ice's gap was L2's**: before issue 007 (`00`) a run lost 5.1 of 12 corners,
  2.3 of them by the L2 criterion. Issue 007's fix alone won back 2.8 (single and sched): more
  than the L2 count, because a car that stays on its plan also starts L1's windows from where
  they can be solved (failed re-plans 28 -> 22 %, sched 27 -> 17 %). So about 55-65 % of ice's
  gap before M4e was L2's tracking, directly or through the windows it handed L1.
- **What is left** (both fixes, 2.45 of 12 a run): L1 1.6, 1.0 of it the one corner above; L2
  0.85, on the 6 m roads (15 of 17), the car 0.4-3.8 m off its plan when it leaves the road (no
  runaway slides: the car's sideslip at most 1-36 deg in 15 of the 17). The 15 m roads are
  nearly done: 115 of 120 runs finished.

**Wall clock** (single, both and `07`, started under a 1-min load of 6; loads during runs
4.3-7.3): `78` 10 and 11 of 12 (21 of 24, failed re-plans 17 %, solve p50 0.24-0.33 s); `07`
12 and 9 of 12 (21 of 24, 23 %). M4c 8 of 12, M4d (no 007) 15 of 24.

**No regression elsewhere**: the planner is unchanged (both fixes are on main); Haruna and
Tsukuba `RP_DET` single 76.28 and 74.33 s, plan and plant digests equal to M4d's.

## Decisions

- **(a) Issue 008's fix stays.** By the rule set for M4e (kept unless, with 007's fix on, it is
  worse pooled beyond noise): -0.15 (se 0.32) single, -0.60 (se 0.42) sched; not beyond noise.
  Cost if wrong: up to ~1 corner of 12 on ice's multi-start arms (sched's -0.60 is 1.4 se),
  of which R 9 x 180 at 6 m is 0.9.
- **(b) L2's tracking was most of ice's gap** (issue 007's fix: +2.8 to +3.4 of 12, 55-65 % of
  the corners M4c lost); what is left is half L1's budget on one hairpin and half the car
  leaving 6 m roads 1-4 m off its plan.

## Rulings

- **Draws by a last-bit start** (`--jit`, 1e-9 of the start speed) stand in for repeats of a
  repeatable run. Cost if wrong: the draws are not independent of the corner's geometry, so the
  standard errors understate the spread a little; no difference called here is near 2 se.
- **Fixes toggled by a study-only patch in one tree**, not by older commits: checked equal to
  M4d's runs of those commits (6 of 6) and to main's digests. Not shipped: both fixes stay, so
  an off switch has no use.
- **No L2 follow-up tried.** The L2 losses left are on 6 m roads with the car 0.4-3.8 m off its
  plan, not slides running away (issue 007's pattern); nothing there points at xi's scale, and
  no cheap, clearly indicated change was found. Not written as an issue.
- **`always` measured with both fixes only** (5 budgets): it is not part of decision (a).

## Left

- R 9 x 180 at 6 m on ice: its windows need 260-1200 iterations from the car's state with
  either rows (L1, the budget; M4d's "windows that start from the car off its plan").
- The 6 m roads: L2 leaves the road 0.4-3.8 m off the plan on R 15 x 90, R 15 x 180 and R 25 x
  180 (the edge's margin against the line's looser weight on ice: not measured).
