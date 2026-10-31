# Grip or drift, model in the loop: what M4b left (roadmap 061 M4c)

Written 2026-10-31. M4b (`docs/mode-choice-m4b.md`) did not pass its gate: Tsukuba's R6 not
measurable (060's cut solves at s 1815), ice lost in a third of runs, the zone chain stopping on
ice and at the packed-snow hairpin, the 10 % wrong-map criterion missed by one run of three, and
once the car's estimate lowered into the mud hairpin's slide. M4c takes these up. Harness as M4b
(`studies/mode_choice_m4.py`, `studies/m4_batch.py`; a tick-and-solve trace wrapper and a
window-solve recorder and replayer, kept out of the repository); cores 30, 31, 29, 9 (10, 23 for
the first traces), the host's least loaded. Results: swb `play_log/race_planner/mode_choice/m4c/`
(dry courses in `quiet/`, the ice ablation in `abl/`); configuration tag `m4c2` (all of M4c's
fixes), `m4c` (before the ice gate of issue 005). Every run's configuration was checked in its
result JSON (`front_slip_max`, `rear_slip_max`, `rear_slip_low`, `tag`, `err`, `seed`, `perc`,
`est`) before it was counted.

Load: 1-5 until 12:10; another user's work took the host to 17-39 from 12:15 to 13:05 (batches
waited: timing runs for a 1-min load under 6, robustness under 10), 3-7 after. Each result
records its own load at start and end.

## 1. The estimate lowering in a planned slide (issue 004)

**Cause.** sim_vehicle_ecu's estimator reads the road's friction as the acceleration the car
reaches at the limit, and its limit test is true through a whole drift (sideslip past half the
ESC limit). In a planned slide the axles' forces turn the car as much as they move it: through
the mud hairpin's reversal (s 234-241, plan at 11-18 deg) it read 0.10-0.17 g against the plan's
0.24 g and mud's 0.29 (36 ticks of one re-run). `Driver.mu_scale` believed such a reading
whenever the car slid 2.5 deg wider than the plan; M4b's 0.394 is 0.114 / 0.29, a reading from
that band. Whether it happens is the coincidence of a tracking error with the reversal (the
re-run took no reading).

**Fix.** No reading where the plan itself slides (sideslip >= 5 deg); the estimate still holds
there. An unplanned slide off a grip plan is still read. Test `test_no_reading_in_a_planned_slide`.

**Result.** The chain's mud hairpin, on maps where it is declared right: 1 stop in 52 M4c chain
runs (sched, 10 % seed 5, s 263, without the estimate; M4b: 1 in 26, the estimate's). The estimate
still lowers on ice (true map: 0.44, both sched + estimate runs stopped on ice at s 832-849, M4b
1 of 2): an unplanned slide on ice is read, by design.

## 2. Tsukuba s 1815 (issue 005) and R6

**Cause.** Recorded every window solve of failing runs and replayed them offline (deterministic
to the iteration). At the end of the back straight (50-54 m/s, braking into the R 85-25 sweepers)
a solve needs 197-782 iterations against the 1 s budget's ~160. A cut solve was discarded and
the next started from the last adopted plan, shifted 13 nodes on, its cold tail longer each time
(28, 40, 54, 65 of 75 columns): cut again, four in a row, the car off its last plan's end at s
1820. IPOPT's iterate there is not monotone in feasibility (at 160 iterations 0.003, 164 0.098,
166 0.026, 170 0.09): the wall clock decides. Warm-started from the cut iterate, the next solve
converged in 48-151 iterations; from the stale plan it was cut. Not the multipliers, not the
tolerances; a consistent cold tail (the guess at steady slip) did not move it.

**Fix.** A cut G solve whose rows and shooting gap are within 0.2 warms the next G solve (single
start and the scheduler's G), never adopted (`live.cut_iterate`, `CUT_WARM`). Not on a track with
ice: there the ablation below found it harmful.

**R6, idle host** (`quiet/`, six repeats an arm, one core each, load 1.3-4.4):

| course | arm | times (s) | mean | finished |
|---|---|---|---|---|
| Tsukuba | single | 74.32, 74.10, 74.43, 73.83, 74.10, 74.37 | 74.19 ± 0.22 (sd) | 6 of 6 |
| Tsukuba | sched | 74.43, 74.10, 74.43, 74.43, 74.12, 74.43 | 74.32 ± 0.16 | 6 of 6 |
| Haruna | single / sched | 76.28 / 76.28 | 0.000 % | 1 of 1 each |

M4b: Tsukuba single 2 of 3, sched 0 of 3. Now 12 of 12. The scheduled arm is G alone on every
re-plan (`g_only` 1.0, 1.00 solves a re-plan): the same planner. Its mean is **+0.18 %** over the
single start's, with a standard error of the difference of 0.15 % (0.11 s); the times fall on
three or four discrete values (74.10, 74.32-74.43) whichever arm, as the re-plans' latencies land.

## 3. Ice: the failed re-plans (issue 006)

**Cause.** M4b freed the windows' first three stages of the slip and held rows (the car may be
past a bound), but `Window._result`, which makes a cut solve a plan when it is feasible, still
counted those stages' slip rows: the car's own slip. On ice, where the rear is held at its peak
and the car is past it whenever it slides, cut solves that were plans were thrown away (a solve
IPOPT called successful reported a violation of 4.0). Replay of `ice:9:180:6`'s ten re-plans
under the budget: 1 of 10 a plan with the old check, 4 of 10 with the free stages left out.

The rest is the solver: ice windows (59 stages, 5.5-6.5 ms an iteration) need 200-930 iterations
from the loop's warm starts (some Restoration_Failed or infeasible after 1000-2600), the first
iterate 0.6-3 off in the acceleration rows and the heading's defect at the first stages (the car
off its plan, on the falling branch). Neither IPOPT's adaptive barrier, nor looser acceptable
tolerances, nor dropping the multipliers cut the counts.

**Fix.** `_result` leaves the free stages' slip rows out. Test
`test_the_free_stages_slip_rows_are_not_counted_against_a_plan`.

**Ablation** (ice's 12 corners, single start, one repeat, load 4-5): issue 006's fix and 005's
cut warm start each on and off.

| 006 fix | cut warm (005) | finished | failed re-plans |
|---|---|---|---|
| off | off (M4b) | 6 | 58 / 193 |
| off | on | 4 | 59 / 166 |
| on | on | 6 | 67 / 181 |
| **on** | **off** | **10** | 61 / 214 |

So on ice the cut warm start is off (`Driver.cut_warm`: not where `rear_slip_low` applies).

**Ice corners, M4c's defaults** (`m4c2`, one repeat an arm, load 4.6-6.9; M4b `rl1` load 1.6-3.8):

| arm | M4b finished | M4c finished | failed re-plans M4b / M4c | L1 p50 (s) | time vs single (corners all finished) |
|---|---|---|---|---|---|
| single | 9 | 8 | 49/209, 55/192 | 0.37, 0.44 | |
| sched | 8 | 6 | 49/194, 56/201 | 0.64, 0.78 | -1.2 % (6) |
| always | 6 | 6 | 37/169, 31/176 | 0.88, 0.91 | -2.8 % (6) |

Ice is not improved in the loop: single start 8-10 of 12 in M4c's two single-start measurements,
multi-start 6. Outcomes flip between repeats of one configuration (single: 6, 8, 10 of 12 across
M4c's runs with the same 006 fix); the multi-start arms run their G at the slowest seed's wall
time and lose more re-plans to the budget at the higher load.

## 4. The packed-snow R 25 hairpin (chain s 1238-1249) and packed snow's corners (issue 007)

- **The chain's hairpin:** M4b stopped there in 2 of 8 true-map runs; M4c 0 of 4 true-map multi-
  start runs and none of the 10 % map runs that reached it (8 reached s 1300 or further). M4b's
  sched stop there had 7 failed re-plans and a drift-to-grip switch at s 1236 for want of a drift
  plan, D's cut solves judged infeasible by the free stages' rows: issue 006's fix covers it. One
  M4c run drove it with the steering at its lock for 1.3 s (the car 1.5-5 deg against the plan's
  9.7): still at the limit.
- **Packed snow's corners** (`m4c2`): single 10, sched 10, always 9 of 12 (M4b `a` 10, 10, 9),
  failed re-plans 3, 1, 2 of ~225 (M4b 6-8). The two lost in every arm since M4, R 15 x 90 and
  R 25 x 90 at 15 m, are issue 007: every re-plan solves, the plans slide 6-8 deg, the car's
  slide grows monotonically from the turn-in (10 deg at s 60 to 56 at s 113) as the speed falls
  to a stall or a spin. A tracking matter (060 R3), open.

## 5. The chain: wrong maps with eight seeds, the estimator, the true map

`m4c2`, robustness gate (load < 10): single runs four at a time (load 7.7-9.7, the worst of the
set), multi-start one at a time (2.4-6.9). Zone index as M4b; zone 4 ice s 744-952. The 10 %
maps' wrong zone per seed: s1 4, s2 8, s3 8, s4 7, s5 6, s6 4, s7 9, s8 7.

| map | arm | runs | finished | stops at ice (zone 4) | reached the wrong zone | stopped there | mean distance (m) | solves / re-plan |
|---|---|---|---|---|---|---|---|---|
| true | single | 2 | 0 | 1 | | | 1052 | 1.00 |
| true | always | 2 | **2** (154.1 s) | 0 | | | 2082 | 2.92 |
| true | sched | 2 | **2** (153.7 s) | 0 | | | 2082 | 2.20 |
| true | sched + est | 2 | 0 | 2 | | | 841 | 2.35 |
| 10 % | single | 8 | 0 | 6 | 3 | 3 | 1019 | 1.00 |
| 10 % | always | 8 | 1 | 5 | 4 | 3 | 1127 | 3.09 |
| 10 % | sched | 8 | 0 | 5 | 3 | 3 | 919 | 2.56 |
| 10 % | sched + perc | 8 | 1 | 5 | 4 | 3 | 1112 | 2.39 |
| 10 % | sched + perc + est | 8 | 0 | 4 | 3 | 3 | 1099 | 2.40 |
| 30 % | single / always / sched / s+p+e | 3 each | 0 | | | | 326 / 138 / 330 / 326 | |

(Earlier the same session, before the ice gate, `m4c`-config chains: true map sched and always
2 of 2 each finished, 152.98-154.70 s.)

- **True map: the multi-start arms finish 4 of 4** (M4b 1 of 6 without the estimate); the single
  start 0 of 2 at load 8-10 (M4b 1 of 2 at 1-4).
- **10 % wrong:** most stops are ice's own (zone 4), on seeds whose wrong zone lies beyond it
  (2, 3, 4, 5, 7, 8: the chain up to s 952 is the true map's). The wrong zone's effect: of the
  runs that reached it, every arm stopped there in 3 (sched 3 of 3, always 3 of 4, single 3 of
  3; the zone-6 and zone-7 seeds). By any stop sched 8 of 8, always 7 of 8.
- **Perception and the estimate** change nothing measurable on the 10 % maps (7, 8 of 8 stops,
  the same stops at the wrong zone); the estimate lowered to 0.41-0.52 on ice and the 30 % maps.
- **30 %:** every arm stops early, at the first zone declared grippier than it is (s 75, 260, 640).

## 6. Loose snow, the 14 remaining corners

All 36 loose-snow corner runs re-run on M4c's defaults (load 4.3-7.1): **36 of 36 finished**
(M4b 22 of 22 run). Time vs the single start: sched +2.68 % (12 corners; -0.02 % without
R 9 x 90 at 15 m), always +0.27 %. The exception reproduces M4b's (+29.7 %): `snow_loose:9:90:15`
sched 17.41 s against 13.14 single and 13.61 always; the scheduled arm switches to drift at s 32,
adopts H once, and the car slides to 45 deg against the plan's 23 (always switches too, adopts D
once, and holds 19 deg). One corner, one H adoption: not chased further here.

## 7. The sweep (p_add, gain_min, sd)

Not run: the chain's stops are ice's (zone 4) and the wrong zones', which no scheduler threshold
reaches (the 10 % maps' stops are in zones the scheduler gives G alone or in ice, where it adds
no start), and the host's load went past 10 for an hour mid-session.

## 8. Gate

| criterion | M4c | |
|---|---|---|
| low friction: fewer stops and failed re-plans than single | mud 12/12 (M4b, unchanged code paths but issue 004); loose snow 12/12 all arms; packed snow sched 10, always 9, single 10; ice sched 6, always 6, single 8 | **met except ice** (as M4b) |
| lap time no worse than single on low friction | loose snow sched +2.7 % (one corner; -0.02 % without it), always +0.3 %; packed snow -0.5 / -0.8 %; ice -1.2 / -2.8 % (corners all finished) | met but loose snow's one corner |
| asphalt within 0.1 % (R6) | Haruna 0.000 %; Tsukuba now measurable, 12 of 12 laps: sched +0.18 % ± 0.15 % (se) | **Haruna met; Tsukuba within one standard error, the mean over** |
| sched at least as good as always, fewer solves | as robust or more on every class (ice 6 = 6, packed 10 against 9); 2.2-2.6 solves a re-plan against 2.9-3.1; Haruna 1.00 | met |
| 10 % zones wrong: sched stops no more than always | at the wrong zone: 3 of 3 against 3 of 4 (equal count); any stop 8 of 8 against 7 of 8, ice's own stops 5 each | **met on the wrong zone's stops (ruling)**, missed by one on all stops |
| the zone chain's mud hairpin | 1 stop in 52 chain runs where it is mapped right (no estimate) | fixed but one |

**Verdict: not passed.** Tsukuba is drivable and measurable now, the estimator no longer lowers
into a planned slide, the true-map chain finishes in every multi-start run, and the wrong-map
criterion is met on the stops the wrong zone causes; but ice is still lost in a third to a half
of runs, more so with the multi-start arms (6 against 8 of 12), and that is the robustness the
reframed gate puts first. What ice needs is solver speed on the falling branch (200-930
iterations a window; the R3 solver levers), not a scheduler change.

## Rulings

- **Tsukuba's R6 is judged met within noise, not missed.** +0.18 % on means, 1.2 standard errors,
  with the scheduled arm G alone on every re-plan (the single start's planner, solved in the
  same process); the times take three or four discrete values per arm, as the latency lands.
  Cost if wrong: a 0.1 s scheduler overhead on Tsukuba hidden in the clock's noise (the
  scheduler's scan is outside the measured solve time).
- **The wrong-map criterion is judged on the stops in the wrong zone.** Most 10 % stops (5 of 8
  in each arm) are ice's own on seeds whose wrong zone lies beyond ice, where the chain is the
  true map's; counting them judges ice, not the map. Cost if wrong: the criterion as written is
  missed by one run (8 against 7), as in M4b.
- **The cut warm start is off on tracks with ice** (`rear_slip_low` in effect), on one repeat of
  12 corners (6 against 10 finished). Cost if wrong: a chaotic class read from one repeat; the
  chain (which has ice) runs without it.
- **A cut iterate within 0.2 warms the next solve.** 0.05 took it in one of four recorded runs;
  iterates at 0.09-0.1 converged the next solve. Cost if wrong: a far iterate starting the next
  solve worse than the stale plan (on asphalt, not seen in 12 laps).
- **No estimate reading where the plan slides >= 5 deg.** Cost if wrong: a zone mapped wrong
  where the plan drifts is not read until the car leaves the slide.
- **Single-start chain runs at load 7.7-9.7 are counted** (robustness gate < 10), but their
  0 of 2 true-map finishes are not set against the multi-start arms' 4 of 4 as an arm effect.
- **Issue 007 (packed snow's runaway slide) is left open as an L2 matter** (060 R3).
