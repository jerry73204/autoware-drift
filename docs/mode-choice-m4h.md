# Grip or drift, model in the loop: the gate re-scored (roadmap 061 M4h)

Written 2026-11-01. M4c (`docs/mode-choice-m4c.md`) scored roadmap 061's gate and did not pass
it, ice first. Since then issue 007's fix (L2's line weight on low friction, 14a79e7) and issue
008's (L1's slip rows in the surface's peak, 9771e96/6cf5108) landed, and M4e measured ice with
both. M4h re-scores the whole gate on that code: autoware-drift **95b902e**, from a worktree
pinned there (main moved on during the session: 065 M2's S2, d7b6dac's `--traj`; none of it is
in these numbers; see "Since 95b902e"). Named M4h, not M5: the roadmap's M5 is the GPU mode
probe; M4f and M4g are ice's, landed during this session.

Results: swb `play_log/race_planner/mode_choice/m4h/`: `r6/` the dry courses' wall-clock
laps, `digest/` the dry courses under `RP_DET`, `det/` the corner suites' repeatable runs,
`corners/` the wall-clock corner runs, `chain/` and `chaindet/` the class-zone chain (wall clock
and repeatable), `traj_runs/` one representative run per arm and course or class with `--traj`
for `tools/arm_video.py`. Every result has a gzipped trajectory beside it in its directory's
`traj/` (the plant's state per tick and the driver's log).

## Method

- **Pinned inputs.** The planner from the 95b902e worktree (each run asserts it imported that
  tree); swb's inputs from a frozen copy (`surface_classes.yaml`, `drift_mustang.yaml`,
  sim_vehicle_ecu at d68637a, the Haruna and Tsukuba course data), so the superproject's
  moving submodules (sim_vehicle_ecu f3f5dcf, issue 009's regen control, landed during the
  session) do not change a run. Check: Haruna and Tsukuba `RP_DET` single 76.28 and 74.33 s,
  plan and plant digests equal to M4e's (9524862c / 9160b94d, d7ca9f95 / 82bd3c28).
- **Harness.** `studies/mode_choice_m4.py run` behind a thin wrapper that records the
  trajectory and nothing else (checked: digests unchanged); arms single, sched, always, with
  perception and the estimate on the chain, as M4c. At most four of my processes at a time, on
  cores 28-31; a multi-start run alone on all four.
- **Wall clock** (as M4c): a dry-course timing run started only when the host's 1-min load was
  under 6 with nothing else of mine running, each run's load recorded at its start and end;
  robustness runs under 10.
- **Repeatable runs** (as M4d, M4e): `RP_DET=1 RP_ITERCAP` 150-200, each budget a draw, with a
  last-bit start (`--jit` 5e-9 to 8e-9, draws not used by M4e). Outcomes compared pooled, mean
  and Welch standard error; a difference under 2 se is noise. For ice M4e's both-fixes runs
  (`m4e/det`, tag `c78`: the same planner, the fixes on) are pooled with M4h's.
- **Load.** 4-11 until 10:40, then other sessions' builds and studies and another user's CARLA
  took the host to 15-140 for most of the afternoon (16:00-16:40 under 10). The dry timing ran
  in the morning's window; the corner suites and the chain are judged on repeatable runs, with
  wall-clock runs where the load allowed.
- Every run's configuration (`det`, `itercap`, `jit`, `tag`, `rp_root`, `rear_slip_low`,
  `style`, `beta_bound`, `rules`, `sd`, and the absence of a study patch) was checked in its
  result JSON before it was counted.

## Gate

| item | target | M4c | M4h (95b902e) | |
|---|---|---|---|---|
| R6 Haruna | sched within 0.1 % of single | 0.000 % (1 lap each) | wall clock 6 laps an arm (load 3.9-7.5): single 76.28 x5, 76.27; **sched 76.33 ± 0.06, +0.063 % (se 0.033 %)**; always +0.19 %; `RP_DET` sched = single bit for bit (G alone, 1.00 solves) | **met** |
| R6 Tsukuba | the same | +0.18 % (se 0.15 %), 12 of 12 laps | single 74.24 ± 0.11 (6 of 6); **sched 74.24 ± 0.48, -0.001 % (se 0.30 %), 5 of 6** (one off road at s 1820, issue 005's place, 4 failed re-plans at load 6.2); always 5 of 6, +0.004 %; `RP_DET` sched = single bit for bit | **met on time**; one lap of six lost (below) |
| low friction: stops, multi-start against single | fewer stops and failed re-plans | mud 12/12/12, loose 12/12/12, packed 10/10/9, ice 8/6/6 of 12 (single/sched/always) | ice pooled 9.52 (25 replicates) / 9.58 (12) / 9.14 (7), failed re-plans 19 / 15 / 15 %; mud 11 / 11 / 11 (repeatable and wall clock: `mud_wet:15:180:6` lost in every arm and draw, issue 012); gravel 12 / 12 / 12; packed snow single 12 (repeatable), sched / always 12 / 12 (issue 007's batch on the same code); loose snow 12 / 12 / 12 (repeatable) | ice met within noise (sched - single +0.06, 0.2 se); **mud lost a corner in every arm (issue 012)** |
| low friction: time against single | no worse | loose +2.7 % (one corner), packed -0.5 %, ice -1.2 % | mud -0.42 %, gravel -0.32 %, ice +0.05 % (8 corners every arm finished, repeatable); loose snow sched +1.43 %, always +2.64 % (12 corners, medians +0.01 / +0.05 %; sched's all from R 15 x 180 at 6 m, +15.6 %: one H and one F adoption, the car to 36 deg against the plan's 18); wall clock ice sched +1.27 % (7) | met but loose snow's one corner (as M4c) |
| sched at least as good as always, fewer solves | | met (2.2-2.6 against 2.9-3.1) | ice 9.58 against 9.14 (+0.44, 0.8 se), mud and gravel equal; solves a re-plan ice 2.00 / 2.35, mud 2.20 / 2.28, gravel 1.07 / 2.24, chain 2.4-2.5 / 3.1, Haruna 1.00 / 3.27 | **met** |
| the true-map chain finishes | (M4c's state) | sched 2/2, always 2/2 | **0 of 20**: wall clock 0 of 8 (single, sched, sched + est, always, 2 each), repeatable 0 of 12 (4 draws an arm); stops at loose snow R 9 x 90 (s 1075) 5, packed snow R 25 x 180 (s 1205) 6, ice 5, mud 3, loose snow R 60 (s 481) 1 | **not met (issue 012)** |
| 10 % zones wrong: sched stops no more than always | | at the wrong zone 3 of 3 against 3 of 4 | every run of every arm stops (wall clock 40, repeatable 24, 8 seeds); at the wrong zone sched 3 of 3 reached, always 2 of 2, in both; most stops in the chain's true-mapped zones 4-6 | **not testable** while the true map fails (issue 012); by the letter missed by one, as M4b |
| the chain's mud hairpin, mapped right | | 1 stop in 52 | repeatable 0 of 36; wall clock 10 of 56 (sched and sched + perc 7, load 6-10) | not met on the wall clock; see below |

## Findings

### 1. Issue 012: the line weight that fixed packed snow's corners costs the chain and a mud hairpin

New, `docs/issues/012-line-weight-costs-the-chain-and-a-mud-hairpin.md`. On the true-map chain nothing finishes on 95b902e (20 runs,
every arm; M4c's multi-start 4 of 4). Of the repeatable runs that reach s 1050, 9 of 9 stop:
the loose-snow R 9 x 90 (s 1075; the plan slides 15-21 deg, the car loses its slide and swings
across the 8 m road) or the packed-snow R 25 x 180 (s 1205; the car at 3-5 deg against the
plan's 8-14, the steering at its lock, drifting 1-2.5 m out of the plan). The same 12 draws with
issue 007's fix off (`tracker.LINE_MU = 0`, M4e's study patch): 4 finish (single 1, sched 1,
always 2), and of those reaching s 1050, 0 of 4 stop. And `mud_wet:15:180:6`, finished in M4b
and M4c, now spins at s 82.3 in every arm and draw (car 23 -> 59 deg against the plan's 23);
with the weight off it finishes 3 of 3. Issue 007's fix was measured on the snow corner suites,
not on mud or the chain; its corners (packed snow's R 15 and R 25 x 90 at 15 m) are what it won.
Suspected, not proven: the line, loosened to 0.2 m / mu, gives way before L2 asks for the
rotation the plan's slide needs, and the 8 m and 6 m roads' margins run out (M4f's finding on
ice's 6 m roads, a front past its peak with no authority left, may be the same mechanism on
packed snow, where the front sits at its lock). Not fixed here.

### 2. Ice: no longer the gate's blocker, still a quarter of its corners lost in every arm

Repeatable, pooled with M4e's both-fixes runs: single 9.52 of 12 (25 replicates, sd 0.82),
sched 9.58 (12, sd 1.00), always 9.14 (7, sd 1.35); sched against single +0.06 (se 0.33),
always against single -0.38 (0.53): the extra starts neither lose nor win corners on ice, and
fewer re-plans fail (15 % against 19 %). M4h's own draws alone: single 9.40, sched 8.50 (2),
always 7.50 (2); wall clock (one repeat, load 5-13): 9, 9, 11. M4c: 8, 6, 6. By the gate's
letter (fewer stops and failed re-plans than single) sched is equal on stops and better on
re-plans. What ice still loses is M4f's (issue 009: the car brakes through its rear axle only;
sim_vehicle_ecu f3f5dcf changes it and ice should be re-scored on it).

### 3. R6

Haruna: sched +0.063 % (se 0.033 %), 6 of 6 laps an arm, inside 0.1 % on the mean and within 2
se of zero; the sched laps over 76.28 are the ones started at the highest load (4.9-7.3), and
under `RP_DET` sched and single are the same plans and plant bit for bit. Tsukuba: equal means
(-0.001 %), but one sched lap of six off road at s 1820 (4 failed re-plans, 4 overruns,
load 6.2) and one always lap at s 1801: issue 005's place, reached when solves are cut at the
budget; single 6 of 6 (M4c 12 of 12 across both arms). Under `RP_DET` sched = single bit for
bit, so the lap is the clock, not the scheduler.

### 4. The wrong maps

Every 10 % run stops (wall clock: single, sched, sched + perc, sched + perc + est, always, 8
seeds; repeatable: single, sched, always, 8 seeds), most of them in the chain's true-mapped
zones 4-6 that issue 012 (and ice) stop on the true map too. Sched stops no more often than
always where the wrong zone is reached (3 of 3 against 2 of 2, both measurements), but no arm
gets far enough often enough for the criterion to mean anything. Perception and the estimate
change nothing measurable (mean distance 600-680 m against sched's 760).

### 5. The mud hairpin on the wall clock

10 of 56 wall-clock chain runs where the mud hairpin is mapped right stop there (s 242-277),
7 of them sched or sched + perc on 10 % maps at load 6-10; none of 36 repeatable runs do
(M4c 1 in 52 on the wall clock). A latency effect under load is the likely reading (the
repeatable runs adopt every plan at the fixed latency); not chased.

## Verdict

**061 does not close.** R6 is met (Haruna +0.06 %, Tsukuba equal; sched is single bit for bit
on both), ice is no longer a gate failure, sched is as good as always with fewer solves. It
fails on robustness, which the reframed gate puts first: the true-map chain, finished by every
multi-start run in M4c, is finished by none, and a mud corner is lost in every arm; both trace
to issue 007's line weight (issue 012). The 10 % wrong-map criterion cannot be judged until the
true map finishes.

Blockers, by weight:
1. **Issue 012** (L2's low-friction line weight): the chain 0 of 20, `mud_wet:15:180:6` lost in
   every arm. Turning the weight off restores both on the same draws but brings back issue
   007's corners; a weight that also respects the road's margin (or the car's lateral
   authority in the horizon) is the open question.
2. **Ice's absolute losses** (2.5 of 12 a run in every arm; issue 009, the car's braking):
   not a gate failure (the gate compares arms), but M6's "no spin stops" will not pass; re-score
   on sim_vehicle_ecu f3f5dcf.
3. **The wrong-map criterion** waits on 1.
4. **Tsukuba's s 1820 under load** (1 of 6 sched laps, issue 005's place): the wall clock's;
   single 6 of 6. Watch, not a blocker on its own.

## Rulings

- **Milestone named M4h, not M5**: the roadmap's M5 is the GPU mode probe; M4f and M4g are ice's.
  Results moved from `m5/` to `m4h/`. Cost if wrong: a name.
- **Corner suites and the chain judged on repeatable runs** (load 15-140 most of the session);
  wall-clock runs where the load allowed agree (ice 9/9/11, mud's corner lost in every arm; the
  wall-clock corner batch stopped at ice and mud, 64 of 180, when the load stayed over 10).
  Cost if wrong: a latency effect hidden, as the mud hairpin's (finding 5).
- **Ice pooled with M4e's both-fixes runs** (identical planner: 95b902e adds documentation and
  `--jit` to 6cf5108). Cost if wrong: none found; M4h's own draws are inside M4e's spread.
- **Packed snow's multi-start arms cited, not re-run** (issue 007's batch, 14a79e7: 12 / 12;
  issue 008 changes ice and sand only, M4d checked packed snow's runs equal), the single start
  re-run (12 of 12); **asphalt's corners not re-run** (L2 and the planner bit for bit there
  since M4c; the representative runs finish in every arm). Cost if wrong: a packed-snow
  multi-start loss unseen.
- **R6 on the wall clock with runs started under a 1-min load of 6**, some reaching 7.5 during the
  lap (each recorded). Cost if wrong: sched's +0.063 % on Haruna is that load (its 76.28 laps
  were the lower-load ones).
- **Issue 012's cause called suspected**: the line weight decides on the same draws (0 of 9
  against 0 of 4 stops past s 1050; 0 of 5 against 3 of 3 on the mud corner), but the
  mechanism (line against rotation, or M4f's front past its peak) is not separated.

## Representative runs (for `tools/arm_video.py`)

`traj_runs/<key>.traj.json` (d7b6dac's `--traj`; its planner is 95b902e's, d7b6dac adds the
recorder and `serve.py`; every one checked equal to the 95b902e run it repeats: outcome, time,
re-plans, and on the dry courses the plan and plant digests), all `RP_DET`:

| course / class | corner, result name | single | sched | always |
|---|---|---|---|---|
| Haruna | lap, `haruna__<arm>_r1_dig` | 76.28 s | 76.28 (= single bit for bit) | 77.07 |
| Tsukuba | lap, `tsukuba__<arm>_r1_dig` | 74.33 | 74.33 (= single) | 73.98 |
| ice | R 15 x 180, 6 m, `ITERCAP` 150, `c_ice_15_180_6__<arm>_r1_d150j5` | 23.10 | 22.02 (H x 3) | 22.02 |
| wet mud | R 9 x 180, 15 m, `c_mud_wet_9_180_15__<arm>_r1_d170j5` | 16.97 | 16.97 | 16.97 |
| gravel | R 9 x 180, 15 m, `c_gravel_9_180_15__<arm>_r1_d170j5` | 11.97 | 11.76 (D x 2) | 11.76 |
| packed snow | R 9 x 180, 15 m, `c_snow_packed_9_180_15__<arm>_r1_d170j5` | 21.77 | 20.48 (H x 3) | 22.58 |
| loose snow | R 9 x 180, 15 m, `c_snow_loose_9_180_15__<arm>_r1_d170j5` | 18.00 | 18.04 (H, F) | 18.07 |
| asphalt | R 9 x 180, 15 m, `c_asphalt_9_180_15__<arm>_r1_d170j5` | 8.88 | 8.88 | 8.88 |
| the chain, true map | no clean run exists (issue 012): the furthest draw an arm, `zones__single_r1_d185j6`, `zones__sched_r1_d200j8`, `zones__always_r1_d150j7` | off road s 1746 | off road s 1205 | off road s 1208 |

## Since 95b902e

Main moved during the session: M4g (sim_vehicle_ecu f3f5dcf, regen held within the driven
axle's grip; the planner's hold on ice, 0c61d28/474a9bf) takes ice's single start to 10.90 of
12; issue 011 (the new ECU leaves two wet-mud corners with an entry straight); 065 M2-M3 (S2).
None of it touches L2's line weight, so issue 012 stands on main; ice's rows above are the old
car's and should be re-scored with the gate after issue 012.
