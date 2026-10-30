# Grip or drift, model in the loop (roadmap 061 M4)

Written 2026-10-30 (part A) and 2026-10-31 (part B). M3b's scheduler picks L1's starts from the window ahead; M4 drives it on
`sim.Plant` behind the ECU (sim_vehicle_ecu) with L2 (the NMPC) tracking, against the single start
(060) and every start always on, on a true map and a wrong one. First, part A: the asphalt rule
tightened (user, 2026-10-30: "we don't pursue drift start for most dry asphalt roads"; approved
"tighten the asphalt rule").

## A. The asphalt rule tightened

**Rule.** On a class that falls off past its peak (asphalt, concrete, gravel, dirt, grass,
packed snow: sliding share below 0.95), a corner gets D (and with it H) only where the drift
map's gain for it (its R, turn and width, as the map is read: a paying cell no wider and turning
no less than 30 deg further, up to 1.25 x its radius) clears the switch threshold, 0.3 %
(`RULES["gain_falloff"]`), not the noise floor (`gain_min`, 0.05 %). Classes without fall-off
(mud, sand, loose snow) keep 0.05 %. H on low friction (the fallback) is unchanged. The camera's
veto reads the map the same way. On asphalt only the R 9 cells clear it (0.32-0.48 % at 6-15 m),
so hairpins up to R ~11 m; gravel's R 9 hairpins (0.44-0.61 %) and its 15 m-wide R 15 (0.35 %);
packed snow's R 9 hairpin (0.34 %).

**Chain check** (`studies/mode_choice_m3b.py`, no plant, three repeats each; cores 8-11 pinned,
host load 4.9-6.3 on 32 cores; results swb `play_log/race_planner/mode_choice/m3b/tightened/`):

| course | config | result (r1/r2/r3) | time (s) | vs single | solves / re-plan | G only | overruns > 1.0 s |
|---|---|---|---|---|---|---|---|
| Haruna | single | done x3 | 76.90 x3 | | 1.00 | 100 % | 0/0/0 |
| Haruna | sched (M3b) | done | 76.85 | -0.065 % | 2.13 | 43 % | 0 |
| Haruna | sched, tightened | done x3 | 76.85 x3 | -0.065 % x3 | 1.73/1.73/1.81 | 59/59/51 % | 1/1/0 |
| Tsukuba | single | done / stopped / done | 72.70, -, 72.70 | | 1.00 | 100 % | 1/4/1 |
| Tsukuba | sched (M3b) | done | 72.75 | +0.138 % | 1.51 | 75 % | 3 |
| Tsukuba | sched, tightened | done / stopped / done | 72.70, -, 72.85 | 0.000 / - / +0.206 % | 1.00 | 100 % | 1/4/4 |

- Haruna: D now goes into four of seven hairpins (R 9.9, 10.6, 11.0, 11.2 m: within 1.25 x the
  R 9 cell), not into R 11.4, 12.0, 13.0, 18.8; a third fewer solves, the same time in every
  repeat (the drift plan at the first hairpins is the time difference, as in M3b).
- Tsukuba: no corner clears the threshold (its hairpins are R 15-18), so the scheduled run is
  060's G alone throughout: bit for bit the single start's plans (tested in M3b). Its +0.206 % in
  r3 and the stop in r2 are the single start's own run-to-run noise: G's solves cut at the 1.0 s
  budget near t 60 s (overruns 1-4 a run in both configs); in r2 both configs stopped there,
  "solves failed", three cut solves in a row. Over repeats the single start itself spans
  72.65-72.70 (M3b and here).
- R6: met on Haruna (-0.065 %, three of three); on Tsukuba the scheduled arm is the single start
  by construction, so R6 holds up to 060's own noise (0-0.2 % between runs, one stop in three at
  load ~5): an R3 budget matter of 060's, not the scheduler's.

## B. Model in the loop

**Method.** `studies/mode_choice_m4.py` drives `live.Driver` (L1 re-planning, L2 the NMPC
tracking) on `sim.Plant` (the two-track car, each wheel behind sim_vehicle_ecu) on the world's
true surface; `studies/m4_batch.py` runs the batches, at most four processes, pinned to cores
8-11 (multi-start: the driver on one core, its seeds on the other three). Arms: `single` (060's
one start), `always` (G, H as fallback, D every re-plan, F into hairpins), `sched` (M3b's
scheduler with part A's rule), and on the class-zone chain `sched` on a wrong map (10 and 30 %
of zones misdeclared, three seeds) with perception (`+perc`) and the car's estimate (`+est`).
Courses: 72 corners (asphalt, mud_wet, snow_loose, gravel, snow_packed, ice; R 9/15/25; 90 and
180 deg; 6 and 15 m wide), two repeats an arm; M3b's class-zone chain; Haruna and Tsukuba dry.
The oracle per corner: best-of-starts offline at the loop's settings (`oracle`), its drift gain
held - best. Tables: `mode_choice_m4.py aggregate` (swb
`play_log/race_planner/mode_choice/m4/aggregate.md`, `aggregate.json`).

**Load.** The corner and chain batches ran 2026-10-30 18:25-00:13 beside another user's work:
host load 2.6-15 (p10-p90), up to 46 in a few runs; a first pass above 16 was set aside
(`m4/loaded/`, not used for timing). The dry courses were re-run on an idle host (2026-10-31,
load 1.5-2.7, three repeats, `m4/quiet/`), and R6 is judged on those. The sideslip-bound arm
ran at load 1.5-5.6 (p10-p90).

### Outcomes and robustness (corners)

Outcomes per class and arm (24 runs each; spin = the driver's spin stop or the plant's 80 deg):

| class | single | always | sched |
|---|---|---|---|
| asphalt | 24 done | 24 done | 24 done |
| mud_wet | 18 done, 2 no first plan, 3 off road, 1 spin | 20, 2 no first plan, 2 off road | 20, 2 no first plan, 2 off road |
| snow_loose | 24 | 24 | 24 |
| gravel | 24 | 24 | 24 |
| snow_packed | 20, 2 spin, 2 timeout | 18, 4 spin, 2 timeout | 20, 2 spin, 2 timeout |
| ice | 13, 2 no first plan, 6 off road, 3 spin | 13, 2 no first plan, 5 off road, 1 spin, 3 timeout | 13, 2 no first plan, 4 off road, 4 spin, 1 timeout |

Low friction (every class but asphalt), runs that started (116 an arm):

| | stops | failed re-plans | without ice: stops, failed | solves / re-plan | L2 fallback ticks, ice |
|---|---|---|---|---|---|
| single | 17 | 78 / 1738 | 8, 14 / 1389 | 1.00 | 8.0 % |
| always | 17 | 67 / 1750 | 8, 10 / 1409 | 2.22-2.26 | 34.3 % |
| sched | 15 | 127 / 1811 | 6, 11 / 1414 | 2.00-2.16 (gravel 1.07) | 18.4 % |

- The extra starts save the car in mud (single: 4 stops of 22; both multi-start arms 2): the
  R 9 hairpin at 6 m, where the single start leaves the road and always/sched finish.
- Ice is lost in every arm: 9 of 22 started runs stop, the same count; the scheduled arm's
  failed re-plans double there (116 against 64), its overruns too (128 against 81; its ice runs
  ran at up to load 19 against 9).
- snow_packed: the same 4 stops single and sched (both the R 15 x 90 at 15 m, spin, and the
  R 25 x 90 at 15 m, timeout); always 6.
- Lateral error (median of each run's maximum) 0.4 m on asphalt, 0.6-1.2 m on low friction, in
  every arm; worst runs 7-13 m (snow_packed, ice), all arms.
- **No first plan** (a separate category, the same in every arm): `mud_wet:9:180:15` and
  `ice:9:180:6`, every repeat. The first plan is G alone from IPOPT's start, 400 iterations;
  offline the corner has plans from H and D (mud) or with more iterations (ice). A start-up
  defect, not the scheduler's: `docs/issues/001-no-first-plan-in-tight-low-friction-hairpins.md`.

### Time, and the oracle's gain kept

Per class, on the corners the drift map calls drift-optimal and every arm finished twice: what
the arm adds over the single start, in units of the oracle's summed gain (number of corners;
how many clear the two arms' repeat spread). The loop's own time is not compared with the
oracle's: the plant is not L1's model, and on asphalt the loop is faster than the oracle's
best plan (8.56 s against 8.62 at R 15 x 180).

| class | always | sched | time vs single, every finished corner: always / sched |
|---|---|---|---|
| asphalt | +0.10 (6; 3) | +0.69 (6; 0) | -0.01 / -0.14 % |
| mud_wet | +0.38 (6; 2) | +0.38 (6; 2) | -0.17 / -0.19 % |
| snow_loose | +0.19 (9; 5) | +0.22 (9; 5) | -0.29 / -0.55 % |
| gravel | +1.26 (5; 3) | +2.22 (5; 2) | -0.63 / -0.68 % |
| snow_packed | +0.25 (6; 3) | -0.25 (6; 2) | -0.41 / +0.29 % |
| ice | (the map calls none drift) | | -0.41 / 0.00 % (one corner) |

- Repeats are nearly bit-identical (median half range 0.00 % per corner on every class but
  ice, 0.55 %), so most differences are real; asphalt's sched gain is the R 9 hairpins
  (8.84 ± 0.11 against 8.95 at 15 m), inside its own spread.
- Above 1: the extra starts gain more than the oracle's drift gain (gravel R 9 hairpins:
  12.15 -> 11.75 s, the oracle 0.11 s): the single start loses on the plant what the model does
  not, and a second valley recovers it. snow_loose's sum is dominated by the R 9 x 180 at 15 m
  (oracle 26 %, 3.7 s: F; the loop keeps 0.9 s of it).
- snow_packed is the one class where sched is slower than single (+0.29 %: the R 9 x 180 at 15
  m, 24.35 ± 1.12 against 23.62 ± 1.16, inside the spread, and R 15 x 180 at 6 m, +0.04 s).

### The class-zone chain: true and wrong maps

Every arm on every map stops at the second corner, the mud_wet R 9 hairpin (s 242-262),
except where a 30 % map declares the first zones wrong (off road at s 76-94):

| map wrong | single | always | sched | sched + perc | sched + perc + est |
|---|---|---|---|---|---|
| 0 % (2 runs) | solves failed, spin | spin, spin | spin, spin | | (est only) off road@101, spin |
| 10 % (3 seeds) | 3 spin | 3 spin | 3 spin | spin, solves failed, off road@261 | 3 spin |
| 30 % (3 seeds) | solves failed, off road@262, off road@76 | spin, off road@94, off road@76 | spin, solves failed, off road@78 | spin, off road@76 | off road@94, off road@76 |

The free plans there ask 52-65 deg of sideslip; the car reaches 78-80 and spins. The 10 %
seeds misdeclare zones 4 (ice) and 8 (mud R 15): both behind the stop, so the wrong-map
criterion is met only vacuously (sched 3 stops of 3, always 3 of 3). The car's estimate never
lowered the friction (minimum 1.00 in every `+est` run): the stops come before a slide on a
misdeclared zone. Solves per re-plan: sched 2.30-3.43, always 3.47-3.67.

### Dry courses (R6)

Idle host, three repeats (`m4/quiet/`):

| course | arm | time (s) | vs single | solves / re-plan | G only | failed / overruns (3 runs) |
|---|---|---|---|---|---|---|
| Haruna | single | 76.28 x3 | | 1.00 | 100 % | 0 / 0 |
| Haruna | sched | 76.37 x3 | +0.118 % | 1.59 | 66 % | 0 / 0 |
| Haruna | always | 76.53, 76.38, 76.53 | +0.262 % | 3.26 | 0 % | 0 / 1 |
| Tsukuba | single | off road@1820, 74.17, 74.33 | | 1.00 | 100 % | 6 / 8 |
| Tsukuba | sched | off road@1820, 74.13, 74.13 | -0.162 % | 1.00 | 100 % | 6 / 6 |
| Tsukuba | always | off road@1809, 74.30, 73.87 | -0.222 % | 2.56 | 0 % | 4 / 5 |

- Haruna: the scheduled arm's D plans into the four R 9.9-11.2 m hairpins cost 0.09 s on the
  plant, in every repeat (the chain check without a plant had them gain 0.05 s). R6 missed by
  0.018 %. Loaded host (load 13-16, two repeats): single 76.30/77.02, sched 76.37 x2: under load
  the single start's own spread (0.9 %) hides it.
- Tsukuba: the scheduled arm is G alone (060's plans bit for bit); each arm's first repeat left
  the road at s ~1815 after four solves cut at the 1 s budget: 060's budget matter (part A's r2
  stopped the same way, cut solves in a row near the same time), not the scheduler's.

### The deep slides: drivable?

Not as planned. Where the free planner slides past 50 deg (mud_wet R 9 and R 15 x 180 at 6 m:
70 and 57 deg; snow_loose R 9 x 180 at 15 m, R 25 x 180: 46-52 deg) the car reaches 35-54 deg,
16-23 deg off its plan; in isolation it still finishes those corners (always and sched; single
leaves the road at mud R 9), but in the chain, entered from the asphalt zone, it spins at
78-80 deg.

A sideslip bound in L1 (`Driver(beta_max=...)`, 45 deg, every window, the seeds' too), on the
24 mud_wet and snow_loose corners and the chain:

- it does not make the mud hairpin drivable: at R 9 x 180, 6 m, every arm now leaves the road
  (sched finished free, 17.41 s); in the chain every arm still stops there (s 243-274, one
  scheduled run on a 10 % map at s 461; the car 52-78 deg against plans of 45);
- it starts `mud_wet:9:180:15` (no first plan free; 18.07 s bounded, the oracle's held plan
  17.05): it changes IPOPT's path, not only its optimum (on a test hairpin a 30 or 45 deg bound
  had no first plan where 60 deg and the free one did);
- time, corners both finished: single -0.36 % mean (21 corners), -1.39 % on the four deep ones
  (mud R 15 x 180 at 6 m: 18.55 -> 17.93 s, the plan followed: error 16 -> 9 deg); sched
  +0.47 % mean, +2.15 % on the deep ones (snow_loose R 9 x 180 at 15 m: 18.63 -> 20.84 s, the
  F plan lost).

So the bound is kept as an option, off by default (060's behaviour), and not adopted: it trades
one corner for another and does not fix the stop. The stop is the mud hairpin's entry: the
planner's mud tyre (no fall-off past the peak) makes a deep slide its optimum, and the plant's
car does not hold it. Proposed for M4b, not made: L2 tracking a slide past the plant's measured
limit (a bound in L2, not L1), or the mud tyre's sliding branch checked against the plant's.

### Gate (061, robustness first)

| criterion | result | |
|---|---|---|
| low friction: fewer stops and failed re-plans than single | stops 15 against 17 (mud 2 against 4; others equal); failed re-plans fewer on every class but ice (without ice 11 against 14), ice 116 against 64 | met except ice |
| lap time no worse than single on low friction | faster on mud, loose snow, gravel (-0.19 to -0.68 %), slower on packed snow (+0.29 %, inside the spread), ice not comparable | met except packed snow (within noise) |
| asphalt within 0.1 % (R6) | corners -0.14 %; Tsukuba G-only (-0.16 %); Haruna +0.118 % in 3 of 3 | **missed on Haruna by 0.018 %** |
| sched at least as good as always, fewer solves | stops 15 against 17, time better on 4 of 5 classes; solves 1.04 against 2.15 (asphalt), 1.07 against 2.23 (gravel), 2.0-2.16 against 2.22-2.26 (low friction), 1.59 against 3.26 (Haruna) | met (not on ice's failed re-plans) |
| 10 % zones wrong: sched stops no more than always | 3 of 3 each, all at the mud hairpin before any wrong zone | met only vacuously |

**Verdict: not passed.** The scheduler is as robust as always-on at a third to a half of its
solves, and saves the car in mud, but (1) R6 is missed on Haruna by 0.018 % (D into R 10-11 m
asphalt hairpins costs 0.09 s on the plant), (2) ice is lost in every arm and sched's failed
re-plans there double, (3) the wrong-map test is uninformative because every arm stops at the
chain's mud R 9 hairpin, and the car's estimate is never exercised.

### Rulings

- **Stops are counted on runs that started; "no first plan" is its own category.** Why: it is
  the same in every arm (the first plan is G alone in all of them), a start-up defect (issue
  001). Cost if wrong: 4 runs a class an arm missing from the stop counts, equally in each arm.
- **Kept = what an arm adds over the single start, in units of the oracle's gain**, not the
  loop's time against the oracle's. Why: the plant is not L1's model, so the loop is sometimes
  faster than the oracle's best plan. Cost if wrong: values above 1 (gravel) read as a loop
  better than the ceiling; they are the single start's plant loss recovered.
- **R6 judged on the idle-host re-run** (three repeats, load under 3), the loaded runs reported
  beside it. Why: under load 13-16 the single start alone spread 0.9 % on Haruna, ten times
  R6. Cost if wrong: none for the verdict (sched is +0.118 % idle, bit-identical in 3 repeats).
- **Tsukuba's off-road in each arm's first repeat is 060's**, not the scheduler's (G alone in
  sched, the same place and signature in single). Cost if wrong: a scheduler effect hidden in
  a repeat that failed for all.
- **The sideslip bound is not adopted** (kept, default None; design 4.4a unchanged). Why: it
  does not make the mud hairpin or the chain drivable and costs the scheduled arm 2.2 % on the
  deep corners. Cost if wrong: the single start's 1.4 % gain on deep corners is left unused.
- **The 10 % wrong-map criterion is recorded as met, vacuously.** Why: the stops precede every
  wrong zone, so the criterion is not falsified, nor tested. Cost if wrong: a wrong-map
  weakness is unseen until the mud hairpin is drivable.
- **The sweep (p_add, gain_min, sd) was not run.** Why: on the chain every arm stops at s 250,
  so a sweep there measures the same stop. Cost if wrong: thresholds stay at M3b's starting
  points.

### What is left (M4b)

1. Haruna R6: keep D off R 10-11 m asphalt hairpins; e.g. on classes with fall-off read the
   drift map without the 1.25 x radius stretch (only R <= 9 m gets D on asphalt), then re-run
   Haruna x3 idle.
2. The mud R 9 hairpin in the chain (and the deep slides): the planner's mud tyre against the
   plant's; a bound in L2 or the plant's limit in L1's tyre; then the chain's wrong-map and
   estimator arms become testable.
3. Issue 001: the first plan from H/D or more iterations when G fails.
4. Ice: why H does not reduce stops and the scheduled arm's failed re-plans double.
