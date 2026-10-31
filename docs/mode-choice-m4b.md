# Grip or drift, model in the loop: what M4 left (roadmap 061 M4b)

Written 2026-10-31. M4 (`docs/mode-choice-m4.md`) did not pass its gate: R6 missed on Haruna by
0.018 %, no first plan at two tight low-friction hairpins (issue 001), every arm stopped at the
zone chain's mud R 9 hairpin (now issue 002), and ice lost in every arm (now issue 003). M4b
fixes what it can, re-runs, and re-judges the gate. Harness as M4 (`studies/mode_choice_m4.py`,
`studies/m4_batch.py`, cores 30, 23, 10, 31: the host's least loaded); results swb
`play_log/race_planner/mode_choice/m4b/` (dry courses on an idle host in `quiet/`). Each run's
configuration was checked in its result JSON (`front_slip_max`, `rear_slip_max`, `tag`,
`beta_bound`) before it was counted.

Load: another user's build took the host to 30-78 at the start (07:10-07:30); batches waited
for a 1-min load under 10 (robustness) or 6 (timing); from 07:45 the host was idle (load
1.5-5 with M4b's own four processes). Every result records its load at start and end.

## 1. Asphalt R6: no D into R 10-11 m asphalt hairpins

**Change** (`schedule.py`, `RULES["r_tol_falloff"]` 1.05): on a class that falls off past its
peak a corner gets D only up to the measured paying cell's radius (5 % for the corner finder's
discretisation: the M4 corners' R 9 reads 9.0), not 1.25 x it; classes without fall-off keep
the stretch. Judged: between the measured R 9 (0.32-0.48 %) and R 15 (0.23 %) the gain at
R 10-11 is the map's interpolation, and the plant says the opposite (D into Haruna's R 9.9,
10.6, 11.0 and 11.2 m hairpins cost 0.09 s in each of M4's three repeats). On swb's courses no
asphalt corner is that tight (Haruna's tightest 9.87 m), so asphalt is now G alone everywhere
but an R <= 9.45 m hairpin; gravel's R 9 hairpins and its 15 m-wide R 15 cell, packed snow's
R 9, keep D. Test: `test_schedule.py` (R 9.0 and 9.4 pay, 9.9-12 do not, mud's R 11 still does).

**Dry courses, idle host** (`quiet/`, three repeats, load 1.3-4.9 over the batch, each run's own
start/end in its JSON):

| course | arm | time (s) | vs single | solves / re-plan | G only | failed / overruns (3 runs) |
|---|---|---|---|---|---|---|
| Haruna | single | 76.28 x3 | | 1.00 | 100 % | 0 / 0 |
| Haruna | sched | 76.28 x3 | **0.000 %** | 1.00 | 100 % | 0 / 0 |
| Haruna | always | 76.88, 76.88, 76.38 | +0.79, +0.79, +0.13 % | 3.26 | 0 % | 0 / 1 |
| Tsukuba | single | 74.50, 74.42, off road @1820 | | 1.00 | 100 % | 5 / 7 |
| Tsukuba | sched | off road @1820 x3 | (not comparable) | 1.00 | 100 % | 12 / 12 |

- Haruna: the scheduled arm is now 060's plans bit for bit (G alone on every re-plan, the same
  times to the centisecond in three repeats): R6 met with 0.000 % (M4: +0.118 %).
- Tsukuba: the scheduled arm is G alone on every re-plan (as in M4: the same plans as the single
  start's, solved in the driver's own process), and both arms leave the road at s 1820 the same
  way, four solves in a row cut at the 1 s budget near s 1815: single in one repeat of three,
  sched in three of three (M4's quiet re-run: one of three each). Which repeat crosses the budget
  there is the wall clock's (the latency is each solve's measured time), so the time comparison
  R6 asks for is not measurable on Tsukuba until 060's budget matter at s 1815 (R3) is fixed;
  the scheduler adds no start and no plan there.

## 2. Issue 001: the first plan

`live.Driver.start` now rescues a first plan that G from IPOPT's start does not find in the cold
window's 400 iterations: G again with 3000, then H held at the window's least rear peak and G
from H's plan, then D (synthetic) and F into the first corner, last G with the slip bounds of
sections 3-4 off; the first feasible plan is
adopted, in every arm alike (no budget: the car is not moving). Issue 001, closed.

| corner | before (M4) | the rescue's tries (offline) | in the loop, free planner (2 repeats an arm) | with M4b's slip bound (section 3) |
|---|---|---|---|---|
| mud_wet R 9 x 180, 15 m | no first plan, every arm | G, G+: Restoration_Failed (181 it.); H 220 it.; G from H 45 it.; 12.8 s | starts; every arm stalls sideways at s 65 (the car 51 deg against 41) | every arm finishes: single 17.73/18.27, always and sched 16.91 x2 s (oracle 16.22 best, 17.05 held) |
| ice R 9 x 180, 6 m | no first plan, every arm | G+: 668 it.; 13.4 s | starts; off road in 11 of 12 (re-plans cut at the budget: issue 003) | see section 4 |

`live.run` also says "stalled" (a car under 0.3 m/s for 5 s with no stop asked for) instead of a
timeout two minutes later: the mud corner's stalls showed the loop's next plan never adopted
once the car stands still ahead of it.

## 3. The mud hairpin: root cause and fix (issue 002)

**Root cause** (issue 002, measured offline on the chain's free plan from s 150 at 20 m/s, the
state the loop enters the mud zone with):

- *Not the model.* L1's car and the plant's, evaluated on the plan's own states and inputs at
  every node of the slide, agree to 0.01 m/s^2 and 0.01 rad/s^2; the mud tyre is the same
  function in both.
- *The plan.* On a class whose friction does not fall past the peak (mud: `sliding` 1.0) a
  saturated tyre keeps its whole force however deep it slides, so the free plan brakes into the
  hairpin by plowing: front wheels steered 50 deg against the slide, the front tyres at 55-66 deg
  of slip, 9-21 times their peak, the rear at 7-8 times, the body at 52 deg and 3-5 m/s. It is
  0.8 s faster on paper than a plan whose front stays within twice its peak.
- *Following it.* L2 on the plant, the plan fixed: on the plan to s 232, then with every tyre
  saturated the car over-rotates while L2 unwinds the steering to no effect: 77 deg at s 254
  against 53, stopped sideways at s 265. No controller holds a slide whose forces no longer
  depend on the slips. The 45 deg body bound of M4 does not touch it.

**Fix**: every L1 window, the seeds' too, keeps the front axle's slip angle within its peak and
the rear's within three times it, on a surface without fall-off past the peak only
(`window.slip_rows`, `live.Driver(front_slip_max=1, rear_slip_max=3)`, the defaults; no rows on a
track without such a surface, so asphalt's windows are 060's exactly); the first stages are free
of the rows (the car's own node may already be past a bound: section 4).

**Choosing the bound** (the zone chain, true map, single/always/sched x 2, load 1.1-2.2;
before the first-stage relaxation):

| bound (front, rear; x peak) | runs done (of 6) | stops at the mud hairpin | other stops |
|---|---|---|---|
| free (M4) | 0 | 6 | - |
| 2, free | 1 | 1 (stall) | snow_loose s 458, ice 774-847, packed snow s 1225 |
| 2, 3 | 2 | 2 (single, s 265: the car 52 deg against 35) | s 1074-1077 |
| 1.5, 3 | 1 | 1 (single, s 265: 64 against 35) | s 100 x2 (always), ice 830, s 1075 |
| **1, 3** | **2** | **0** | ice s 832 x2, packed snow s 1228-1254 |

Front at its peak, rear to three times it: the mud hairpin is driven in every run (the car's
largest sideslip 23-32 deg against plans of 27-36), and the chain now stops on ice and at the
packed-snow hairpin.

**Its cost and benefit on the corners** (mud and loose snow, the classes it binds on; M4b's
defaults, one repeat an arm, load 3.7-6.8; against M4's r1 on the corners both finished):

| class | arm | finished: M4 / M4b | time vs M4 |
|---|---|---|---|
| mud_wet | single | 9 / 11 | +0.19 % (9) |
| mud_wet | sched | 10 / 12 | +0.57 % (10) |
| mud_wet | always | 10 / 12 | +0.56 % (10) |
| snow_loose | single | 12 / 12 | -0.79 % (12) |
| snow_loose | sched | 12 / 5 of 5 run | +8.94 % (5) |
| snow_loose | always | 12 / 5 of 5 run | +3.15 % (5) |

(Loose snow's multi-start arms ran 5 of 12 corners each before the host's load passed 10 again
at 10:30 and the batch was stopped; the remaining 14 runs and the sweep were not run.)

Mud finishes every corner in both multi-start arms (M4: two no first plan, two off road), the
single start 11 of 12. Time moves both ways: where the free plan's deep slide was not followed
(mud R 15 x 180: 57 deg planned) the bounded plan is faster (-2 to -3 %); where a deep F drift
was followed in isolation (loose snow R 9 x 180 at 15 m, 48 deg planned) it is slower (+9 to
+11 % sched/always), the drift worth keeping lost to the bound. Loose snow's R 9 x 90 at 15 m
sched (+29.7 %, 13.43 -> 17.42 s) is one run at load ~5 against 13.4-13.6 in its other arms.

## 4. Ice (issue 003)

**Root cause** (issue 003; the loop instrumented tick by tick, idle host):

1. *The plans sit past the rear tyre's peak, on its falling branch.* Ice peaks at 2.95 deg of
   rear slip angle and falls to 0.7 of its friction past it. `ice:15:90:15`: every re-plan
   solved, but the plans ran the rear at 4.1-4.9 deg (1.4-1.7 x the peak); a degree's lag at s
   55 and the car's slip ran away monotonically (8 deg at s 59, 20 at 88, 30 at 100) while the
   plan stayed at 4-5: off road at s 116. Past the peak more slip gives less force; the slide is
   unstable, the plant (the same tyre) only shows it.
2. *H cannot start there.* H's held rows bound every node, the car's own first: once the car's
   rear was past the bound, every H window was infeasible (`ice:15:90:15`: H failed on all 8
   re-plans from s 56, exactly when the car left its plan).
3. *The solves hit the budget.* `ice:9:180:6`: G and H both cut at the 1 s wall on 8 of 10
   re-plans, G's finished plans running the rear to 8.8 deg (3 x the peak); ice's L1 p50 is
   0.5-0.9 s against 0.1-0.2 elsewhere.

**Fixes**: (a) the held rows (and issue 002's slip rows) leave the window's first three stages
(12 m) free, `window.FREE_STAGES`, in the live loop's windows: a plan must come back within its
bound, not start in it; (b) on a surface that falls off deepest and is slippery (sliding share
<= 0.75 and planned friction <= 0.45: ice, not packed snow's 0.8), every start holds the rear at
its peak (`live.Driver(rear_slip_low=1)`, `window.slip_rows`).

**Measured** (the 12 ice and 12 packed-snow corners, one repeat an arm, load 1.6-3.6; `a`: (a)
alone, `rl1`: (a) and the rear held; M4 at load 6-15 for reference):

| class | arm | M4 done | (a) done | (a)+(b) done | (a)+(b) failed re-plans | time, (a)+(b) vs M4 (corners both finished) |
|---|---|---|---|---|---|---|
| ice | single | 6 | 6 | **9** | 49 / 209 | +3.3 % (5) |
| ice | sched | 7 | 4 | **8** | 49 / 194 | +4.0 % (5) |
| ice | always | 7 | 3 | **6** | 37 / 169 | +1.8 % (4) |
| packed snow | single | 10 | 10 | 9 | 9 / 227 | -0.6 % (9) |
| packed snow | sched | 10 | 10 | 8 | 12 / 215 | +1.1 % (8) |
| packed snow | always | 9 | 9 | 8 | 4 / 213 | +6.8 % (8) |

(M4's ice counts include `ice:9:180:6`'s no first plan as not done.) The rear held at its peak
lifts ice from 13 to 23 runs of 36 finished; on packed snow it cost 4 of 29 (why, not measured),
so (b) is ruled to ice only: the sliding
share separates them (0.7 against 0.8), and with it packed snow's windows are (a)'s. Ice
remains the weakest class: 8 of 12 for the scheduled arm, a third of its re-plans failing at the
budget; what is left is (3), L1's speed on the falling branch, an R3 solver matter (design
section 10's solver levers), not the scheduler's.
## 5. The chain: wrong maps and the estimator

M4b's defaults (`fin`: front at its peak and rear within 3 x on surfaces without fall-off, ice's
rear at its peak, first stages free), the chain on the true map (two repeats; sched with the
estimate too) and on 10 and 30 % wrong maps (three seeds each; single, always, sched, sched +
perception, sched + perception + estimate), load 1.2-3.7 (one run 11); `aggregate.md` has every
run. Zone index: 0 asphalt (uncertain), 1 mud R 9 hairpin (s 248), 2 loose snow, 3 gravel,
4 ice (R 40 hairpin, s 804-932), 5 loose snow, 6 packed snow (R 25 hairpin, s 1200-1280),
7 asphalt, 8 mud R 15, 9 asphalt.

| map | single | always | sched | sched + perc | sched + perc + est |
|---|---|---|---|---|---|
| true (2 runs; est: sched + est) | ice @835, **done 154.22** | packed snow @1249, ice @837 | packed snow @1238, ice @767 | | (est) ice @854, **done 154.28** |
| 10 % s1 (zone 4 wrong) | ice @850 | ice @852 | ice @849 | ice @849 | ice @841 |
| 10 % s2 (zone 8) | **done 155.18** | **done 153.73** | ice @944 | ice @838 | mud hairpin @265 (estimate 0.39) |
| 10 % s3 (zone 8) | ice @835 | first corner @100 | mud R 15 @1707 | @1708 | @1706 |
| 30 % s1 (3, 4, 7) | gravel @639 | first corner @101 | @639 | @639 | @638 (estimate 0.55) |
| 30 % s2 (1, 2, 6) | mud hairpin @261 | @258 | loose snow @419 | stalled @268 | @101 |
| 30 % s3 (0, 1, 6) | @76 | @76 | @77 | @76 | @76 |

- The mud hairpin no longer stops any run on the true map or a 10 % map but one (the estimate's,
  below); the chain's stops are now ice (zone 4) and the packed-snow hairpin, so the 10 % maps'
  wrong zones (4 and 8) are reached: the wrong-map test is no longer vacuous.
- **10 % wrong: sched stops 3 of 3, always 2 of 3** (single 2 of 3). The criterion "sched stops no
  more often than always" is **not met**, by one run. By distance the scheduled arm got further
  (849, 944, 1707 m against 852, 2082, 100 m: always's s3 left the road at the first corner, the
  uncertain asphalt zone, where all-starts-always adopted a plan sched does not run).
- Perception changes little (the same stops within 10 m); the car's estimate is now exercised
  (it lowered to 0.39 and 0.55 in two runs, held through the slides) and once made it worse:
  on 10 % s2 it lowered on the mud hairpin's slide and the re-plan at the lowered friction left
  the road at s 265, where without it the run went on to the ice zone.
- 30 % wrong: every arm stops early, mostly at the same place (a zone declared grippier than it
  is: @76 on s3, @639 on s1), as in M4.
- Solves per re-plan on the chain: sched 2.2-2.7, always 2.9-3.5.

The sweep (p_add, gain_min, sd) was not run: the host's load rose past 10 again at 10:30 (another
user, 16-20), and the chain's stops are now ice and the packed-snow hairpin, which no scheduler
threshold reaches.
## 6. Gate

| criterion | M4b | |
|---|---|---|
| low friction: fewer stops and failed re-plans than single | mud: sched/always 12 of 12, single 11; ice: sched 8, always 6, single 9 of 12 (rear held); packed snow (unchanged by M4b's bounds) sched 10, always 9, single 10; failed re-plans equal or fewer but ice (49 each) | met except ice (single finishes one more) |
| lap time no worse than single on low friction | M4's per-class result stands where M4b's bounds do not bind (gravel, packed snow); mud and loose snow re-measured against M4, not against single in one batch | not re-judged |
| asphalt within 0.1 % (R6) | Haruna 0.000 % x3 (sched = single bit for bit); Tsukuba not measurable (060's cut solves at s 1815 stop both arms) | **met on Haruna** |
| sched at least as good as always, fewer solves | as robust or more on every class (ice 8 against 6); chain solves 2.2-2.7 against 2.9-3.5 a re-plan; Haruna 1.00 against 3.26 | met |
| 10 % zones wrong: sched stops no more than always | now tested (the wrong zones are reached): sched 3 of 3, always 2 of 3, single 2 of 3; sched got further on average (1167 against 1011 m) | **missed by one run** |
| the zone chain's mud hairpin | 0 stops in 26 runs on the true and 10 % maps but one (the estimate's) | fixed |

**Verdict: not passed.** R6 is met on Haruna and the mud hairpin, issue 001 and the asphalt
regression are fixed, but the wrong-map criterion, now testable, is missed by one run of three
(the scheduled arm left the road on the ice and mud zones where always and single each
finished one 10 % map), and ice remains lost in a third of runs in every arm, its re-plans cut
at the budget (an R3 solver matter). Three seeds a map are too few to separate 3 of 3 from 2 of
3; M4c should run the 10 % maps with more seeds before re-judging, and take up ice's solve time.

## Rulings

- **R6 on Haruna is judged met at 0.000 %** (three of three, bit-identical to the single start).
  Why: the rule leaves asphalt G alone outside R <= 9.45 m hairpins, which swb's courses do not
  have. Cost if wrong: an asphalt R 9 hairpin elsewhere still gets D, untested on the plant
  (M4's corners: R 9 asphalt hairpins finished in every arm, -0.14 % on average).
- **Tsukuba's R6 is recorded as not measurable, not as missed.** Why: the scheduled arm is G alone
  (the single start's plans) and the off-road at s 1820 is 060's cut solves, in both arms
  (single one of three, sched three of three, M4 one of three each): which repeat crosses the
  budget is the wall clock's. Cost if wrong: a scheduler effect on Tsukuba hidden in 060's
  budget noise; it can only be timing (no start is added there).
- **The slip bound is a limit of control, applied only where the tyre does not fall off.** Why:
  on those surfaces nothing else keeps a plan out of a slide with every tyre saturated; where the
  tyre falls off, its own fall-off does (a front bound on ice made every re-plan fail). Cost if
  wrong: a mud or loose-snow drift deeper than three times the rear's peak is never planned
  (M4's free plans had the rear at 7-8 x on the deep corners; see section 3 for the time).
- **The front at its peak (1 x), not 1.5 or 2.** Why: of the three, only 1 x drove the mud
  hairpin in all six chain runs; the drift doctrine agrees (the front axle keeps its grip, the
  steering keeps its authority). Cost if wrong: slower deep mud corners than a looser bound.
- **The rear held at its peak on ice only, separated by the sliding share (<= 0.75).** Why: it
  finished ice 13 -> 23 of 36 and cost packed snow 4 of 29, one repeat each; no drift pays on ice.
  Cost if wrong: a one-repeat result on a chaotic class (ice's outcomes flip between repeats of
  the same configuration); and the bound made one 30 % wrong map's first window (asphalt onto a
  zone declared ice) locally infeasible in every start, so the first-plan rescue ends with G
  with the slip bounds off.
- **The first stages free of the held and slip rows** (12 m). Why: those nodes are the car's
  state and the next; a bound there is infeasible whenever the car is already past it, which is
  exactly when H is needed. Cost if wrong: a plan may start 12 m past its bound.
- **Ice's remaining loss is ruled an R3 solver matter.** Why: a third of ice's re-plans still
  fail at the 1 s budget in every arm, the scheduler adds starts but not speed. Cost if wrong:
  a planner-side fix for ice (another formulation of the falling branch) left unexplored.
