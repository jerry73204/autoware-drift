# Grip or drift: seed coverage and the oracle (roadmap 061 M1, E0 and E0b)

Written 2026-10-30. The question: if L1's window is solved from a grip start and one or two
rear-out starts, does the best of them find the best plan any start finds (the oracle), and
does it do so inside the live re-plan's budget? This is experiment E0 of
[mode-choice-methods.md](mode-choice-methods.md); E0b asks whether another linear solver
shortens the live solves. Code: `studies/drift_optimality.py` (grids `multistart`,
`multistart-live`, `multistart-live-warm`, `e0b`; `multistart-report` prints every table
below). Results (jsonl, every plan as npz, logs, `report.md`): swb's
`play_log/race_planner/mode_choice/m1/`.

## Verdict

**The gate fails, honestly, on all three counts.** At the study's settings {G, Dc} (grip plus
the continuation seed) reaches the oracle in 90 % of the drift-optimal cases (45/50) and
{G, Ds, Dc} in 92 %; {G, Ds} (the synthetic seed) only in 72 %. At the live settings no
two-seed set reaches 90 %: {G, Dc} 73 %, {G, Ds} 47 %, all four starts 87 %. The undecided
mu 0.45 cases do not fall to 2: 9 of 30 remain at the study's settings (5 at the looser
threshold), because most of them are **ties**, not missed valleys: their best drift and best
grip plan differ by 0.01-0.03 s, inside the solver's own noise. And the drift seeds need
112-130 iterations at the median and 229-278 at p95 from scratch: 0.9-1.1 s at window.py's
4 ms an iteration, 2.1-3.1 s on this run's loaded host. E0b: SPRAL, the only other linear
solver in this build (HSL is absent), costs 4.4 times MUMPS's per iteration; MUMPS stays. What does work: the multi-start finds faster
plans than either study start in most low-friction cases (the rear-out and flick seeds are the
oracle in 52 of 70 cases), and it decides two of the study's eight undecided cases (both
drift, by 0.11 and 0.23 s).

**Use Dc live (a stored continuation plan for the corner), not the synthetic Ds**, with Ds as
the fallback where no stored plan fits; F only for tight hairpins. M3's mode-switch threshold:
**0.3 % of the window's time** (the noise floor's p75 on low friction), grip on a tie.

## Method

**Cases (70).** Every mu 0.45 and mu 0.3 corner of the study (sliding 1.0, 0.8, 0.6; R 9,
15, 25, 40, 60 m; 90 and 180 deg; 30 each) and every 180 deg asphalt hairpin (mu 1.12,
sliding 0.8 and 1.0, the five radii). The section, entry speed and car are the study's
([drift-optimality.md](drift-optimality.md), *Method*).

**Starts (8).** Each case is solved free from:

| start | what it is |
|---|---|
| C | cold: the minimum-curvature line at the quasi-steady speed (`window.initial`): the live loop's first plan, the study's free0 |
| G | grip: the plan held to the rear tyres' peak (held window, cold), the bound then removed. The live planner, defaulting to grip, warm-starts from its last plan shifted; offline there is no last plan, and the held plan is the grip valley's own. This is also the brief's held->free start |
| Ds | rear out, synthetic: G with the body turned so the rear axle runs 7.5 deg past its tyre's peak from 25 m before turn-in to the apex (ramped in over 5 m, out over 10): the path and the speed kept, the heading giving way to the new sideslip, the yaw rate from it, the steering keeping G's front slip; the request -0.3 m/s^2 on low friction, 0 (coasting) on asphalt there |
| Dc | rear out by continuation: the free plan with no fall-off (sliding 1.0) from C, then sliding stepped 0.9, 0.8, (0.7, 0.6) to the surface's, each step from the last. On a surface with no fall-off it is C itself |
| F | flick: as Ds, but the rear first out the other way (5 deg past the peak, 35 to 18 m before turn-in) |
| P1-P3 | G with smooth noise (12 m running mean) on every state: 3 % speed, 0.3 m/s sideways, 0.03 rad/s yaw, 0.3 m offset, 0.02 rad heading, 0.01 rad steer |

The held (grip) time a drift must beat is the better of two held solves: cold, and from the
best free plan clipped into the bound.

**Settings.**

- *Study*: the study's window (ds 2 m, 360 m, tol 1e-6, 3000 iterations, no budget), one solve
  over the whole section, time read 50 m past the corner.
- *Live*: live.Driver's window: ds 4 m, 260 m (the longest the Driver builds at these speeds),
  tol 1e-3, 400 iterations, `max_wall_time` 1.0 s (the Driver's budget), v_end the
  quasi-steady ceiling at the window's end, as live. Two option sets for a seed's solve:
  `multistart-live` (IPOPT's own barrier start, as the Driver's cold window) and
  `multistart-live-warm` (the Driver's warm window: mu_init 1e-6 and warm-start bound pushes,
  but no multipliers, since a fresh seed has none of its own). The grip plan G starts from, the
  held plans and Cnb (C without a budget, for the oracle only) are solved without the budget,
  as the Driver's first plan is. Each mode seed (G, Ds, Dc, F) is also solved without the
  budget, to count the iterations it needs.

A start counts if IPOPT converged and the plan is feasible and on the road (study), or if the
live loop would adopt it: feasible within the budget, on the road (live). The oracle is the
best time over every start. A case is **drift** if the oracle's rear axle is 3 deg or more past
its tyre's peak and the oracle beats the grip time by more than the threshold, **undecided**
if its rear is past the peak but the gain is within the threshold, **grip** otherwise. A seed
set reaches the oracle if its best plan is within the threshold of it.

**The noise floor.** Each case's spread over G and the perturbed starts that stay in G's
valley (the rear within 2 deg of G's past its peak): what the solver alone moves the time by,
without a change of valley. The threshold is its p75 per friction, as a share of the section's
time, at least 2 ms; p50 and p90 are reported too.

## Rulings

- **"The 20 mu 0.45 cases" are 30.** The study's mu 0.45 grid has 30 cases (3 slidings, 5
  radii, 2 angles); its rule (rear >= 3 deg past the peak, gain within the spread of its two
  starts) labels 14 drift, 8 undecided and 8 grip. All 30 were run; the gate's "8 -> 2" is read
  as "the study's 8 undecided, and the undecided count of the 30". Cost if wrong: none to the
  data, every case is in the jsonl.
- **G is the held->free start.** The brief lists both; offline there is no previous plan to
  shift, and the grip valley's plan is the held one. C (cold) is the eighth start instead.
  Cost if wrong: G is the cold held plan, which on low friction is itself a poor local optimum
  (the held solve from the oracle beats it by 0.12 % at the median, 1.1 % at p90), so {G} is a
  pessimistic stand-in for a well-converged live grip plan.
- **The live window is 260 m from the section's start, its time read at most 220 m.** Two
  sections do not fit one live window: R 40 x 180 deg (timing point 244 m) is read at 220 m,
  26 m past the corner; R 60 x 180 deg (its corner ends at 256 m) is read inside the corner,
  36 m before its end. The most faithful option: the live loop decides the mode in the window
  holding the corner's entry, which this is. Cost if wrong: those two cases' live times leave
  out part of the corner's exit.
- **Grip 1.0 at both settings**, the study's (the live planner's 0.92 moves a case down the mu
  axis, drift-optimality.md): the live labels then answer the same question as the study's.
- **Dc live is a stored plan.** A continuation chain is two to five solves (p50 9.4 s at the
  study's settings): offline work. Live, Dc is the study setting's continuation plan for the
  same case resampled onto the live grid (ds 2 -> 4), then one budgeted solve: a library of
  corner archetypes that matches exactly, the best such a library could do. Cost if wrong: a
  real library's nearest archetype is further from the corner, so Dc's live coverage and
  iteration counts here are an upper bound.
- **The host was loaded.** Another user's CARLA and Python ran throughout: load 17-128 on 32
  threads (a Ryzen 9 9950X, 16 cores with SMT). A live iteration cost 7-10 ms in the grids and
  6-9 ms in a serial check at load 17 (median 7.7 ms), against 4 ms in window.py's own
  measurement on the same size of window (50-65 nodes): the wall times below are up to twice
  that. Iterations do not depend on the load, so the live coverage is also given against an
  iteration budget (225 iterations: 0.9 s at window.py's 4 ms, 1.7 s at today's 7.7). Cost if
  wrong: if 4 ms an iteration does not hold on a quiet core of this host, the iteration-budget
  tables flatter the seeds, and the budget itself is short for G too.

## Noise floor

The spread of G and its perturbed starts that stay in G's valley, % of the section's time
(ms in brackets):

| friction | setting | cases | p50 | p75 | p90 | max |
|---|---|---|---|---|---|---|
| mu 0.45 | study | 30 | 0.128 (13) | 0.252 (27) | 0.427 (61) | 0.555 (74) |
| mu 0.3 | study | 28 | 0.029 (5) | 0.169 (26) | 0.462 (63) | 1.542 (253) |
| asphalt hairpins | study | 10 | 0.000 (0) | 0.001 (0) | 0.002 (0) | 0.002 (0) |
| mu 0.45 | live (warm) | 25 | 0.101 (14) | 0.302 (35) | 0.486 (67) | 0.899 (94) |
| mu 0.3 | live (warm) | 18 | 0.090 (13) | 0.184 (25) | 0.312 (39) | 1.022 (126) |
| asphalt hairpins | live (warm) | 10 | 0.000 (0) | 0.040 (4) | 0.208 (18) | 0.568 (47) |

On asphalt the solver lands on the same plan from every nearby start (2 ms at most at the
study's tolerance); on low friction the same valley holds several shallow local optima a few
hundredths of a second apart. The live spread is the converged solver's plus what a solve cut
by the budget leaves undone. **M3's threshold: 0.3 % of the window's time** (the live p75 on
mu 0.45, the plan's own "about 0.3 %"). On asphalt that is the tracking's margin, not the
solver's: only the 9 m hairpin's 0.40 % (swb's asphalt, sliding 0.8) clears it.

## E0: coverage at the study's settings

Threshold p75 (mu 0.45 0.252 %, mu 0.3 0.169 %, asphalt 2 ms):

| friction | cases | drift | grip | undecided |
|---|---|---|---|---|
| mu 0.45 | 30 | 18 | 3 | 9 |
| mu 0.3 | 30 | 24 | 0 | 6 |
| asphalt hairpins | 10 | 8 | 2 | 0 |

| seed set | reaches the oracle (50 drift cases) | all 70 cases |
|---|---|---|
| {C} (the study's free start) | 28 (56 %) | 44 |
| {G} | 31 (62 %) | 49 |
| {G, Ds} | 36 (72 %) | 56 |
| **{G, Dc}** | **45 (90 %)** | 64 |
| {G, Ds, Dc} | 46 (92 %) | 66 |
| {G, Ds, F} | 43 (86 %) | 63 |
| {G, Dc, F} | 50 (100 %) | 70 |
| {G, Ds, Dc, F} | 50 (100 %) | 70 |

At p50 (mu 0.45 0.128 %, mu 0.3 0.029 %) there are 60 drift cases and the shares are lower
({G, Dc} 80 %, {G, Ds} 62 %, {G, Ds, Dc} 92 %, {G, Dc, F} 97 %); at p90, 26 drift cases,
{G, Dc} 92 %, {G, Ds} 65 %. The oracle came from Ds in 20 cases, Dc 17, F 15, C 6, G 5, the
perturbed starts 7: **every start finds plans the others miss**, and the two study starts (C,
and the held plan's) are the oracle in only 11 of 70.

What the seeds find:

- **On asphalt the flick wins the 9 m hairpin with no fall-off** (sliding 1.0): F is 0.05 s
  (0.6 %) faster than any other start, the drift's gain 0.118 s (1.4 %, the study had 0.90 %).
  With swb's fall-off (0.8) Dc is the oracle in the three tight hairpins and the gains are the
  study's (0.034 s at 9 m, 0.021 s at 15 m, 0.011 s at 25 m); C and G miss them by 0.4-4 ms.
- **On low friction the held side is multi-modal too.** The held solve from the oracle's plan
  beats the cold held one by 0.12 % at the median and up to 1.1 %: part of the study's "noise"
  on mu 0.45 hairpins was the held plan's, not the free one's.

**The study's eight undecided mu 0.45 cases** (threshold p75; gain = grip time - oracle):

| case | oracle start | rear past peak | gain (s) | threshold (s) | now |
|---|---|---|---|---|---|
| sliding 1.0, R 15, 180 deg | C | 29.2 deg | 0.114 | 0.034 | drift |
| sliding 1.0, R 40, 180 deg | P1 | 12.6 | 0.028 | 0.038 | undecided |
| sliding 0.8, R 25, 180 deg | F | 4.5 | 0.023 | 0.036 | undecided |
| sliding 0.8, R 40, 180 deg | G | 3.8 | 0.020 | 0.038 | undecided |
| sliding 0.8, R 60, 180 deg | Ds | 3.3 | 0.011 | 0.042 | undecided |
| sliding 0.6, R 40, 90 deg | Ds | 3.1 | 0.009 | 0.026 | undecided |
| sliding 0.6, R 25, 180 deg | F | 4.2 | 0.228 | 0.036 | drift |
| sliding 0.6, R 40, 180 deg | Ds | 3.7 | 0.022 | 0.038 | undecided |

Two are decided (drift, 0.114 and 0.228 s); at p50 five are (the three more by 0.022-0.028
s). The six left have gains of 0.009-0.028 s, 0.06-0.2 % of the section: the oracle's drift
plan and the best grip plan are equal to within the solver's noise. Three cases the study
called drift (sliding 1.0 R 60 180 deg, 0.8 R 40 90 deg, 0.6 R 60 180 deg: gains 0.011-0.025
s) become undecided for the same reason, since the better held plans shrank their gains. So
**9 of 30 mu 0.45 cases are undecided at p75, 5 at p50, never 2**: the gate's premise (a
missing valley) holds for two of the eight; the rest are ties, where either mode is as fast
and R2's default (grip) is the answer.

## E0: coverage and cost at the live settings

The Driver's warm options (`multistart-live-warm`), threshold p75 of its own noise (mu 0.45
0.302 %, mu 0.3 0.184 %, asphalt 0.040 %):

| friction | cases | drift | grip | undecided | no plan |
|---|---|---|---|---|---|
| mu 0.45 | 30 | 6 | 11 | 12 | 1 |
| mu 0.3 | 30 | 17 | 0 | 8 | 5 |
| asphalt hairpins | 10 | 7 | 3 | 0 | 0 |

("no plan": no held plan converged at the live tolerance in 400 iterations, so no grip time.)

| seed set, one budgeted solve each | reaches the oracle (30 drift cases) | all 70 |
|---|---|---|
| {C} | 3 (10 %) | 11 |
| {G} | 9 (30 %) | 32 |
| {G, Ds} | 14 (47 %) | 42 |
| {G, Dc} | 22 (73 %) | 55 |
| {G, Ds, Dc} | 26 (87 %) | 60 |
| {G, Ds, F} | 14 (47 %) | 44 |
| {G, Dc, F} | 22 (73 %) | 55 |
| {G, Ds, Dc, F} | 26 (87 %) | 60 |

With IPOPT's own barrier start instead (`multistart-live`, p75, 30 drift cases): {G} 50 %,
{G, Ds} 67 %, {G, Dc} 60 %, {G, Ds, Dc} 73 %, all four 77 %. The high barrier lets G and Ds
wander further from their start (so they find more drifts), the warm options keep a seed in
its valley and converge it sooner: G in 18 iterations at the median (62 cold), Dc's stored
plan in 82 (usable 64 of 70 against 33). For a seed that is meant to hold a valley, the warm
options are the right ones.

Against an iteration budget instead of this host's clock (each mode seed's solve without the
budget, counted if it converged within the budget; the study's noise floor; 37 drift cases in
`multistart-live-warm`):

| seed set | 150 iterations (0.6 s) | 225 (0.9 s) | 300 (1.2 s) |
|---|---|---|---|
| {G} | 30 % | 30 % | 30 % |
| {G, Ds} | 43 % | 51 % | 54 % |
| {G, Dc} | 35 % | 59 % | 70 % |
| {G, F} | 46 % | 54 % | 54 % |
| {G, Ds, Dc} | 49 % | 76 % | 86 % |
| {G, Dc, F} | 51 % | 76 % | 86 % |
| {G, Ds, Dc, F} | 57 % | 81 % | 89 % |

Iterations and wall time per start at the live settings (warm options; wall times on the
loaded host, up to twice window.py's 4 ms an iteration; the budgeted solves are cut at 1.0 s):

| start | usable | iterations p50/p95 | wall p50 | p95 | p99 | within 0.9 s | ms/iteration |
|---|---|---|---|---|---|---|---|
| G | 63/64 | 18/74 | 0.13 | 0.56 | 1.01 | 95 % | 7.5 |
| Ds | 46/64 | 100/142 | 0.83 | 1.01 | 1.01 | 53 % | 7.9 |
| Dc | 64/70 | 82/133 | 0.66 | 1.01 | 1.01 | 67 % | 7.9 |
| F | 29/64 | 108/141 | 1.00 | 1.01 | 1.02 | 30 % | 7.9 |
| P1-P3 | 125/192 | 102-107/134-145 | 0.89-1.00 | 1.01 | 1.01-1.02 | 38-50 % | 7.8 |
| C | 22/70 | 128/151 | 1.01 | 1.01 | 1.01 | 11 % | 7.6 |
| G, no budget | 63/64 | 63/121 | 0.42 | 1.09 | 1.23 | 89 % | 7.1 |
| Ds, no budget | 62/64 | 112/229 | 0.85 | 2.18 | 3.59 | 62 % | 7.4 |
| Dc, no budget | 70/70 | 123/278 | 0.88 | 3.11 | 3.43 | 51 % | 8.3 |
| F, no budget | 64/64 | 130/268 | 0.93 | 2.08 | 2.71 | 44 % | 7.2 |

(The no-budget solves start from IPOPT's own barrier, the Driver's cold window: their
iterations are the cold counts. The budgeted warm solve of Dc needs fewer: 82 at the median.)

So a drift seed needs 110-130 iterations at the median and 230-280 at p95 to converge from
scratch: **p95 0.9-1.1 s at 4 ms an iteration, 2.1-3.1 s here.** The gate's "D's p95 <= 0.9 s"
fails here and is at best borderline at 4 ms an iteration. A seed that misses the budget is often
still feasible (Dc 64 of 70 usable under the warm options), so the live loop gets a plan, but
not the seed's optimum: that is the gap between the live 87 % and the study's 92-100 %.

## E0b: the linear solver

**HSL is not in this build.** CasADi 3.8.2's IPOPT (3.14.19) loads MA27/MA57/MA97 from
`libhsl.so`, which is not on this host (and no `libcoinhsl` either): `linear_solver ma27`
fails with "libhsl.so: cannot open shared object file". HSL is licensed software; it was not
downloaded (ask before doing so). Pardiso is not built in. **SPRAL is** (statically linked),
so E0b ran the live grid on MUMPS and on SPRAL, each case on both in the same worker, the
order alternating by case (`e0b`, 70 cases, 4 workers, one OpenMP thread):

| start | MUMPS p50/p95/p99 (s) | SPRAL p50/p95/p99 (s) | iterations p50, MUMPS/SPRAL | same plan (within 1 ms) |
|---|---|---|---|---|
| C, no budget (Cnb) | 2.21/4.66/5.29 | 9.63/16.26/19.06 | 248/248 | 31/64 |
| G (1 s budget) | 0.45/0.96/1.00 | 1.02/1.04/1.05 | 63/27 | 3/29 |
| Ds | 0.89/1.01/1.02 | 1.02/1.07/1.09 | 98/24 | 1/9 |
| Dc | 1.00/1.01/1.02 | 1.02/1.04/1.05 | 83/23 | 3/11 |
| F | 1.00/1.01/1.01 | 1.02/1.06/1.15 | 105/24 | 0/3 |

SPRAL takes the same number of iterations (248 at the median without the budget) at **4.4
times the cost each** (35-38 ms against 8-10 ms; with four OpenMP threads 34.6 ms, no
better), so within the budget it gets through a quarter of the iterations and almost never
converges; its plans differ from MUMPS's in half the converged cases (other pivots, another
path). Over every budgeted start p95 is 1.05 s against 1.01 s (+4 %). **Not adopted; MUMPS
stays.** E0b's lever, if any, is HSL's MA27/MA57, which needs a licence decision first.

## What M3 should use

- **Seeds: G (the shifted plan, warm), Dc (a stored continuation plan per corner archetype),
  and Ds where no archetype fits.** {G, Ds, Dc} is the best three-start set at every setting
  (92 % study, 87 % live in one budgeted solve). F adds only tight hairpins with no fall-off;
  run it, if at all, in place of Ds on 180 deg corners of R <= 15 m.
- **A seed converges over re-plans, not in one.** Most budget misses are feasible plans short
  of their valley's floor; a seed process that keeps its own plan (warm-started, with its own
  multipliers) from one re-plan to the next is continuation in time and should close the gap
  that one solve leaves. M3 should measure this before cutting the seeds.
- **The mode-switch threshold: 0.3 % of the window's time**, two re-plans in a row, grip on a
  tie. The ties are real: 5-9 of the 30 mu 0.45 cases have drift and grip within the noise.
- **The budget**: 0.9 s a seed is enough for G, short for a cold drift seed (p95 230-280
  iterations). On this host while it is shared, a seed's iteration takes 7-10 ms; M3's timing
  gate (overruns <= 1 % of re-plans) should be measured on a quiet core or by iterations.
- **The held plan's own valleys** matter for any comparison of modes: the grip time must be
  the best of at least two held starts.
