# Grip or drift, model in the loop: ice's windows and the solve budget (roadmap 061 M4d)

Written 2026-10-31. M4c (`docs/mode-choice-m4c.md`) left ice: its corners lost in a third to a
half of runs, a third of its re-plans failed, the windows needing 200-930 IPOPT iterations
against the 1 s budget's ~160. M4d's task: make ice's windows fit the L1 solve budget, root
cause first. Results: swb `play_log/race_planner/mode_choice/m4d/` (`rec/` the recorded runs,
`det/` the repeatable runs, the rest wall-clock runs). Every run's configuration was checked in
its result JSON (`tag`, `rear_slip_low`, `det`, `itercap`, `rp_root`) before it was counted.

All of M4d's runs are on L2 before issue 007's fix (autoware-drift 14a79e7, another session,
landed after them); the tracker's change is not in these numbers.

Load: 3-8 until 16:40, then another user's CARLA and another session's Rust builds took the
host to 15-87 for the rest of the session. Wall-clock runs were started only under a 1-min load
of 6 and record their own load; most of M4d's closed-loop comparisons are therefore made in
**repeatable runs** (below), which no load changes.

## Method

- **Recorder and replayer** (M4c's, kept out of the repository): every budgeted window solve of
  a run, its inputs (state, warm start, multipliers, friction) and outcome, pickled; replayed
  offline deterministic to the iteration. Corpus: ice's 12 corners, single start, two repeats,
  on M4c's code: **413 solves**, 119 of them not a plan within the budget (the replay at 170
  iterations makes 294 of 413 plans, the loop's own record 292).
- **Offline metric**: a plan (IPOPT's success or `Window._result`'s feasibility) within 170
  iterations, the budget's iterations at the loop's 5.5-6.5 ms an iteration; and, for the 119,
  whether and in how many iterations they converge unbudgeted.
- **Repeatable closed loop** (`studies/mode_choice_m4.py`, new `RP_ITERCAP`): `RP_DET=1` (no
  clock: every plan adopted at the fixed latency) with each budgeted solve cut at `RP_ITERCAP`
  iterations a second of budget instead of by the wall clock. A run is then repeatable bit for
  bit at any load (checked: two runs of a corner, identical). Budgets 150, 160, 170, 185, 200
  and 400 iterations stand in for repeats.

## 1. Why ice's windows take so many iterations

- **A bug (issue 008).** The slip-ratio rows' `KAPPA_MAX` (0.6) is absolute; ice peaks at slip
  0.05, a fifth of the rig's, so on ice the rows allowed twelve of its peaks, nearly all on the
  flat of its fall-off where the slope rows have no gradient. Of the 119 windows not solved
  within the budget, 56 end `Infeasible_Problem_Detected` even unbudgeted: IPOPT's iterate at
  the rear wheels' 0.6 bound, the rear slope rows violated, multipliers 1000 there
  (`ice:15:90:15`, a 9 m/s exit straight).
- **What remains.** The car is off its plan (it slides where the plan does not): the warm
  start's first ten stages are 1.3 m/s^2 off in the acceleration rows at the median where the
  solve fails (0.3 where it succeeds), and the stages' Hessian blocks are strongly indefinite
  (eigenvalues to -1e8, multipliers to 2.6e4 on the front-balance and acceleration rows: ice's
  sharp tyre). IPOPT then spends half its iterations in restoration (one window: 130 of 461)
  with its Hessian regularised by 10^2-10^4, steps of 1e-3. And the first stage's slope rows can
  be infeasible outright: a wheel's slope depends on its slip angle, and at stage 0 that is the
  car's.

## 2. Candidates, offline (the 413 recorded ice solves)

| candidate | plans within 170 iterations | the 119 unbudgeted: converge (median iterations) | |
|---|---|---|---|
| M4c's code | 294 (71 %) | 58 (282) | 56 infeasible |
| no multipliers | 294 | | |
| acceleration rows made consistent in the warm start | 291 | | |
| `mu_init` 1e-2 | 291 | | |
| IPOPT options (13 solves of `ice:9:180:6`: `recalc_y`, `neg_curv_test_reg` no, `theta_max_fact` 1e8, `alpha_for_y` safer, L-BFGS, `mu_init` 1e-3, cold `mu_init` 0.1, bound pushes 1e-3) | | 6-10 of 13 converge, median 289-1000 (M4c's: 9, 340) | none better |
| first 10 stages rolled out from the car's state | | 5 of 13 | worse |
| 8 free stages; ice's rear bound off | | 11, 12 of 13 (median 249, 351) | not the cause |
| wheel speeds scaled by the surface's peak | | 6 of 13 | worse |
| slips (in the surface's peak) as the controls | | 10 of 13 (median 446) | no better |
| **slip rows in ice's peak (issue 008, shipped)** | **313 (76 %)** | **83 (233)** | 30 infeasible |
| and the free stages free of the slope rows | 325 (79 %) | 112 (218) | drives worse (section 3) |
| Fatrop (issue 008's rows, 300-iteration cap) | 231 of 280 (82 %) | | 6.9 ms an iteration against IPOPT's 6.7 on this host (load 3-5): within 1 s 75 % against IPOPT's 72 % (1 s wall, 296 of 413) |
| SQP (CasADi `sqpmethod`): `qrqp` | 0 of 7 | | 3 s an iteration (dense QP) |
| SQP on HPIPM | not built | | CasADi's interface refuses the problem: the request's rate penalty couples a_k and a_k+1 across stages; the window would have to carry the request as a state |

Fatrop is no faster per iteration on ice's windows here (M3 measured 3.3 against 5.9 ms on
Haruna's), takes no multipliers and makes other plans (M3: 17 % of windows beyond 0.3 %): not
adopted. MUMPS orderings and HSL were not tried: with the iterations, not the iteration, the
problem (200-900), no per-iteration factor in reach closes it.

## 3. Closed loop

**Repeatable runs, ice's 12 corners, single start** (finished; failed re-plans):

| budget (iterations) | M4c's code | issue 008 |
|---|---|---|
| 150 | 9 | 7 (another compiled form: 8) |
| 160 | 7 | 6 |
| 170 | 8 | 7 (11) |
| 185 | 8 | 7 |
| 200 | 8 | 5 (9) |
| 400 | 6 | 6 |
| all | **40 of 60, failed 28 %** (283 / 1013) | **32 of 60, 27 %** (268 / 977); other form 28 of 36, 17 % |

(The "other compiled form" is the same rows written `1 / fmin(share, 1)` before the 0.5 share
threshold: identical values on ice, a different last bit in the compiled code. Its runs differ
from the shipped form's corner by corner: ice's corners are chaotic down to the last bit.)

- **The budget does not decide ice's finishes.** M4c's code finishes 6-9 of 12 at every budget
  from 150 to 400 iterations (2.3 times the budget), with 26-34 % of re-plans failing at any
  of them. The failing windows mostly do not converge at all (infeasible from the car's state,
  restoration failures), and the corners lost with few failed re-plans are lost by the car
  leaving its plan (lateral error 8-10 m at `ice:25:180:15` with 2-3 failed re-plans, spins):
  L2's matter, as issue 007 is on packed snow.
- **Issue 008's fix** removes half the locally infeasible windows offline, but in the loop
  the finishes are the same within the corners' chaos (40 against 32 and 28 of 36).
- **Freeing the first stages' slope rows** (offline the best: 112 of 119 converge): ice 15 of
  36 finished (6, 4, 5 at 170, 150, 200), and Tsukuba changed (74.50 s against 74.33 in the
  repeatable lap). Rejected.
- **Multi-start arms**, 170 iterations: sched 8 -> 7, failed 56/198 -> 42/196; always 6 -> 6,
  61/196 -> 45/192.
- **A same-node re-plan warm from the plan** (the start's re-plan and a crawling car's start
  from the cold guess with a 1.75 m jump at the second node; tried as a fix): wall-clock A/B,
  interleaved, 13 of 32 finished against 25 of 33. The cold guess there works as a second start
  (one corner: 24.0 s plan against 24.4 from the plan). Rejected; not written as an issue.
- **The cut warm start (issue 005) with issue 008's rows**: 17 of 24 against 18 (wall clock,
  load 4.7-6.1): still off on ice.

Wall-clock runs (single, ice's 12 corners): M4c's code 15 of 24 (load 3.8-8.0, failed re-plans
26 %); issue 008 (the other compiled form) 18 of 24 (5.0-6.1, 17 %), 25 of 33 (5.3-12.6, 22 %),
7 of 12 (5.6-7.2, 38 %). M4c: 8, 10, 6 of 12.

**No regression elsewhere.** Issue 008's rows change only surfaces peaking at under half the
rig's slip (ice, sand). Haruna and Tsukuba, single and sched, repeatable laps: the same plan and
plant digests before and after (76.28, 74.33 s): R6 as M4c measured it. Packed snow and mud:
the same runs bit for bit (packed snow 10 of 12, mud 11 of 12, repeatable at 170 iterations).
Packed snow with its rows scaled too (share 0.59) lost 3.6 s on R 9 x 180 at 15 m (11 failed
re-plans against 0), hence the threshold. Suite: 154 pass.

## 4. Gate

| criterion | M4d | |
|---|---|---|
| ice corners finished, better than M4c (single 8, sched 6, always 6) | single 32-40 of 60 either code, 5-9 of 12 at any budget; sched 7, always 6 (repeatable, 170) | **not met** |
| failed re-plans on ice (M4c 55/56/31) | single 41, sched 42, always 45 of ~195 (repeatable, 170); 27 % over five budgets against 28 % | not materially changed |
| R6 Haruna, Tsukuba | bit-identical to M4c's code | met (as M4c) |
| suite | 154 pass | met |

**Verdict: not passed.** Ice's windows do not fit the budget, but the budget is not what ice
loses to: with 2.3 times the iterations the corners finish no more often. What is left is the
car off its plan on ice (L2) and the windows that start from it (infeasible or non-converging
from the car's state at any budget).

## Rulings

- **Issue 008's fix shipped although ice's finishes do not move.** It is a bug (the rows twelve
  peaks out where two were meant), it halves the locally infeasible windows offline, and it
  changes no other surface the gate measures. Cost if wrong: ice's single start 32 against 40
  of 60 is a real loss, not chaos (the other form of the same rows: 28 of 36).
- **Repeatable runs (`RP_DET` + `RP_ITERCAP`) stand in for wall-clock timing runs** while the
  host's load was 15-87. Cost if wrong: an iteration count is not the wall clock (5.5-6.9 ms an
  iteration seen); the loop's latency is the fixed one, not the solve's.
- **Ice's corner outcomes are read as chaotic** (one corner flips with the last bit of compiled
  code); 12-corner samples are compared pooled over budgets, not one by one.
- **Fatrop, SQP and per-iteration levers not pursued further**: the iterations (200-900), not
  the iteration, are the gap, and more iterations do not finish more corners.

## Left

- Ice's tracking (L2): the corners lost with few failed re-plans (with issue 007).
- The windows infeasible from the car's own state at stage 0 (the slope rows): a plan that
  admits the car's slide at its start without planning past the peak (freeing the rows was
  worse).
- An HPIPM-shaped window (the request as a state) if an SQP is wanted later.
