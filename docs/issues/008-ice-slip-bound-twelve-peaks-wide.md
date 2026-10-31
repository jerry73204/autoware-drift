# 008: on ice the slip-ratio rows sit twelve peaks out: IPOPT strands there, locally infeasible

**Severity:** high on ice (a third of ice's re-plans fail, M4c; of the 119 recorded ice windows
not solved within the budget, 56 end `Infeasible_Problem_Detected` even unbudgeted).
**Status:** fixed (roadmap 061 M4d); ice's closed-loop finishes unchanged (see Measured).

## What is wrong

L1's window bounds every wheel's slip ratio within `KAPPA_MAX` (0.6; a front wheel's upward
`FRONT_KAPPA_UP`, 0.05) and holds it on the rising branch by the slope rows (dFx/dkappa at least
`SLOPE_MIN` of the stiffness). `KAPPA_MAX` is absolute: 2.3 times the rig's longitudinal peak
(0.26), but ice peaks at 0.05 (`surface_classes.yaml`'s `peak_slip`), so on ice the rows allow
twelve of its peaks, nearly all of it on the flat of the fall-off (sliding 0.7), where the
force's slope is near zero and does not change: the slope rows there have no gradient.

## Root cause

Recorded every L1 window solve of all 12 ice corners, single start, two repeats each (413
solves, M4d) and replayed them offline (deterministic to the iteration). At the end of the
windows that fail, IPOPT's iterate has the rear wheels' slip at the 0.6 bound and the rear slope
rows violated (`dFx/dkappa` -0.001 against 0.01; e.g. `ice:15:90:15` repeat 1, solves 13 and 20,
both `Infeasible_Problem_Detected` on the exit straight at 9 m/s, multipliers 1000 on
`slope_rl`/`slope_rr` at the first stages): the iterate wandered past the peak early and cannot
come back, since nothing there tells it which way the rising branch lies. The window's own
comment at `KAPPA_MAX` names this failure for KAPPA_MAX 1.0 on asphalt; ice's peak puts 0.6
further out than 1.0 was there.

## Fix

The slip-ratio rows in the surface's own peak where it peaks at under half the rig's slip
(`window.kappa_rows_by_peak`, `KAPPA_SHARE_LOW` 0.5: each row over the peak share): on ice
0.6 x 0.196 = 0.117, 2.3 of its peaks, as on asphalt. Of swb's classes ice (0.20) and sand
(0.47); every other surface's rows unchanged, bit for bit (Haruna and Tsukuba, single and
sched: the same plan and plant digests as before; packed snow and mud the same runs). Scaling
packed snow's too (0.59) cost one of its twelve corners 3.6 s (R 9 x 180 at 15 m, 11 failed
re-plans against 0), so it is not. Test `test_window.py::
test_the_slip_ratio_rows_are_in_the_surfaces_own_peak_where_it_peaks_earlier`.

## Measured

- **Offline** (the 413 recorded solves): a plan within 170 iterations (the 1 s budget's, the
  replay reproducing the loop's own 292 of 413) 294 -> 313; of the 119 the old rows fail at
  170, unbudgeted to 1500 iterations: 58 -> 83 converge, `Infeasible_Problem_Detected` 56 -> 30.
- **Closed loop** (ice's 12 corners, single start, repeatable runs with the budget as 150-200
  iterations, `RP_DET=1 RP_ITERCAP`): finished 40 of 60 before, 32 of 60 after (another
  compiled form of the same rows: 28 of 36); failed re-plans 28 % before, 27 % and 17 % after.
  Within the corners' own chaos (one corner's outcome flips with the last bit of the compiled
  code), no change in what finishes.

## Not the fix: the free stages' slope rows

What is left infeasible after the fix is the first stage's slope rows: a wheel's slope depends
on its slip angle too, and at stage 0 that is the car's (`ice:9:180:6` repeat 2, solve 3:
`slope_rr` -0.08 against 0.01, nothing else violated). Freeing the first three stages of the
slope rows as of the slip rows (issue 003) converged 112 of the 119 offline (3 infeasible) and 325 of
413 within 170 iterations, but drove worse (ice 15 of 36 finished against 28; Tsukuba 74.50 s
against 74.33 in the repeatable run): plans starting with a wheel past its peak are not plans
the car can follow. Not adopted.
