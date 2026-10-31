# 008: on ice the slip-ratio rows sit twelve peaks out: IPOPT strands there, locally infeasible

**Severity:** high on ice (a third of ice's re-plans fail, M4c; 34 of 119 of the failing ice
windows end `Infeasible_Problem_Detected` or at the iteration limit even unbudgeted).
**Status:** open (roadmap 061 M4d).

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

The slip-ratio rows in the surface's own peak where it peaks earlier than the rig's (`window.
kappa_rows_by_peak`: each row over min(1, peak share)): on ice 0.6 x 0.196 = 0.117, 2.3 of its
peaks, as on asphalt. The reference surface's rows are unchanged (share 1), so are loose
snow's and gravel's (later peaks).
