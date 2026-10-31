# 005: Tsukuba s 1815: a solve cut at the budget is thrown away, the next restarts further back

**Severity:** high for R6 (Tsukuba's lap is lost in one to three of three repeats in every arm,
M4 and M4b; the time comparison R6 asks for cannot be made). **Status:** fixed (roadmap 061
M4c).

## What is wrong

Roadmap 060/061: on Tsukuba the car leaves the road at s ~1820 after four solves in a row cut at
the 1 s budget between s 1514 and 1724 (the end of the back straight, 50-54 m/s, braking into
the R 85-25 sweepers at s 1750-1830). Which repeat crosses the budget is the wall clock's.

## Root cause

Recorded the loop's every window solve (inputs, multipliers, outcome) in a failing run
(M4c, `tsukuba` single, load 5.3) and replayed them offline, deterministic to the iteration:

- The four cut solves need 197, 197, 564 and 782 iterations unbudgeted (5.5-6.5 ms each on a
  300 m, 75-stage window); the one before them 100. Not the multipliers (none: 219, 238, 474),
  not IPOPT's tolerances.
- **A cut solve is discarded whole and the next one restarts from the last adopted plan**,
  shifted 13 nodes further on, with a longer cold tail (the tail's columns: 28, 40, 54, 65 of
  75), while `Window.solve` keeps the cut solve's multipliers for the next start: primal from
  the stale plan, duals from the cut iterate. The cut iterates were nearly feasible (largest
  row violation 0.016-0.026 of its limit, shooting gap 0.001-0.002), 1.5 times the 1e-2 that
  makes a cut solve a plan.
- Chain replay of the same four calls under the 1 s budget: today's rule cuts all four; the
  next solve warm-started from the cut iterate (shifted as a plan is, with its own multipliers)
  converges in 48 iterations (0.25 s).

- IPOPT's iterate is not monotone in feasibility there: the first cut solve stopped at 160
  iterations is feasible (0.003), at 164 0.098, at 166 0.026, at 170 0.09, at 180 0.043. Where
  the wall clock stops it decides whether the cut is "near feasible". Warm-started from the
  iterate at 150, 164 or 170 iterations (0.09-0.098), the next solve converges in 151, 73 and
  135 iterations under the budget; from the stale plan it is cut again (158).

The cold tail is also inconsistent (the minimum-curvature guess at zero slip makes no lateral
force: 8 m/s^2 of lateral acceleration short at the R 85 sweeper), but giving it the steady
slip angles moved the first cut solve by nothing (203 against 197 iterations).

## Fix

`live.Driver` keeps a cut G solve's iterate when it is near feasible (row violation and
shooting gap within `CUT_WARM`, 0.2: a 0.05 bound took the iterate in one of four
recorded runs, the clock's cut point deciding) and warm-starts the next G solve from it instead of the
adopted plan (both the single start and the scheduler's G); dropped on the next solve. Not on a
track with ice (`rear_slip_low` in effect, `Driver.cut_warm`): on the rear's falling branch the
cut iterate led the next solves further off (ice's 12 corners, single start, one repeat, load
4-5: 10 finished without it, 6 with it; with issue 006's fix in both). A cut
iterate is never adopted (the car still follows its last plan). Test:
`test_live.py::test_a_cut_solve_warms_the_next`.
