# 006: a plan's feasibility check counts the slip rows the first stages are free of

**Severity:** high on ice (a third of ice's re-plans fail, M4b), medium on mud and loose snow.
**Status:** fixed (roadmap 061 M4c).

## What is wrong

M4b (issue 003) freed the window's first `FREE_STAGES` stages of the slip and held rows: the
car's own state may be past a bound. `Window._result` judges a solve that stopped short (the
1 s budget, the iteration limit) a plan when it is feasible, its rows within 1e-2 of their
bounds; it evaluated every stage's rows against the bounds, the slip rows of the free stages
too. A cut solve whose only "violation" is the car's own slip at the window's first nodes is
thrown away, and on ice, where the rear is held at its peak and the car is past it as soon as
it slides, that is most cut solves. The reported `violation` of a successful solve read up to
4.0 (`ice:9:180:6`, a solve IPOPT called `Solve_Succeeded`).

## Root cause

`_result` did not know which rows `__init__` had relaxed (the last `nk` rows of each of the first
`free_stages` stages' `g`).

## Measured

`ice:9:180:6`, single start, the loop's ten re-plans from s 40 recorded (M4c, load 4-5) and
replayed in a chain under the 1 s budget, the warm starts as the loop makes them: plans 1 of 10
with the old check, 4 of 10 with the free stages' slip rows left out (the cut solves' violation
0.006 where it read 0.15-5.6).

## Fix

`Window` keeps `free_stages` and `nk`; `_result` leaves the free stages' slip rows out of the
check. Test: `test_window.py::test_the_free_stages_slip_rows_are_not_counted_against_a_plan`.
