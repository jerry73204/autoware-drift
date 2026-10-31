# 007: on packed snow the car's slide runs away from a modest planned slide

**Severity:** medium (packed snow's R 15 x 90 and R 25 x 90 corners at 15 m wide stop in every
arm and every configuration since M4: 2 of 12 corners lost per arm). **Status:** open (an L2
matter: roadmap 060 R3, the tracker holding the plan's sideslip).

## What is wrong

`snow_packed:25:90:15` stalls at s 115, `snow_packed:15:90:15` stops at s 108 (spin), single,
sched and always alike, in M4, M4b (`a`, `rl1`) and M4c.

## Root cause (as far as measured)

M4c trace (single start, every tick, load 4.5-7.7): every re-plan solves (7 of 7 and 7 of 9,
0.07-0.9 s), the plans' sideslip stays at 6-8 deg, the car's grows monotonically from the
turn-in: 25:90:15, s 60 10 deg (plan 5.7), s 75 17, s 90 20, s 103 28, s 110 48, s 113 56 deg,
the speed falling from 9.0 to 2.3 m/s against the plan's 7.7-9.9, L2's steering 4-32 deg; the
car stalls sideways. 15:90:15 the same, then spins. Packed snow falls past its peak (sliding
share 0.8): a slide a few degrees past the plan's loses rear force, as on ice (issue 003, its
point 1), but holding the plan's rear at its peak (M4b `rl1`) did not save these corners. The
plan is feasible and kept; the car does not stay on it.

## The chain's packed-snow hairpin (R 25, s 1200-1280)

M4b: two of eight true-map chain runs stopped there (s 1238, 1249). M4c: 0 of 4 true-map runs
(sched 2, always 2), after issue 006 (M4b's sched run that stopped there failed 7 re-plans and
was switched from drift to grip at s 1236 for want of a drift plan: D's cut solves were judged
infeasible by the free stages' rows). One M4c run (always r1) drove it with the steering at its
lock for 1.3 s (s 1200-1213: the car at 1.5-5 deg against the plan's 9.7, 1.0 m wide of the
plan): the hairpin is still driven at the steering's limit.
