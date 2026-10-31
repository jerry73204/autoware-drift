# 007: on packed snow the car's slide runs away from a modest planned slide

**Severity:** medium (packed snow's R 15 x 90 and R 25 x 90 corners at 15 m wide stop in every
arm and every configuration since M4: 2 of 12 corners lost per arm). **Status:** open (an L2
matter: roadmap 060 R3, the tracker holding the plan's sideslip).

## What is wrong

`snow_packed:25:90:15` stalls at s 115, `snow_packed:15:90:15` stops at s 108 (spin), single,
sched and always alike, in M4, M4b (`a`, `rl1`) and M4c.

## What M4c measured

M4c trace (single start, every tick, load 4.5-7.7): every re-plan solves (7 of 7 and 7 of 9,
0.07-0.9 s), the plans' sideslip stays at 6-8 deg, the car's grows monotonically from the
turn-in: 25:90:15, s 60 10 deg (plan 5.7), s 75 17, s 90 20, s 103 28, s 110 48, s 113 56 deg,
the speed falling from 9.0 to 2.3 m/s against the plan's 7.7-9.9, L2's steering 4-32 deg; the
car stalls sideways. 15:90:15 the same, then spins. Packed snow falls past its peak (sliding
share 0.8): a slide a few degrees past the plan's loses rear force, as on ice (issue 003, its
point 1), but holding the plan's rear at its peak (M4b `rl1`) did not save these corners. The
plan is feasible and kept; the car does not stay on it.

## Root cause (issue 007's investigation, 2026-10-31)

L2's cost, not its model, its convergence or its constraints. Traced every tick of both corners
offline (single start, `RP_DET=1`, so the runs repeat bit for bit: 25:90:15 stalls at s 114.7,
15:90:15 spins at s 109, as in M4c), with L2's prediction, references, inputs and the plant's
wheels recorded (logs: swb `play_log/race_planner/mode_choice/i007/trace/`):

- **The model is right.** L2's predicted sideslip one stage ahead matches the car's to 0.1-0.5
  deg the whole way, 10 to 56 deg; its yaw rate five stages (0.2 s) ahead to 0.01 rad/s. The
  plant's tyre is L2's (model.tyre, the same surface), and with the wheels' speeds as inputs
  the axle model tracks the two-track plant closely even far past the rear's peak.
- **The solver converges to it.** Five SQP iterations a tick instead of RTI's one: the same
  runaway (25:90:15 spins at s 117, 15:90:15 off the road at s 110). No QP failed, no
  fallback ticks on 25:90:15, 6-16 HPIPM iterations.
- **L2 chooses the slide.** Its own solution predicts the runaway: at s 55 its horizon ends at
  13 deg against the plan's 6, and it steers 11-12 deg where the plan steers 3.4, braking
  (-0.8 m/s^2, all of it on the rear as regen: rear slip ratio -0.14, packed snow's peak
  0.15) where the plan asks -0.4. The cost's terms at its solution: the line's offset n
  540-690 against the sideslip's 34-84 and the yaw rate's 3 (s 55-62); later n 1500-9000.
- **Why:** the line's scale, 0.2 m (an error of 0.2 m costs as much as 0.8 m/s of sideslip),
  was set on asphalt. On packed snow (mu 0.2, a tenth of asphalt's lateral acceleration) the
  car can move itself across the road only slowly; within L2's 1 s horizon the fastest
  handle on n it has is the car's rotation. So a few tenths of a metre of predicted line
  error buy steering into the turn and rear braking, the slide grows past the rear's peak
  (6 deg on packed snow, the falling branch: sliding 0.8), the rear loses force, the car slows
  (9.6 against the plan's 10.1 m/s), its path tightens, the line error grows, and L2 rotates
  more: a loop that ends sideways. The re-plans keep starting from the car and plan 6-8 deg;
  every one is tracked the same way.
- **Check:** the line's scale alone, everything else as it was: 0.5 m or 1.0 m, both corners
  finish (car's sideslip 5.9-8.1 deg against the plans' 6.1-6.4); the slide's scales tighter
  (vy 0.3, r 0.15) with the line's at 0.2: 15:90:15 still stalls; the speed's looser (2.0)
  with the line's at 0.2: both lost. (`i007/sweep1`.)

**Fix:** the line's scale grows as the friction falls, 0.2 m x max(1, 1 / mu) at each stage's
friction (the map's times the car's estimate, what L2 already models): asphalt (1.12) and
concrete keep 0.2 m exactly, packed snow 1.0 m, ice 0.75, mud 0.69, loose snow 0.8.

## The chain's packed-snow hairpin (R 25, s 1200-1280)

M4b: two of eight true-map chain runs stopped there (s 1238, 1249). M4c: 0 of 4 true-map runs
(sched 2, always 2), after issue 006 (M4b's sched run that stopped there failed 7 re-plans and
was switched from drift to grip at s 1236 for want of a drift plan: D's cut solves were judged
infeasible by the free stages' rows). One M4c run (always r1) drove it with the steering at its
lock for 1.3 s (s 1200-1213: the car at 1.5-5 deg against the plan's 9.7, 1.0 m wide of the
plan): the hairpin is still driven at the steering's limit.
