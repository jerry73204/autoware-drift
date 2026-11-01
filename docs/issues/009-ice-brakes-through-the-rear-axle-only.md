# 009: on ice the car brakes through its rear axle only; L1 brakes with the front tyres steered to lock

**Severity:** high on ice's 6 m roads (R 9 x 180 and R 15 x 180 at 6 m lost in every M4e run
with issue 008's rows; the 6 m roads hold 44 of the 49 corners ice lost in M4e).
**Status:** open. The cause is the car (its ECU's braking on its spec), not the planner; a fix
needs a ruling (roadmap 061 M4f, `docs/mode-choice-m4f.md`).

## What is wrong

M4e left ice two losses: R 9 x 180 at 6 m (L1: its windows need 260-1200 IPOPT iterations from
the car's state, the budget ~170) and the 6 m roads' "L2" losses (the car leaving the road
0.4-3.8 m off its plan, no runaway slide). Both have one cause.

## Root cause

**The car brakes on ice through its rear axle only, at about 0.46 of the road's friction.**
The drift car's ECU (sim_vehicle_ecu's ev powertrain) regenerates first, up to its motor's
regen torque, 150 N m x 6 / 0.32 m = **2812 N** at the rear tyres, and adds the friction
brakes only beyond that. The rear tyres on ice carry about **1630 N** (0.265 x 0.92 x the rear
axle's 6665 N under braking). So all the braking the rear can carry is regen on the rear; any
more drives the rear past its grip, and the friction brakes (the front) join only above
2812 N. The spec has ABS off (`abs_enabled: false`, a drift car) and the regen torque is not cut
when the rear locks: in the plant (sim.Plant, the ECU itself) a request of -2.4 m/s^2 on ice
turns the rear wheels backwards (-32 m/s peripheral within 0.8 s, regen 2812 N throughout).
L1's axle model is the ECU's rule (`axle.brakes`: rear = max(regen, (1 - share) B)) and its
slope rows hold the rear on its rising branch, so L1 plans at most the rear's grip: a request
of 1.1-1.3 m/s^2 on ice against the 2.39 the road allows.

**The corner's start speed assumes more.** `drift_optimality.v_start`: the corner's
point-mass speed plus 0.6 mu g of braking over the 50 m straight; R 9 on ice, 12.85 m/s. At the
rear's 0.46 mu g the car needs about 9 m more than it has.

**So L1 finds another brake: the front tyres steered to lock.** Ice plans steer the front
wheels to a slip angle of 30-58 deg, 10-19 times ice's peak (3 deg), left and right in turn
on the straight, where ice's sliding share (0.7 of the peak) makes them drag (first plan of
`ice:9:180:6`: delta -36, -39, +42, +49, +58, +58 deg at s 12-68). Every ice corner's first
plans do it (control runs, 8 corners: 31-57 deg in the braking zone on the 6 m roads, 12-55
on the 15 m ones, 2-9 deg once the car is slow). The front slip rows (issue 002) bound the front only on surfaces without
fall-off; on ice "the model's own fall-off" was to keep plans out, and with the rear unable to
brake it does not. On a 15 m road the plans' and the car's errors fit; on 6 m they do not.

**L1 (R 9 x 180 at 6 m).** Even scrubbing, the corner is at or past the car from its start
speed: the first plan's rescue (G with 3000 iterations, H, G from H, D, F) ends **off the road
in 5 of 5 starts** (8-15 m outside). IPOPT finds an off-road compromise of the soft edges (10 s
a metre), 8-12 m out, or a near one 0.3-0.5 m out, by the last bits (issue 008's rows or not,
the edge weight 10 or 1000: each form lands in either). The loop adopts the off-road one (its
status is Solve_Succeeded), the car drives off its edge, and every warm window starts from a
plan 8-12 m off the road: the warm start 8.6 m past the edge rows, 250-1230 iterations
unbudgeted (M4e's 260-1200), most ending off the road again or locally infeasible (the slope
and front-balance rows at the first stages). Started 10 % slower (11.56 m/s, the study's retry
when no first plan is accepted), the windows on the straight solve in 41-159 iterations; in
the hairpin (the car at its rear peak, the front at 25-50 deg) they still need 177-1549.

**L2 (the 6 m losses).** The 17 corners M4e lost with 3 or fewer failed re-plans, re-run bit
for bit and traced: 15 on 6 m roads. In each of the 15 the plan's front slip angle in the last
3 s was 11-58 deg (4-19 x the peak; 12 of 15 above 20 deg); the car leaves 0.4-3.3 m off the
plan, 7 inside and 8 outside; L2's edge rows (1e4 a m^2) are violated in its own prediction
(slack 0.6-3.0 m) when it solves. It is not an edge margin or a line weight: L2 follows a
front at 40-58 deg, where the steering barely changes the front's force, and has no lateral
authority left. `ice:15:180:6` (M4e `d170c78`): the plan steers 54-57 deg on the straight at
9 m/s, the car runs 1.3 m inside the plan's line and off the inside edge at s 63.6, L2 at the
lock. `ice:25:180:6` (`d170c78j1`): 37-39 deg, then the rear slides 12 deg past the plan's 2.
The plans reach their edge bound (1.75 m) in 3 of the 15 and pass it in 3 (2.1-2.9 m); in
the other 9 they keep 0.9-1.6 m from the centre.

## Measured: the car with regen within its rear's grip (experiment, not shipped)

The plant's ECU's regen capped at 1100 N (0.7 of the rear's grip on ice; `sim_vehicle_ecu.
powertrain.regen_limit` patched in the study) and L1's axle model told the same (regen within
0.7 of the rear axle's grip at the planned friction). Ice's 12 corners, single start,
repeatable (`RP_DET`), budgets 150, 170 (two draws) and 200: **10.5 of 12 against M4e's 9.5**
on the same replicates (+1.0, se 0.71); the 6 m roads **22 of 24 against 14**, R 9 x 180 at
6 m 3 of 4 against 0, R 15 x 180 at 6 m 4 of 4 against 0; the 15 m roads 20 of 24 against 24
(R 9 x 180 and R 15 x 180 at 15 m 2 of 4 each). Its plans steer the front at 4-18 deg, the car
at most 10-11 deg (against 36-57), lateral error 0.45-0.71 m on R 15 and R 25 x 180 at 6 m.
A car change (sim_vehicle_ecu, the spec): not the planner's to make.

## Not the fix

- **The front's slip rows on ice** (the front at 1 or 2 of its peaks where issue 003's rear row
  applies): R 9 x 180 and R 15 x 180 at 6 m, 1 of 8 finished, re-plans failing 7 of 7 (the
  plan has no brake left, and the car's own state is past the bound).
- **L1 told regen stops at the rear's grip, the car not**: the plan brakes at 2.3 m/s^2 and
  the plant's rear locks (ABS off): off the road or spun on the straight, 1 of 8 finished.
- **A plan off the road not adopted** (its car's body more than 0.5 m past the edge, live.run's
  off-road rule, past the first three nodes: `Off_Road_Plan`): ice's 12 corners, 2 draws at
  170 iterations, 19 of 24 as M4e's (20 runs bit-identical, 4 the same outcome); the corner
  itself 0 of 2 (the first plan is then made 10 % slower, and the hairpin's windows still
  fail).
  Right in principle (a plan whose own car leaves the road is not a plan), no gain: not
  shipped.

## For a fix (ruling)

- **The car**: regen limited by what the driven axle can carry (the ECU's friction estimate, or
  ABS working the motor's torque as it does the brakes), and `axle.brakes` given the same rule.
  The experiment's fixed 1100 N cap is the size of the effect, not a design: it brakes
  asphalt and every other surface the same way.
- **Or the study**: start speeds from the car's own braking (rear-only on ice) rather than
  `v_start`'s 0.6 mu g. That changes every ice corner's start, so every earlier comparison.
