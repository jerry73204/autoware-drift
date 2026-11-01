# 009: on ice the car brakes through its rear axle only; L1 brakes with the front tyres steered to lock

**Severity:** high on ice's 6 m roads (R 9 x 180 and R 15 x 180 at 6 m lost in every M4e run
with issue 008's rows; the 6 m roads hold 44 of the 49 corners ice lost in M4e).
**Status:** fixed in the car (user's ruling: option 1), roadmap 061 M4g
(`docs/mode-choice-m4g.md`): sim_vehicle_ecu's regen slip control, the planner's axle model
following it, L2's front wheels held tighter where it applies. See "The fix".

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

## The fix (M4g)

**The car** (sim_vehicle_ecu `ev`, as a real EV's brake module blends regen; README "Regen
slip control"; all from the module's own estimates):
- the regen beyond the driven axle's share of the brakes (by the brake distribution) is cut
  while a driven wheel slips (the ABS/TC modulator on the motor's torque); the friction brakes
  take what it gives up. Released past 0.25 slip, or on a runaway wheel: past 0.06, slowing
  3 g faster than the car, its axle carrying 30 % less than the brakes put on it (each alone
  fires on dry asphalt; together they did not on Haruna or Tsukuba). Regen within the axle's
  own share is left: cutting it hands it to the same axle's friction brakes;
- in such a slip event the axle's grip is measured from the wheels' own balance (brake torque
  and drag less what their inertia gives up; the car's deceleration lagged the tyre and once
  read 0) and kept (its highest in the event; forgotten at 0.001 /s, never below what the axle
  carries unslipping); regen beyond the axle's share is then held within 0.7 of that grip.
On ice a request of -1.8 m/s^2 now finds 0.227-0.243 (ice's peak 0.265) within 0.1 s and the
rear rolls (slip within 0.02); before, the rear turned backwards. **ABS stays off** in the spec:
not needed (the rear's own share at the static 50:50 split is within its grip below ~2.1 m/s^2,
and the planner keeps its tyres on their rising branch).

**The planner**: `axle.dynamics` holds regen within `car.regen_grip_share` (0.7) of the rear's
grip at the planned friction, smoothly (`_smin`), where the road is `held`: slick enough for the
motor's full regen to slip the rear (`axle.regen_slip_mu`, 0.42 on drift_mustang) and falling
off past its peak as ice does (sliding at most 0.75). So rear = max(min(regen, 0.7 mu Fz_r),
(1 - share) B), front = B - rear. Where no node is held (`axle.regen_hold_car`, live.Driver and
the trackers) the model is the old one expression for expression: Haruna and Tsukuba `RP_DET`
digests equal to M4e's. Not on snow or mud: there the hold's plans used the fronts in a
braking drift that left the 6 m roads (packed snow 28 of 36, loose 24, against 36 each), and
the old plans brake the rear within its grip, where the car does not need the hold. L2's front
wheels' proximal term is 0.3 m/s (1.0 elsewhere) where the hold applies: their brakes then
work on ice at a front past its 3 deg lateral peak, where the balance is flat in the wheel's
speed (W_PROX 1.0: the predicted front speed swung 5-10 m/s tick to tick, HPIPM failed every
other tick, the LQR's +0.5 m/s^2 drive in between; 15 m roads lost).

**The study**: a car on ice has braked on it before the corner, so its ECU knows the
rear's grip (`sim.Plant.regen_grip_found`: braking at -1.8 m/s^2 for 0.6 s on the course's
start; result JSON `ecu_primed`). Without it the corner's first braking was the car's probe
while the plan already assumed the hold: 4 of 12 ice corners (M4e 8) at 150 iterations.

**The crude cap's 15 m losses (M4f)**: its model's hard `ca.fmin(regen, cap)`. L2's RTI
linearises at the kink and HPIPM stopped at its minimum step (status 2) every other tick: 136,
97, 33 and 340 LQR ticks in the four lost runs (0-3 in M4e's); with a smooth min the same four
finished, failures 0-17.

**Measured** (M4g, repeatable runs, M4e's 20 / 10 replicates; `docs/mode-choice-m4g.md`): ice
single 10.90 of 12 against 9.55 (+1.35, 6.1 se), sched 10.30 against 9.80 (+0.50, 1.5 se); the
6 m roads 108 against 76 of 120 (single), L2's losses gone; R 9 x 180 at 6 m 8 of 20 (0 before);
R 9 x 180 at 15 m worse (10 of 20 against 17, L1 at the hairpin's entry, now hotter). Haruna and
Tsukuba bit for bit; packed and loose snow and mud finished as before.

## For a fix (ruling, M4f)

- **The car**: regen limited by what the driven axle can carry (the ECU's friction estimate, or
  ABS working the motor's torque as it does the brakes), and `axle.brakes` given the same rule.
  The experiment's fixed 1100 N cap is the size of the effect, not a design: it brakes
  asphalt and every other surface the same way.
- **Or the study**: start speeds from the car's own braking (rear-only on ice) rather than
  `v_start`'s 0.6 mu g. That changes every ice corner's start, so every earlier comparison.
