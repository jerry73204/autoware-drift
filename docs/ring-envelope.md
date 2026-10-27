# Speed, acceleration and the bends: drift_mustang on Town05's ring

Written 2026-10-27 for swb roadmap 057. The numbers come from `tools/ring_envelope.py`, from the
car's spec (sim_vehicle_spec `drift_mustang.yaml`) and the ring's lane measured on the lanelet
map in Autoware's frame. They are a steady-state model, to set the stack's limits and the drift
law's targets; the laps measure what the car really does.

## The ring

The lane the scenario drives (lanelets 123842, 48002, 53191, 129974, 73626) is 1540 m: four
left bends of 90 deg and four straights.

| segment | length | radius |
|---|---|---|
| north straight (x 98 to -138) | 230 m | |
| north-west bend | 168 m of arc | 109 m mean, 104 m least |
| west straight | 196 m | |
| south-west bend | 166 m | 107 m mean, 102 m least |
| south straight | 238 m | |
| south-east bend | 170 m | 111 m mean, 106 m least |
| east straight | 196 m | |
| north-east bend | 170 m | 111 m mean, 106 m least |

The lane is 3.5 m wide. The ring's road (OpenDRIVE roads 34, 37 and 38) is a 65 mph road
(104.6 km/h, swb's `repair_lanelet_speed_limits.py` writes it on the lanelets), and part of the
west side (road 37) is raised, up to 10 m: the walkways under it lie flat on the lanelet map,
which has no heights, so Autoware's walkway module stops the car in front of each one (lap
12: the stop-and-go down the west side). The lap scripts launch the ego without that module
(`launch_walkway_module:=false`).

## The car

1450 kg, 50:50, the centre of mass 0.48 m high on a 2.886 m wheelbase; a 330 kW rear motor
(700 N m to 4500 rpm through 6:1, 13.1 kN at the rear tires below 90 km/h), a locked rear
axle, no ABS, traction control or ESC. On swb's asphalt class (the surface map,
`config/surfaces/town05_surfaces.yaml`) the tire's peak friction is 1.12 and past the peak it
falls toward 0.8 of it with the combined slip, so a sliding tire has 0.9.

## Formulation

### Grip

A car on grip through a bend of radius R is limited by `V^2 / R <= mu g`:

    V_grip = sqrt(mu g R) = sqrt(1.12 x 9.81 x 105) = 34.0 m/s (122 km/h)

### A steady drift

In a steady drift the car follows radius R at speed V with a constant sideslip beta, so its yaw
rate is V / R and its acceleration a = V^2 / R points at the bend's centre, beta off the body's
lateral axis. Following Velenis, Frazzoli and Tsiotras (2010), on a single-track model with the
front tire rolling free and the rear sliding:

    rear lateral force    f_Ry = m a cos(beta) l_F / L
    rear thrust           f_Rx = m a sin(beta) + drag(V)
    rear load             f_Rz = (m g l_F + m h a sin(beta)) / L
    rear saturated        sqrt(f_Rx^2 + f_Ry^2) = mu_s f_Rz,     mu_s = 1.12 x 0.8
    front, within grip    f_Fy = m a cos(beta) l_R / L <= mu f_Fz

(l_F, l_R: the centre of mass to the front and rear axles; L the wheelbase; h the centre of
mass's height). Without drag the rear's saturation gives the drift's acceleration directly:

    a = mu_s g l_F / L / ( sqrt(sin^2 beta + (cos beta l_F / L)^2) - mu_s h sin(beta) / L )

On the ring's 105 m:

| beta | V | a_y | front friction used | rear thrust | power | yaw rate |
|---|---|---|---|---|---|---|
| 10 deg | 29.9 m/s (108 km/h) | 8.5 m/s^2 | 0.80 | 2.8 kN | 83 kW | 0.28 rad/s |
| 15 deg | 29.4 m/s (106 km/h) | 8.2 | 0.78 | 3.7 kN | 108 kW | 0.28 |
| 20 deg | 28.6 m/s (103 km/h) | 7.8 | 0.74 | 4.5 kN | 128 kW | 0.27 |
| 25 deg | 27.9 m/s (100 km/h) | 7.4 | 0.68 | 5.1 kN | 142 kW | 0.27 |
| 30 deg | 27.1 m/s (98 km/h) | 7.0 | 0.63 | 5.6 kN | 152 kW | 0.26 |
| 40 deg | 25.7 m/s (92 km/h) | 6.3 | 0.51 | 6.4 kN | 163 kW | 0.24 |

So a drift is slower than grip: the sliding rear has 0.9 instead of 1.12, and part of it pushes
the car forward. The more sideslip, the slower the drift and the more power it takes: the
rear's thrust grows with sin(beta), and it has to, because the car's velocity is beta off its
nose. The front never needs all its grip: the margin is what the steering holds the drift with
(it counter-steers more as beta grows). These are the drift law's numbers: its target
sideslip, 20 deg, holds at 28.6 m/s and 0.27 rad/s; the 90 deg bend takes 1.57 / 0.27 = 5.8 s,
168 m at that speed, which is its `hold_time` of 6 s.

Checked against the car: on swb's proving ground (no surface map, the rig's own tire whose
friction falls only to 0.93 at the slip of a drift) the car held 15.2 deg on 97.8 m at 30.3 m/s.
The model gives 28.4 m/s there with the sliding share 0.8, 29.9 with 0.85; the rig's tire slides
at nearer 0.9. With the asphalt class on, the drift should hold about 1.5 m/s slower than on the
proving ground.

A drift also takes room: at sideslip beta the car sweeps `l sin(beta) + w cos(beta)` across
the lane, 3.4 m at 20 deg for this 4.72 m by 1.90 m body, all of a 3.5 m lane. Any error in the
line takes it into the next lane.

### Accelerating and braking

The rear tires take `mu (m g l_F + m a h) / L`, so the traction limit is

    a = mu g l_F / L / (1 - mu h / L) = 6.6 m/s^2

and the motor keeps it there up to about 34 m/s (330 kW / (1450 kg x 6.6)); less drag, the car
makes 6.6 m/s^2 at 10 m/s, 6.4 at 28, 6.0 at 35. Braking: 11.0 m/s^2 at the tire's peak, 8.8
with the wheels locked (no ABS).

### The straights

A straight entered and left at the drift's speed (28.6 m/s) peaks at

    V_peak^2 = (2 s a_acc a_dec + a_dec V_in^2 + a_acc V_out^2) / (a_acc + a_dec)

| limits | 196 m | 230 m | 238 m |
|---|---|---|---|
| Autoware's upstream (1.0 / -1.0 m/s^2) | 31.9 m/s | 32.4 | 32.5 |
| autoware-drift's (3.0 / -2.5) | 36.8 | 38.1 | 38.3 |
| the car's (about 6 / 8) | 46.5 | 49.0 | 49.5 |

## The limits this sets

- **Speed.** The drifts sit at 26-30 m/s on these bends (98-108 km/h at 10-30 deg), so the
  bends need no more than the road's 104.6 km/h: `max_velocity:=29.06` (was 27.78, the user's
  100 km/h), which the lanelets' speed limit caps anyway. Autoware's smoother, at its 8 m/s^2
  lateral limit, reaches a 105 m bend at `sqrt(8 x 105)` = 29.0 m/s: the drift's entry speed,
  with a short trail-brake to the hold's. Faster straights (37-38 m/s with Autoware's 3 / -2.5)
  would need the ring's speed limit overridden on the map, and buy about 2 s a lap; not done.
- **Acceleration.** From the start (x 30) to the first drift point (x -125) is 155 m: at 3 m/s^2
  the car reaches `sqrt(2 x 3 x 155)` = 30.5 m/s, less the jerk-limited start, so the first
  drift is armed above `min_entry_speed` (20 m/s). Lap 11 at Autoware's upstream 1 m/s^2 reached
  16.3 m/s there and armed none.
- **The bends.** Target sideslip 15-25 deg; the hold's speed is the table's, about 28-29 m/s;
  the yaw rate 0.27-0.28 rad/s.
- **The law's geometry.** `Params.a` and `b` are 1.35 m each (a 2.70 m wheelbase); the car's is
  2.886 m, 1.443 m each. Worth setting from the spec.

## Related work

- Steady-state drift: Voser, Hindiyeh and Gerdes, "Analysis and control of high sideslip
  manoeuvres", Vehicle System Dynamics 48 (2010): unstable drift equilibria of a single-track
  model with nonlinear tires, counter-steered, held by a controller linearised about them
  ([TRID](https://trid.trb.org/View/1086179)); Hindiyeh and Gerdes, "Equilibrium analysis of
  drifting vehicles for control design", ASME DSCC 2009.
- Velenis, Frazzoli and Tsiotras, "Steady-state cornering equilibria and stabilisation for a
  vehicle during extreme operating conditions", Int. J. Vehicle Autonomous Systems (2010): the
  balances above, with the rear's longitudinal slip as an input; for a RWD car the feasible
  equilibria are those with front braking or free rolling and rear drive; multiple equilibria
  for one (R, V), counter-steer decreasing with speed at high sideslip
  ([paper](https://dcsl.gatech.edu/papers/ijvas09R.pdf)). Trail braking as minimum-time
  cornering, and its three stages (entry, steady sliding, exit), are Velenis and Tsiotras's and
  You and Tsiotras's ([DCSL](https://dcsl.gatech.edu/papers/cdc09a.pdf)).
- Drifting along a path: Goh, Goel and Gerdes, "Toward automated vehicle control beyond the
  stability limits: drifting along a general path", JDSMC (2019), MARTY (a DeLorean): path
  tracking by the velocity vector's rotation rate, sideslip by yaw acceleration, rear wheel
  speed control; no equilibrium needed
  ([DDL](https://ddl.stanford.edu/publications/journal/towards-automated-vehicle-control-beyond-stability-limits-drifting-along)).
  TRI and Stanford's autonomous tandem drift (two GR Supras, Thunderhill, 2024) with NMPC and a
  learned tire model ([TRI](https://www.tri.global/news/toyota-research-institute-and-stanford-engineering-achieve-worlds-first-fully-autonomous)).
- Drift as racers measure it: Formula Drift cars reach above 1 g laterally in a drift, about 1.3
  g, on tires and power far beyond this car's
  ([MotoIQ](https://motoiq.com/Revenge-of-the-Nerd-End-of-the-FD-Season-and-a-lesson-in-drift-car-dynamics/)).
- Lap time and racing lines: quasi-steady-state lap simulation on a g-g diagram, and the
  minimum-curvature line (Heilmeier et al., Vehicle System Dynamics 2019, TUM's
  global_racetrajectory_optimization); free-trajectory minimum-lap-time optimal control (Veneri
  and Massaro 2019; Massaro and Limebeer's review, 2021)
  ([survey](https://research.unipd.it/bitstream/11577/3389257/5/2021%20-%20Minimum-lap-time%20optimization%20and%20simulation%20PP.pdf)).
  docs/related-work.md has the racing stacks.
