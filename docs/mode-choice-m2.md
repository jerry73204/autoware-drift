# The planner's tyre on each surface's own peak (roadmap 061 M2)

Written 2026-10-30. Until M2 the planner (`model.tyre`, and everything built on it: axle.py,
L1's window, L2's tracker, `sim.Plant`, `modes.rear_peak`) gave every surface the PhysX rig's
own stiffness: a class's friction moved the peak force, but its slip stayed proportional to mu,
so on packed snow (mu 0.2) the planner's tyre peaked at 2 deg where Isaac's peaks at 6. swb's
surface layer does not do that: it rescales each wheel's stiffness so PhysX's tyre peaks at the
class's own `peak_slip` and `peak_slip_angle`. M2 makes the planner's tyre do the same, checks it
against swb's code, and re-runs the drift study's low-friction arms on it.

Code (autoware-drift): `model.tyre(..., surface=...)`, `track.SurfaceTable.tyre`, the four
`Track` fields `stiff_long, stiff_lat, peak_long, peak_lat` (`track.TYRE_FIELDS`), carried by
`axle.dynamics`, `window.node_functions`/`Window` (the stage's parameters 4 -> 8), `ocp`,
`tracker` (10), `sim.Plant`, `modes.rear_peak`/`past_peak`/`Corner`, `multistart`, `steady`;
`test/test_surfaces.py`; the study's grids `m2-lowmu`, `m2-classes`, `m2-classes-old` and
`m2-report` in `studies/drift_optimality.py`. Results: swb's
`play_log/race_planner/drift_study/m2-*.csv|npz|log` and `m2_report.md`.

## What swb does (the ground truth: the code Isaac runs)

`swb_isaac_world/surfaces.py`, `WheelSurfaces.update`, every physics step, per wheel (`surface_layer
.py` calls it and writes the results with `set_wheel_friction` and `set_wheel_stiffness`):

1. **The peak's friction**: `mu_peak = tire_friction * friction_scale * mu_class * wet / mu_ref`
   (blended over 0.2 m of travel); dry asphalt (the reference, 1.12) gives the rig's own
   `tire_friction`.
2. **Where the tyre peaks**: the class's `(peak_slip, tan(peak_slip_angle))`, or the rig's
   `rig_peaks * tire_friction` where the class gives none. `rig_peaks` (surface_layer.py) is
   PhysX's knee at friction 1: `(2.3 m g/4 / (0.9 long_stiff_value g), 2.3 / (0.9
   lat_stiff_value))`; for drift_mustang (1450 kg, long 3625, lat 17, friction 1.0): kappa
   0.2556, 8.55 deg.
3. **The stiffness**: PhysX's tyre saturates near `2.3 mu F_z / C`, so each stiffness is scaled
   by `rig_peak * mu_peak / class_peak` (lateral, longitudinal) and PhysX's knee lands at the
   class's peak whatever the friction. A class with no peak of its own keeps the rig's slip and
   scales the stiffness with its friction (concrete 0.94).
4. **Past the peak**: rho = |(kappa / kappa_p, tan alpha / tan alpha_p)|; the friction written
   is `mu_peak (sliding + (1 - sliding) / rho)` for rho > 1, Stribeck-lowered on ice with the
   sliding speed, lagged over the class's relaxation length, and divided by the rig's
   friction-vs-slip graph, which PhysX multiplies back.

Per class (drift_mustang; from swb's code, `test_surfaces.py` checks every row):

| class | mu | sliding | peak slip | peak angle (deg) | stiffness lon | lat | peak share lon | lat |
|---|---|---|---|---|---|---|---|---|
| asphalt | 1.12 | 0.80 | 0.256 (rig) | 8.55 (rig) | 1.000 | 1.000 | 1.000 | 1.000 |
| concrete | 1.05 | 0.80 | 0.256 (rig) | 8.55 (rig) | 0.938 | 0.938 | 1.000 | 1.000 |
| gravel | 0.60 | 0.90 | 0.30 | 12 | 0.456 | 0.379 | 1.174 | 1.414 |
| dirt | 0.68 | 0.85 | 0.20 | 10 | 0.776 | 0.518 | 0.783 | 1.173 |
| grass | 0.55 | 0.75 | 0.15 | 9 | 0.837 | 0.466 | 0.587 | 1.054 |
| sand | 0.265 | 0.95 | 0.12 | 14 | 0.504 | 0.143 | 0.470 | 1.659 |
| sand_loose | 0.245 | 0.95 | 0.12 | 14 | 0.466 | 0.132 | 0.470 | 1.659 |
| mud_wet | 0.29 | 1.00 | 0.20 | 12 | 0.331 | 0.183 | 0.783 | 1.414 |
| mud_soft | 0.26 | 1.00 | 0.20 | 12 | 0.297 | 0.164 | 0.783 | 1.414 |
| snow_packed | 0.20 | 0.80 | 0.15 | 6 | 0.304 | 0.255 | 0.587 | 0.699 |
| snow_loose | 0.25 | 1.00 | 0.30 | 12 | 0.190 | 0.158 | 1.174 | 1.414 |
| ice | 0.265 | 0.70 | 0.05 | 3 | 1.209 | 0.679 | 0.196 | 0.349 |

(Soil zones are another model, the soil layer's traction; this is a class on friction alone.)

## What the planner does now

`SurfaceTable.tyre(cls, car, friction_scale=1.0)` is swb's rule: the class's peak as a share of
the rig's at the tyre's friction (`peak_long, peak_lat`), and the stiffness share `mu_rel /
peak share` (`stiff_long, stiff_lat`). `model.tyre(..., surface=(stiff_long, stiff_lat,
peak_long, peak_lat))` multiplies the brush's two stiffnesses by the first pair and takes rho over
the rig's peaks times the second; `surface=None` is the code as it was. `model.peaks` now
multiplies the rig's peaks by the tyre's friction, as swb does (1.0 on drift_mustang). Every
layer carries the four shares per node: a Track built from swb's course takes them from its
road's class; any Track without them has all four at 1, the rig's tyre.

The planner's tyre against swb's, pure slip at rest load, every class (`test_surfaces.py`;
the reference is PhysX's tyre (model.tyre with no fall-off, checked on Isaac's dry sweeps in
test_model.py) at the stiffness swb writes under the friction swb writes):

- stiffness shares and peaks equal to swb's to 1e-12, every class; asphalt exactly (1, 1, 1, 1);
- the force curve within 0.6 % of the peak force everywhere (ice; 0.4 % on asphalt): all that
  differs is the planner's softened knee at rho = 1;
- the peak (smallest slip within 0.5 % of the largest force) within 2 % (asphalt 8.65 against
  8.50 deg), its value within 0.6 %; the fall-off at 2x and 4x the peak within 0.5 %;
- where a class falls off (sliding 0.8 and below) the force peaks at the class's angle (asphalt
  8.5, packed snow 6.0, ice 3.0 deg); where it barely does (0.9-1.0: gravel, sand, mud, loose
  snow) PhysX's brush still rises 1.4 % past it, to 1.1-1.3 times the class's angle (swb's 2.3
  mu F_z / C is 98.6 % of the brush's saturation), in swb's model as in the planner's.

The rear tyre's peak the study and the chooser use (`modes.rear_peak`: outer rear wheel at the
limit's load), deg, rig's tyre (before) and the class's own (after):

| class | before | after | | class | before | after |
|---|---|---|---|---|---|---|
| asphalt | 12.10 | 12.10 | | sand | 2.85 | 17.80 |
| concrete | 11.10 | 11.80 | | mud_wet | 3.10 | 16.35 |
| gravel | 7.00 | 15.85 | | snow_packed | 2.10 | 6.35 |
| dirt | 7.75 | 12.85 | | snow_loose | 2.65 | 16.10 |
| grass | 6.35 | 10.10 | | ice | 2.85 | 3.10 |

and for a friction with no class (swb's road condition on asphalt: the rig's peak, the
stiffness scaled with the friction), mu 0.45: 5.05 -> 12.35 / 9.75 / 8.70 (sliding 1.0 / 0.8 /
0.6); mu 0.3: 3.20 -> 11.75 / 9.30 / 8.55. The old tyre's low-friction peak was 2-3 deg on every
loose surface; swb's is 6-18.

## Asphalt is unchanged

The live path (Haruna, Tsukuba: road class asphalt) carries (1, 1, 1, 1). Measured against the
code before M2 (0a9e126) on the same inputs:

- `model.tyre`, `axle.dynamics` (50 random states), `ocp.solve_min_time` (L0 on a bend): **bit
  for bit identical**;
- L1's stage functions: f identical; g 31 of 7,800 entries differ, by at most 1e-14 relative,
  the constraint Jacobian and the time's Hessian 6 of 50,700 entries by 2.6e-16: last-bit
  rounding. The tyre's stiffness and peak are now symbols (parameters) where they were constants,
  and CasADi folds constants (c1 c2 x as one product) where it cannot fold a symbol, so a few
  products associate differently;
- solved tightly (tol 1e-8, cold, 260 m windows from Haruna s 60 and Tsukuba s 100): the same
  optimum, plan times 11.472805855120672 against ...1201 s and 11.43913500355609 both, states
  within 2.6e-10;
- at the live tolerance (1e-3) the iterates' last bits steer IPOPT onto another path that
  stops at another point inside the tolerance: Haruna's first window 184 iterations against 167,
  11.473607 s against 11.473619; a 13-window warm chain wanders up to 0.08 % (that much is the
  receding chain carrying each plan into the next). The old code does the same to itself: the
  start speed moved by 1e-9 m/s gives Haruna's first window 11.457094 s (-0.14 %) and states
  0.36 off. So the difference is the solver's tolerance, not the model (L2's ticks and the
  plant downstream of those plans move with them).

## The low-friction arms on the new tyre

Grids (studies/drift_optimality.py, L1's window at study settings: ds 2 m, 360 m, tol 1e-6, cold,
free against the rear axle held to its tyre's peak + 1 deg, two starts each; drift-optimal =
the free plan's rear axle 3 deg or more past the peak and the held plan slower by more than the
noise): `m2-lowmu`, the 60 mu 0.45 and 0.3 cases of the study's `rear` grid on swb's tyre for a
friction with no class (asphalt under a friction scale: the rig's 8.55 deg peak, the stiffness
scaled with mu); `m2-classes`, eight of swb's classes as classes (their mu, sliding and peak),
R 9/15/25/40/60 at 90 and 180 deg (80 cases); `m2-classes-old`, the same 80 on the rig's tyre
(the study's tyre before M2). 4 jobs, 220 cases, every solve counted (0 failed).
Full tables: `m2_report.md`.

**Verdict: on the tyre swb gives Isaac, drift is optimal in far fewer low-friction corners and
is worth less; it stays optimal, by about 1.1-1.3 %, where the surface does not fall off past
the peak (mud, sand, loose snow, sliding 0.95-1.0) and in tight hairpins.** Drift-optimal cases:
abstract mu 43 of 60 -> 18; classes 53 of 80 -> 32.

| surface | drift-optimal (rig -> own) | largest gain, rig -> own | where it still pays |
|---|---|---|---|
| mu 0.45, sliding 1.0 | 5 -> 4 | 1.76 -> 1.31 % | 180 deg, R 9-40 |
| mu 0.45, sliding 0.8 / 0.6 | 9 -> 1 | 0.72 -> 0.38 % | the 9 m hairpin (0.8) |
| mu 0.3, sliding 1.0 | 10 -> 8 | 1.75 -> 1.20 % | 180 deg every R, 90 deg R 9-25 |
| mu 0.3, sliding 0.8 / 0.6 | 19 -> 5 | 0.95 -> 0.35 % | hairpins R 9-15 |
| snow_packed (0.20, 0.8, 6 deg) | 10 -> 3 | 1.95 -> 0.34 % | hairpins R 9-25 |
| snow_loose (0.25, 1.0, 12) | 10 -> 6 | 2.54 -> 1.11 % | 180 R 9-40, 90 R 9-15 |
| ice (0.265, 0.7, 3) | 7 -> 0 | 1.56 -> none | nowhere |
| mud_wet (0.29, 1.0, 12) | 9 -> 8 | 1.81 -> 1.21 % | 180 every R, 90 R 9-25 |
| sand (0.265, 0.95, 14) | 9 -> 8 | 1.47 -> 1.32 % | 180 every R, 90 R 9-25 |
| grass (0.55, 0.75, 9) | 3 -> 3 | 0.38 -> 0.30 % | hairpins R 9-25 |
| gravel (0.60, 0.9, 12) | 3 -> 2 | 0.34 -> 0.61 % | hairpins R 9-15 |
| dirt (0.68, 0.85, 10) | 2 -> 2 | 0.29 -> 0.40 % | hairpins R 9, 25 |

Why. The rig's tyre on low friction was stiff for the friction it had: its brush saturated at
2-3 deg (tan = 3 mu F_z / C) while swb's fall-off began only at the rig's 8.55 deg, so it gave
the full peak flat from 3 to 8.5 deg: in effect no fall-off over the first 5 deg of a slide.
Packed snow, Fy / mu Fz at 2/4/6/10/20 deg: 1.00/1.00/1.00/0.97/0.88 before, 0.53/0.83/0.96/
0.92/0.86 now; wet mud 0.93/1.00/1.00/1.00/1.00 before, 0.29/0.52/0.70/0.91/1.00 (its peak
16 deg at the loaded wheel). A free slide was cheap on that plateau; now the grip is reached
later (a gripping car already runs 6-16 deg) and lost past the class's peak, so holding the
rear at the peak costs little where the class falls off (packed snow, ice, the 0.8 and 0.6
arms), while on a surface that never falls off (mud, sand, loose snow) the slide still buys the
rotation and keeps half to nine tenths of its gain. The rear past its peak, median: abstract arms
+3..+21 deg before, +1.5..+15 now; classes +5.5..+20 before, +1.5..+13 now.

Every section is slower on the new tyre: the free plan's time +0.9..+4.0 % (median per
surface; ice up to +7.9 %), because the grip comes at larger slips (and the longitudinal one
too: packed snow's stiffness is 0.30 of the rig's, so a braking force needs three times the
slip). The noise (spread over starts) fell: median 0.002 s on the new tyre against 0.008-0.036
s on the old at low mu, so fewer cases are undecided; on grass, gravel and dirt it rose a
little (0.004-0.010 s).

What this changes in M1's picture: M1's drift-optimal set at mu 0.45 and 0.3 was largely the
old tyre's plateau. On swb's tyre a drift is optimal (1) in hairpins of R 9-25 on almost every
loose surface, (2) in corners of every radius on surfaces with no fall-off (mud, sand, loose
snow, abstract sliding 1.0), and nowhere on ice. The gains M4 and M6 should look for: about
1.2 % on mud, sand and loose snow; 0.3-0.6 % in loose-surface hairpins; none on ice. The
corner library (`corner_library.npz`, M1's Dc plans) and M1's coverage numbers were made on the
old tyre; its archetypes' slides are now 2-8 deg too deep for snow and ice (see Rulings).

## Rulings

- The planner takes swb's rule, not Isaac's sweeps: `SurfaceTable.tyre` computes the stiffness
  and peak shares from swb's class table as `WheelSurfaces.update` does, and test_surfaces.py
  checks them against swb's code imported from the superproject. Why: Isaac per class cannot run
  on this host beside another user's CARLA; swb's code is what Isaac runs. Cost if wrong: where
  PhysX's real tyre departs from its brush beyond the knee, per class; the asphalt fixture checks
  only the reference. Isaac's per-class sweeps are M6's (deferred).
- A friction with no class (the study's abstract mu, a Track built by hand) keeps the rig's tyre
  (all shares 1), today's behaviour; the study's `m2-lowmu` arm builds swb's tyre for it
  explicitly (asphalt under a friction scale). Why: no class says where its peak is; the default
  must leave asphalt and every existing Track bit for bit. Cost if wrong: a hand-built low-mu
  track plans on the stiff tyre and over-rates drifting (by the table above).
- The grip share (0.92 live), the friction estimate's `mu_scale` and the plant's patches lower
  the friction alone, the stiffness kept (as before M2), where swb's road condition would scale
  the stiffness and keep the class's peak. Why: the grip share is a margin, not a surface, and
  the live asphalt path must not change. Cost if wrong: under a patch the plan's peak comes
  early by the patch's scale (0.45 / 1.12: 3.5 deg instead of 8.6); M4 should model a known
  low-mu patch as a class instead.
- Stribeck (ice's sliding share falling with sliding speed) and the relaxation length stay out
  of the planner (a steady tyre). Why: as before. Cost if wrong: ice slides worse than planned,
  which only strengthens "no drift on ice".
- Asphalt's live windows are not bit-identical: L1's code changed in its last bits (symbols for
  constants), and IPOPT at tol 1e-3 is chaotic to that (a 1e-9 m/s change of the start does as
  much). Ruled equivalent: the tight-tolerance optimum is the same to 1e-10. Cost if wrong:
  none found; R6's 0.1 % is above the chain's 0.08 % wander.
- The corner library and M1/M3's seeds are left as built (old tyre). Why: M3's driver uses them
  as starts, not as plans; a start that is too deep is pulled back by IPOPT. Cost if wrong: Dc
  seeds on snow start 2-8 deg past the new peak and take more iterations; rebuild the library
  from a multistart grid on the classes before M4 if its iteration counts say so.
- Eight classes, not twelve: mud_soft and sand_loose are mud_wet's and sand's tyre with mu 0.03
  and 0.02 lower; concrete is asphalt's peak with mu 0.94 of it. Cost if wrong: none expected.
