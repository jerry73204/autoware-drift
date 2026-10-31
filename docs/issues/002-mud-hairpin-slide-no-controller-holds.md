# 002: the mud hairpin's planned slide is one no controller holds

**Severity:** high (the car spins and stops in every arm at the zone chain's mud R 9 hairpin, so
nothing past it is tested). **Status:** open.

## What is wrong

Roadmap 061 M4 (`docs/mode-choice-m4.md`, "The deep slides: drivable?"): on the class-zone
chain every arm on every map stops at the second corner, the mud_wet R 9 hairpin (s 248-284),
the car spinning at 78-80 deg against plans of 52-65 deg. A 45 deg bound on the body's sideslip
in L1 (`Driver(beta_max=)`) did not help.

## Root cause

Measured offline (scripts in the M4b session; numbers below), three candidates in turn:

1. **The planner's tyre or car against the plant's: not it.** L1's car (axle.py with the
   wheels' speeds, as the window solves it) and the plant's (model.py's two-track, the same
   `model.tyre`) were evaluated on the free chain plan's own states and inputs at every node of
   the slide (s 192-280): dvx, dvy and dr agree to the printed 0.01, the rear tyre's Fx to the
   newton, the rear balance is 0. The mud tyre (sliding 1.0, peak 12 deg) is the same function
   in both.
2. **The plan.** The free plan from s 150 at 20 m/s (what the loop sees entering the mud zone
   from the asphalt straight) brakes into the hairpin by plowing: the front wheels steered
   50 deg against the slide, the front tyres at **55-66 deg of slip angle, 9-21 times their
   peak** (rho, the combined slip over the peak), the rear at 45-48 deg, **7-8 times** its peak,
   the body at 52 deg, 3-5 m/s. On a class whose friction does not fall past the peak
   (`sliding` 1.0: mud, sand, loose snow) a saturated tyre gives its whole mu Fz along its slip,
   however deep, so to the optimiser a plowing front axle is a free brake: the plan is 0.8 s
   faster than one that keeps the front within twice its peak (15.00 against 15.82 s, s 150-300).
3. **Following it.** The same plan, fixed (no re-planning), followed by L2 on the plant from the
   same state: the car follows it to s 232 (37.7 deg against 33.8), then over-rotates with every
   tyre saturated (front rho 12-22, rear 5-19) while L2 unwinds the steering (-20 deg against the
   plan's -27) to no effect: 57 deg at s 243, 77 at s 254 against the plan's 53, and the car
   stops sideways at s 265. With all four tyres past saturation the forces no longer depend on
   how far past: the steering and the request lose their authority over the yaw, so the slide is
   an equilibrium the car cannot be held on, whatever the controller.

So the cause is the plan, not the model: L1 plans slides that saturate the front axle, where
the car has no control left. The 45 deg body bound does not touch it (a 45 deg body slide can
still plow the front at 60 deg).

## Fix

Keep the axles within a multiple of their peak in L1, on a surface without fall-off only
(`window.slip_rows`; `Window(front_slip_max=kf, rear_slip_max=kr)`: each axle's slip angle, as
`model.tyre` takes it, at most k times the surface's lateral peak where the sliding share is
0.95 or more; the seeds' windows too; `live.Driver` defaults kf 1, kr 3, and adds no rows on a
track without such a surface, so asphalt's windows are 060's exactly). The front at its peak is
what drift control keeps (the front axle unsaturated: the steering is the yaw's control while
the rear slides); the rear may slide to three times its peak. Where the class falls off (ice,
packed snow, asphalt) the model's own fall-off already keeps the plan out of the slide (a front
bound there made every ice re-plan fail at the budget: 5 of 5, `ice:9:180:6`).

Fixed-plan following on the chain from s 150 at 20 m/s (front bound only, rear free):

| front bound | plan sideslip / front rho / rear rho | car sideslip, max error | result |
|---|---|---|---|
| free | 54 / 21 / 8.0 deg | 77, 31 deg | stopped sideways at s 265 |
| 2.5 x peak | 68 / 4.4 / 13.6 | 81, 14 | spun at s 251 |
| 2.0 x peak | 46 / 3.0 / 6.7 | 39, 21 | through, s 300 |
| 1.5 x peak | 40 / 2.4 / 5.3 | 25, 24 | through, s 300 |

The receding loop's result, and the bound's cost elsewhere, in `docs/mode-choice-m4b.md`.

In the receding loop, the zone chain on the true map (swb `play_log/race_planner/mode_choice/
m4b/zones__*`, two repeats an arm, load 1.1-2.2), stops at the mud hairpin (s 240-300) of six
runs: free (M4) 6; front 2 x 2 (one stall); front 2, rear 3: 2 (single, off road at s 265, the
car 52 deg against 35); **front 1, rear 3: 0** (the car 23-32 deg against plans of 27-36). With
it the chain's next stops are the ice zone (s 832) and the packed-snow hairpin (s 1228-1254),
issue 003 and its kin; two of six runs finish the chain (2082 m, 153.5 and 156.5 s).
