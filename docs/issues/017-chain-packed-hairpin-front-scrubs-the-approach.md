# 017: the class-zone chain's packed-snow hairpin is approached braking with the front tyres

**Severity:** high (blocks roadmap 061's gate item "the true-map chain finishes": with issue 016
fixed, the packed-snow hairpin stops 11 of the 14 runs reaching it). **Status:** fixed in the
planner (the front's slip row binds on packed snow, `window.slip_rows`; see "Root cause" and
"The fix"). Found in
issue 016's verification (roadmap 061, "Chain blockers after issue 013").

## What is wrong

The class-zone chain (`studies/mode_choice_m4.py run zones`, 8 m wide), zone 6: packed snow
from s 1137 (60 m before the turn-in) to 1303, a 180 deg left-hander R 25 from s 1197, entered
from 40 m of asphalt after the loose-snow zone (ends s 1097). Repeatable runs (`RP_DET=1
RP_ITERCAP` 150-200, last-bit `--jit` draws), autoware-drift 145749d (issue 016's fix) and
b4854da (main, the same runs to the bit), the car at sim_vehicle_ecu 760cd78:

| | runs | reaching zone 6 | stopped there |
|---|---|---|---|
| single / sched / always, 10 draws each (swb `i016/fb/`, `budget_r6/A/`) | 30 | 14 | **11** (s 1206.4-1209.9) |
| the corner alone, `c:snow_packed:25:180:8` (issue 012) | every draw | | 0 |

## What the runs show (the 14 runs' `.traj.json`, `i016/fbtr/` traced)

Every one of the 14, the 3 that finish too, drives the same thing to the decimetre:

- s 1097: the loose-snow corner left at 6.4-8.0 m/s. On the asphalt the plan accelerates at
  5.4 m/s^2 to 16.3-17.1 m/s at s 1130 and brakes only from there (-3.2 m/s^2): 15.5 m/s at the
  snow's edge (s 1137).
- s 1140-1197, the straight on packed snow: the plan steers 26 deg at s 1150 and 40-43 deg from
  s 1160 on, the front tyres at slip angles of 47-50 deg (packed snow's peak: 6), the car
  sliding (vy -1.4 m/s) and heading 10-17 deg off the road: the front used as a brake (the
  rear brakes at -0.8 m/s^2, its grip; together 1.0-1.2 m/s^2 from 15.5 to 9.5 m/s).
- The car follows that steering within 1 deg and the speed within 0.2 m/s, but drifts 1-2 m
  off the plan's line (n); L2 reaches the lock (57.9 deg) at s 1195-1197 with no lateral
  authority left (M4f's mechanism: a front far past its peak barely changes its force with
  the steering), and the car leaves the road on the inside at s 1206-1210.

Packed snow (sliding share 0.8) falls off past its peak, so the windows' front slip rows
(issue 002, `window.slip_rows`) do not bound it there: "the model's own fall-off does". At 0.8
of its peak force the front still drags well enough to be the plan's cheapest brake.

## Reproduce

```
cd src/planning/race_planner
RP_DET=1 RP_ITERCAP=150 env -u PYTHONPATH .venv/bin/python studies/mode_choice_m4.py run zones \
    --arm single --jit 7e-9 --tag d150j7       # off_road @1207.9 on 145749d
```

## Root cause (2026-11-02, autoware-drift b4854da, the car at sim_vehicle_ecu 760cd78)

The front's slip row (issue 002, `window.slip_rows`) binds only on a flat road (sliding share
0.95 or more); elsewhere "the model's own fall-off" was meant to keep the plan out of a deep
slide. Packed snow falls off to 0.8 of its peak: a front at eight peaks still makes 0.8 of its
force, and at 40 deg of steering most of it is drag. So on the hairpin's approach the plan's
cheapest brake is the front scrubbing on the snow, not the 40 m of asphalt before it (where
it accelerates instead, to 16.8 m/s). The plan is consistent in the model and the car follows
its speed, but a front far past its peak no longer turns the car: L2 cannot correct the 1-2 m
of line the car loses and reaches the lock (M4f's mechanism, as on ice's 6 m roads). The corner
alone starts at its own speed for packed snow, with nothing to scrub off, and finishes.

Traced (`i017/a1tr/`, `zones single d150j7` with the fix): the plan brakes on the asphalt
(15.4 m/s at s 1130 against 16.7), reaches the snow at 13.5 m/s against 15.5, the front within
1-2 of its peaks on the approach (steering 1-7 deg against 40-43) and through the hairpin.

## The fix (the planner)

`window.slip_rows`: the front's row binds (`front_slip_max`, 1 peak) also on a road that falls
off but not deepest (sliding share above `SLIDING_LOW`) whose planned friction is `MU_LOW` or
less: of swb's classes, packed snow only. Not on ice (its front is the plan's only brake besides
the rear's regen, M4f, issue 009: bounding it there failed every re-plan), not on asphalt,
gravel or dirt (grippy: nothing asked it). As before the rows exist only on a course with a
flat road (`live.Driver`), so courses without one plan as before. Test
`test_window.py::test_the_front_slip_bound_holds_on_packed_snow_but_not_on_ice_or_asphalt`
(fails before the fix).

## Result (repeatable; the true-map chain, issue 016's 30 draws, 10 an arm)

| stops by zone (30 runs) | b4854da (`i016/fb/`) | the fix (`i017/fix/`) |
|---|---|---|
| zone 0 (asphalt declared uncertain) | 2 | 2 (the same draws) |
| ice (zone 4) | 10 | 10 (the same draws, the same s) |
| loose snow R 9 x 90 (zone 5), of the runs reaching it | 4 of 18 | 4 of 18 (other draws, below) |
| **packed-snow hairpin (zone 6), of the runs reaching it** | **11 of 14** | **1 of 14** |
| later (mud R 15 x 90, zone 8, s 1721-1745) | 0 | 3 |
| **finished** | **3** (152.5-155.9 s) | **10** (153.0-155.4 s) |

By arm, finished: single 1 -> 2, sched 1 -> 5, always 1 -> 3. In all 14 runs reaching the
hairpin the car now meets the snow at 13.4-13.7 m/s against 15.4-15.6, the front's slip angle
on the approach at most 21 deg (car, any tick) against 47-50 in every base run. The one packed
stop (single d200j8, s 1282.9) is another mechanism: the car at the inside edge through the
hairpin (n 2.9), its rear sliding from 7 to 24 deg at the exit, L2 at the lock from s 1273.

The windows from s ~935 see the packed snow (200 m), so every run from there is another run
(diverging at s 935-974): the loose-snow stops moved between draws (single d150j3 passes, d160j9
and d185j6 stop; sched 3 -> 1; always 0 -> 1), 4 of 18 both times, and the mud zone 8 is reached
by 13 runs against 3 (its 3 stops: not looked at; issue 012's and budget_r6 C's runs stopped
there too).

Unchanged, bit for bit (every field of the result but wall time): one corner each of ice
(`c:ice:9:90:6`), wet mud (`15:180:6`), loose snow (`9:90:6`), gravel (`15:180:15`), packed
snow (`25:180:15`), single and sched, d170j5; Haruna / Tsukuba `RP_DET` single and sched
76.28 / 74.33 s, digests 9524862c / 9160b94d, d7ca9f95 / 82bd3c28 (`i017/corners/`,
`i017/digest/`). Planner suite 211 passed.

## Left

- The chain's next stoppers: ice (zone 4) 10 of 30, loose snow R 9 x 90 4 of 18, the mud R 15 x
  90 (zone 8) 3 of 13, zone 0 2 (issue 015).
- The front is still free on packed snow on a course with no flat road (the packed corners
  alone, Haruna@snow_packed): the rows' gating is issue 002's; those plans did not need it.
- Rulings: the bound at 1 peak, the flat roads' (not swept); on low-friction roads only, ice
  excepted (M4f: its front is its brake). Cost if wrong: a packed-snow corner entered too fast
  to stop on the rear alone now has no planned brake but the rear (none seen on the chain).
