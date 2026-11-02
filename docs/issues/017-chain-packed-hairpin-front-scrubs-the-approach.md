# 017: the class-zone chain's packed-snow hairpin is approached braking with the front tyres

**Severity:** high (blocks roadmap 061's gate item "the true-map chain finishes": with issue 016
fixed, the packed-snow hairpin stops 11 of the 14 runs reaching it). **Status:** open. Found in
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

## Where to look

- The front's slip angle on a surface that falls off but not as deep as ice (packed snow): a
  plan the car cannot be held to, which the slip rows keep out only on flat surfaces.
- Why the asphalt's 40 m are not the brake: the plan reaches the snow at 15.5 m/s where the
  rear alone (regen, issue 009) brakes it at under 1 m/s^2.
