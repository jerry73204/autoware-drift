# 016: the class-zone chain stops in its loose-snow R 9 x 90 (zone 5), which finishes alone

**Severity:** high (blocks roadmap 061's gate item "the true-map chain finishes": with issue 013
fixed, zone 5 is the chain's first stopper, 11 of 30 runs). **Status:** open, not root caused.
Found in issue 013's verification (roadmap 061, Issue 013).

## What is wrong

The class-zone chain (`studies/mode_choice_m4.py run zones`, 8 m wide), zone 5: loose snow from
s 996 to 1096, a 90 deg right-hander R 9 from s 1056 to 1076, entered from 44 m of asphalt after
the ice zone (s 744-952). Repeatable runs (`RP_DET=1 RP_ITERCAP` 150-200, last-bit `--jit`
draws), autoware-drift 82f0b69 (issue 013's fix), the car at sim_vehicle_ecu 760cd78:

| | runs | stopped in zone 5 |
|---|---|---|
| issue 013's fix, single / sched / always, 10 draws each (swb `i013/fix/`) | 30 | **11** (single 7, sched 3, always 1; s 1069.7-1077.9) |
| main before 013 (`i013/base/`, `i012/q/`) | 30 (4 reach zone 5) | 3 |
| the corner alone at 8 m, `c:snow_loose:9:90:8` (M4i, issue 012) | every draw | 0 |

M4i: the plan's flick (its sideslip 15, 3, 8 deg) is not followed; the car arrives 4 deg against
15 and then slides 25 deg against 11. Since the corner alone finishes, something the chain brings
into the zone is suspect: the entry state, the ECU's grip found and held from the previous roads,
the plan carried across the road change, issue 013's recover rows, the flick seed.

The packed-snow hairpin (zone 6, 3 stops on 013's fix) is checked for the same cause.

## Reproduce

```
cd src/planning/race_planner
RP_DET=1 RP_ITERCAP=150 env -u PYTHONPATH .venv/bin/python studies/mode_choice_m4.py run zones \
    --arm single --jit 7e-9 --tag d150j7       # off_road @1071.5 on 82f0b69
```
