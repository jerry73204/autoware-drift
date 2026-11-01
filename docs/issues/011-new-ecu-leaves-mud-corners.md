# 011: with regen slip control, the car leaves wet-mud corners S1 used to finish

**Severity:** high (a car change makes the fastest-lap style leave the road on two wet-mud test
corners it finished before; the planner is not involved). **Status:** open.

## What is wrong

sim_vehicle_ecu `f3f5dcf` (regen slip control, issue 009) changes the car on wet mud, not only
on ice. With it, S1 leaves the road on two wet-mud corners that the previous car (`d68637a`)
finishes. 009's check of the other surfaces (wet mud 33/36, same as the old car) did not
include these corners.

## What was measured

`RP_DET=1`, `--arm sched`, no style record, one run each; the planner is the same in each pair,
only the car's ECU differs (`SWB` pointing at a tree whose `src/sim_vehicle_ecu` is the named
commit):

| course | planner | ECU d68637a | ECU f3f5dcf |
|---|---|---|---|
| `c:mud_wet:25:90:8` | 0a624f2 | done 12.96 s | done 12.96 s |
| `c:mud_wet:25:90:8` | 474a9bf | done 12.96 s | done 12.96 s |
| `c:mud_wet:25:90:8:100` | da6e230 | done 15.63 s | **off road** |
| `c:mud_wet:15:90:8:160` | da6e230 | done 20.03 s | **off road** |

So the planner change of 009 (`0c61d28`) is not the cause: the same planner finishes with the
old car and leaves the road with the new one. Corners with an entry straight (100 m, 160 m) are
hit; the 5-field corner (its default entry) is not.

Roadmap 065 M3 first saw this as an S1 regression "since the rebase onto 474a9bf": the
superproject's `src/sim_vehicle_ecu` checkout moved to `f3f5dcf` during its runs, so its runs
after the rebase drove the new car.

## Where to look

- Which part of the regen slip control fires on mud: the cut on slip, the 0.25 release, the
  runaway-wheel test, or the hold (0.7 x found grip). A longer entry means more speed and a
  harder first brake.
- Whether the planner should model the hold on wet mud after all, or the ECU should not act
  there (009 found the planner's hold made snow and mud plans leave the 6 m roads).

Outputs: swb scratch runs reproduced with the table's commands; see roadmap 061 for the entry.
