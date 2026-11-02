# 015: the multi-start arms drift the chain's uncertain asphalt corner and leave the road

**Severity:** high (blocks roadmap 061's gate item "the true-map chain finishes": with issue 013
fixed, zone 0 stops 10 of the 20 multi-start runs, at s 92-101, five seconds in).
**Status:** open, not reproduced on afd0cc5 (below). Found in M4i (`docs/mode-choice-m4i.md`, its chain blocker 2) and issue 013's
final runs.

## What is wrong

The class-zone chain (`studies/mode_choice_m4.py run zones`, 8 m wide), zone 0: the first corner
(R 25 left, turn-in s 80, exit s 125), declared asphalt with confidence 0.85 and wet mud the
alternative (`Zone(..., "asphalt", 0.85, None, ("mud_wet",))`), true asphalt. The planner's track
there is asphalt's tyre at the mixture's lower friction (`SurfaceMap.friction`: the 0.15 wet mud
sets the floor). Repeatable runs (`RP_DET=1 RP_ITERCAP` 150-200, `--jit` draws), autoware-drift
82f0b69 (issue 013's fix), the car at sim_vehicle_ecu 760cd78:

| | single | sched | always |
|---|---|---|---|
| stopped in zone 0, of 10 draws | 0 | 5 | 5 |

The same draws stop there on main before issue 013's fix (the s to 0.1 m), and M4i found the
d185 and d200 draws stop there on every code since 95b902e.

## What the runs show

`zones__sched_r1_d185j6fx` (swb `play_log/race_planner/mode_choice/i013/fix/`, `.traj.json`):

- Grip plans first (D at t 0, G at t 3.0), the car on them (n within 0.1 m, beta within 1 deg).
- At t 4.07 (s 81, the turn-in) a drift plan from D is adopted (`mode_changes [[80, grip,
  drift]]`): beta -15.4 deg at s 92 against the grip plan's -8.4, on the planned (lowered)
  friction.
- The car is on true asphalt: its rear does not slide (rear slip angles 4-7 deg, beta to -5 deg).
  L2 steers for the plan's attitude, 12 deg at s 83.6, 27 at 85.9, 43.5 at 88.6, then the 57.9
  deg stop; the fronts at slip angles of 50-70 deg. The car yaws faster than the road
  (heading 0.13 rad inward at s 98) and leaves the road on the inside at s 98.8, t 5.08.

## Where to look

- Whether a drift is offered where the declared class is uncertain and the world may be grippier
  than planned: a grip plan on a lowered friction is safe on a grippier road, a slide is not
  (the rear does not go past a peak that is higher than planned).
- L2's response to a planned sideslip the car does not reach: steering to the stop.

## Reproduce

```
cd src/planning/race_planner
RP_DET=1 RP_ITERCAP=185 env -u PYTHONPATH .venv/bin/python studies/mode_choice_m4.py run zones \
    --arm sched --jit 6e-9 --tag d185j6 --traj          # off_road @98.8
```

## On afd0cc5 (every start at 0.85 s), 2026-11-02

The same 30 draws (10 an arm) on afd0cc5 (swb `play_log/race_planner/mode_choice/i015/base/`):
zone 0 stops **0** (single 0, sched 0, always 0). The budget change moved the iteration caps
(`RP_ITERCAP` scales with the budget), and with them which plan each re-plan adopts: in
`d185j6` the drift plans made in zone 0 slide only at the next corner (s 220-284), not over the
uncertain stretch (the map's grippier nodes, s 24-144). The mechanism above is not gone, only not
hit by these draws. (A first count, "drift adopted in zone 0, 18 of 20", counted plans made in
zone 0, not slides over it; wrong.)

## A gate tried and not kept

`SurfaceMap.grippier(s)` (a class at least `p_floor` probable gripping more than the planned
friction) and a `MultiStart` filter dropping any plan whose rear is past `modes.REAR_PAST` over
such a node while another feasible plan is not. Three spans tried for the slide (the corner the
re-plan is for; with no corner, the window past `EXCLUDE`; the whole window past `EXCLUDE`). On
82f0b69, the 10 multi-start draws that stop in zone 0 still stop there with each (s 92-101):
at the turn-in every feasible start slides over the stretch (H too, rear past 0.11 rad), so the
filter has nothing to keep and falls back. On afd0cc5 the chain, the corners and the Haruna /
Tsukuba digests are unchanged with it (it is never decisive). Not kept: it does not fix the
failure it was for. Results `i015/at82/`, `i015/fix/` (local commit af80ad2, not pushed).

## Left

- Whether L1 should offer no slide over a grippier-probable road at all (a slip bound there,
  as the held rows do for ice), rather than a choice among starts that all slide.
- L2 steering to the stop for a sideslip the car does not reach (above).
- Re-open on any draw that stops in zone 0 again.
