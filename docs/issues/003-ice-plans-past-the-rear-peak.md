# 003: on ice the plans sit past the rear tyre's peak, and the fallback cannot start there

**Severity:** high (ice is lost in every arm: 9 of 22 started runs stop in M4, the same count in
every arm). **Status:** mitigated (roadmap 061 M4b); open for L1's speed on ice.

## What is wrong

Roadmap 061 M4 (`docs/mode-choice-m4.md`): on ice every arm stops in 9 of 22 started runs (off
road 4-6, spin 1-4, timeout 0-3), with the plans' sideslip small (3-9 deg) and the car's 40-80
deg. The scheduled arm's failed re-plans double (116 against 64). L1's solves are 3-5 times
slower there than on any other class (p50 0.50-0.77 s against 0.14-0.24, p95 at the 1 s budget;
M4's corners, load ~8).

## Root cause

Instrumented runs (the loop with every tick's plan and car logged: the rear axle's slip angle,
the wheels' slips, the request, each start's solve), idle host (load 1.5-2), sched arm:

1. **The plan is past the rear tyre's peak, on its falling branch.** Ice peaks at 2.95 deg of
   rear slip angle and falls to 0.7 of its friction past it (`sliding` 0.7; the planner's and the
   plant's tyre are the same function, issue 002). `ice:15:90:15`: every re-plan solved (11 of
   11), but the plans run the rear at 4.1-4.9 deg, 1.4-1.7 times its peak; at s 55 the car's
   rear is 1 deg past the plan's (5.1 against 4.1), and from there the slip runs away
   monotonically, 8 deg at s 59, 12 at 73, 20 at 88, 30 at 100, while the plan stays at 4-5 and
   L2 steers 0.12-0.2 rad against the plan's 0.05: past the peak more slip gives less force, so
   the slide is unstable and a lag of a degree is enough. Off road at s 116. On a class that
   keeps its friction past the peak (mud) the same lag is harmless; on ice it is a spin.
2. **The fallback cannot start where it is needed.** H holds the rear at the peak plus 1 deg
   (4 deg on ice) at every node, the first one too, which is the car's state: once the car's
   rear is past 4 deg every H window is infeasible. `ice:15:90:15`: H failed on every re-plan
   from s 56 on (8 of 8), exactly when the car left its plan; `ice:25:90:15`: H reported
   `Infeasible_Problem_Detected` on 5 consecutive re-plans from s 60.
3. **The solves are cut at the budget.** `ice:9:180:6`: G and H both stopped at the 1 s wall
   (G 170-210 iterations, H 160-190) on 8 of 10 re-plans; G's plans that did finish ran the rear
   to 8.8 deg, three times its peak. Offline (M4's oracle) the corner's cold solves take 520-850
   iterations, against 70-250 on mud. The falling branch is non-convex, and IPOPT wanders in it.

So ice is lost to the planner, not to the plant or the scheduler: it plans on the tyre's
unstable branch, which the car cannot be held on and IPOPT is slow in, and its fallback is
built so it fails once the car has slid past the peak.

## Fix

- The held rows (H) and the slip rows (issue 002) not on the window's first stages
  (`window.FREE_STAGES`, 3: the car's node and the next two, 12 m): a plan must come back within
  the bound, not start there.
- On a class that falls off deepest and is slippery (sliding share at most 0.75, planned
  friction at most 0.45 x grip: ice), the rear axle held at its peak in every start
  (`live.Driver(rear_slip_low=1)`, `window.slip_rows`): no drift pays on ice (the drift map) and
  its falling branch is a slide the car cannot be held on.

Measured (`docs/mode-choice-m4b.md` section 4; 12 ice corners, one repeat an arm, idle host):
runs finished, M4 6/7/7 (single/sched/always), the first fix alone 6/4/3, both **9/8/6**; time
+1.8 to +4.0 % on the corners finished both ways. Tried on packed snow too (planned friction
0.2, sliding 0.8) it cost 4 of 29 finished runs, so it is ice's only.

Left open: L1's speed on the falling branch (a third of ice's re-plans still fail at the 1 s
budget), an R3 solver matter.
