# 001: no first plan in two tight low-friction hairpins

**Severity:** medium (the car never starts; no wrong plan is driven). **Status:** open.

## What is wrong

`live.Driver.start` solves the first plan once, from IPOPT's own start (`window.initial`), in
the cold window (`max_iter` 400, the window's default), with G's formulation only: no H, no D,
even when the driver has `multistart`. Where that one solve does not converge it raises
`RuntimeError("no first plan: ...")` and the run ends before the car moves.

Roadmap 061 M4 (`docs/mode-choice-m4.md`) hit it on two of its 72 corners, in every arm and
repeat (12 runs): `c:mud_wet:9:180:15` and `c:ice:9:180:6`, both
`Maximum_Iterations_Exceeded`, also from 10, 19 and 27 % below the corner's entry speed
(the harness's retries; its `v_start_used` field records the next, untried, step).

## How it was confirmed

The M4 oracle (`studies/mode_choice_m4.py oracle`, ds 4, tol 1e-6, 3000
iterations, the same start state) on the same corners (a 300 m window where the loop has ~200 m):

| corner | G from cold | H (held at the rear peak) | D (library / synthetic) |
|---|---|---|---|
| mud_wet R 9 x 180, 15 m | Restoration_Failed after 1273 iterations | solved, 181 it. | Dc solved, 94 it.; Ds 253 it. |
| ice R 9 x 180, 6 m | solved in 818 it. | solved in 780 it. (G from H's plan: 265) | Dc 579 it., Ds 529 it. |

So both corners have plans. On mud G from IPOPT's start never gets there while H and D do in
under 400 iterations; on ice every start does, but past 400 iterations (G warm from H's plan in
265).

## Why it matters

A car that cannot make its first plan at a tight slippery hairpin cannot start there (a grid
slot, a restart after a stop). The receding loop's fallback (the last plan, then any seed's)
only exists after a first plan.

## Fix (proposed, not made)

In `start`, when G fails: more iterations (the first solve has no budget: the car is not
moving), then H (held at the rear tyre's peak, from cold) and G from H's plan, and with a
schedule D from the library; adopt the first feasible, as a re-plan does when G fails. The
extra solves cost only start-up time. A test: both corners above start.
