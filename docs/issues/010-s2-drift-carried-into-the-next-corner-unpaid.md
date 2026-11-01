# 010: S2 carries a drift into the next corner without its entry check or its debit

**Severity:** medium (S2's account and allowance are skipped for every bought corner the car
reaches already in the drift mode: on Haruna at 0.5 s, 3 of the 5 bought corners it drove
in a drift; the lap's spend is under-reported, and a forced drift the gate would refuse is
driven). **Status:** fixed (roadmap 065 M3; see the fix).

## What is wrong

`MultiStart._choose_s2` (roadmap 065 M2) enters a bought corner's drift only from the grip mode:
the drift plan within its price plus 0.3 % on two re-plans, then the account debited once. In
the drift mode it only checks the wider exit band (price + 0.3 % + `exit_hyst`). The mode is the
gate's, not the corner's: S1's rule keeps the current mode while no corner is ahead
(`corner_ahead` is None, every plan takes the current mode), so a drift entered in one corner
is still the mode when the next corner comes into the window. That corner is then never
"entered": no allowance check, no debit, and the exit band keeps a forced drift that costs more
than the corner's allowance.

## What M3 measured

Haruna, S2 at `eps_lap_s` 0.5 s, `RP_DET=1`, every re-plan's record (swb
`play_log/race_planner/style/m3/i010/`):

- s 155.9: no grip plan in corner 275.8 (G itself slides): mode drift, debited 0.029 s (right).
- s 275.8 to 691.5: corners 375.8, 503.7, 599.6 all met in the drift mode (G slides there too);
  503.7 is a priced corner (0.029 s), never debited.
- s 719.5: corner 867.4, G is a grip plan again, the mode still drift: B (the forced drift)
  adopted at a live loss of 0.066 s against an allowance of 0.055 s, kept by the exit band
  (0.081 s); never debited.
- s 1087.3: corner 1195.2 entered properly from grip (debited 0.029 s).
- s 1271.2: corner 1411.1 met in the drift mode carried from 1195.2 (no corner between), B kept
  at 0.061-0.068 s against 0.056-0.058 s; never debited.

The account reported 0.058 s spent; the corners driven in a drift were priced at 0.146 s.

## Root cause

The gate's hysteresis state (mode, count) spans corners, but S2's entry rule and debit are per
corner. Nothing records which corner the current drift was entered (and paid) for.

## Fix

`_choose_s2` keeps the corner its drift was entered for (`_s2_drift`). In the drift mode, a
bought or lex corner other than that one is an entry: kept in the drift if it meets the entry
rule (the drift plan within the allowance, or no grip plan), then debited and recorded;
otherwise counted towards leaving, left on `confirm` re-plans (as the exit rule, so a slide
carried in is not cut in one re-plan). S1's gate is untouched (S1 has no account). Tests:
`test_s2_gate.py`, a drift carried into a second bought corner is debited when it fits and left
when it does not.
