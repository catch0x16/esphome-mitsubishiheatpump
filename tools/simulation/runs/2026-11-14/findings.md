# November 14 offline workbench findings

Read-only: no controller, firmware, YAML or Home Assistant changes.

The export requested 28 days. All three rooms have 21 complete usable local days from October 24 through November 13, with partial days on October 23 and November 14. Splits: 12 training, 4 validation, 5 final testing. Analysis uses the first 16 eligible windows per partition.

These errors are **conditional response replay** errors with recorded future commands, enablement, weather and fireplace state as inputs, not origin-only operational forecast accuracy.

| Room | 15-minute MAE | Initial model screen |
|---|---:|---|
| primary | 0.191°C | Fail: does_not_beat_stronger_baseline_by_10_percent, fewer_than_three_held_out_command_change_episodes |
| hailee | 0.128°C | Pass |
| main | 0.171°C | Fail: does_not_beat_stronger_baseline_by_10_percent, fewer_than_three_held_out_command_change_episodes |

Hailee has six eligible held-out command-response episodes; Primary has zero and Main Floor one. Hailee nominal 90% interval coverage at 15 minutes is 75% (16 observations): uncertainty is underestimated. Weather below about 4.4°C and cooling are unqualified.

## Batch gain search

64 candidates plus an incumbent comparison, six scenarios, five models, two workers: approximately 80 seconds. 26 distinct command trajectories. None of the ten retained best-scoring candidates passes every model guard. The best score reduces overshoot in four models but increases it in the fifth. No gains are approved.

The selected scenarios have zero modeled incumbent shutdowns and zero substantial recovery deficit. They do not establish performance during those conditions. Broader, representative event coverage is required before making gain recommendations; there is no evidence here to replace the successful deployed controller.

Inspect `analysis.json`, `hailee-search.json`, `report.html` and the executed `workbench.ipynb` for coefficients, model disagreement, errors and source/input hashes.
