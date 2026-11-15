# Enabled-time audit: November 14, 2026

Objective: maximize enabled time while maintaining temperature and recovery. Earlier demand reduction remains a hypothesis. No firmware, device or ESPHome configuration changes.

The same cached history as the first workbench run was audited in full. Percentages below describe enabled time while heat mode was requested with valid adjacent samples. They are not compressor duty cycle or an estimate of avoidable downtime.

| Room | Enabled fraction | Hysteresis-consistent shutdowns |
|---|---:|---:|
| primary | 32.6% | 67 |
| hailee | 22.5% | 142 |
| main | 32.6% | 20 |

Transitions near hysteresis bounds are associations, not proven causes. Fireplace/fusion context, target changes, right-censored off intervals, missing/changed gains and partition exclusions are recorded in events.json and the CSV catalogs.

## Does the simulator reproduce enabled time?

| Room | Held-out windows | Thermal replay enabled-time MAE | Measured-temperature controller replay MAE |
|---|---:|---:|---:|
| primary | 1 | 1.00 min | 1.00 min |
| hailee | 4 | 71.75 min | 0.50 min |
| main | 4 | 77.50 min | 0.50 min |

Each response window is 150 minutes; windows can overlap, so these are not independent trials. Both replays use recorded stable running gains, not YAML starting gains. Thermal replay generates its own room temperature; controller-only diagnostic intentionally supplies measured temperature. External weather, fireplace, targets and other-head activity are recorded scenario conditions.

Hailee/Main Floor controller-only replay matches sampled shutdown timing within five minutes, but thermal replay does not. Hailee controller-only command agreement is approximately 69%, another limitation to investigate. Primary has only one eligible held-out replay (a target change), so its apparently small enabled-time error is insufficient evidence.

All rooms fail incumbent event validation. The previous Hailee temperature-prediction screen did not establish a valid enabled-time simulator. No new gain search was run and no gain change is recommended. Default gain search now requires a matching qualified event replay; exploratory bypass remains explicitly labelled.

Next technical task: investigate thermal response mismatch on training/validation shutdown/coasting events and Hailee command parity. Do not fit against these held-out results or relax gates to obtain a pass. Changed models must be checked on new independent data before claiming improvement.

See events.html for recorded-versus-simulated traces and events.json for gate settings and source/input provenance.
