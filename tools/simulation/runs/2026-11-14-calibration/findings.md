# Bounded correction and next action

One training-only correction of four existing parameters; eight training and four development-validation contexts per room. Final-test days were excluded. These validation days were previously inspected; this is not new independent confirmation.

| Room | Temperature MAE before → after | Enabled-time MAE before → after | Shutdowns matched after |
|---|---:|---:|---:|
| primary | 0.308 → 0.256°C | 127.0 → 60.2 min | 0/5 |
| hailee | 0.261 → 0.236°C | 112.2 → 81.0 min | 0/6 |
| main | 0.289 → 0.207°C | 135.0 → 23.0 min | 1/4 |

Temperature errors use recorded commands/enablement and simulated remote-temperature/fusion, while enabled-time errors use fully closed-loop replay at stable recorded gains. Each response window lasts 150 minutes. All results are conditional on recorded external scenario inputs.

The correction improves errors but fails to reproduce shutdown timing. Stop model expansion and do not search gains from this model. No model or gains were deployed.

## Concrete next firmware change

The current live learner does not accumulate enabled time. Its win function ranks shutdown rate and temperature errors. Add enabled_seconds from Sample.power and compare normalized off duration as the primary outcome, with the existing temperature/recovery/shutdown guards and independent confirmation. This directly aligns the tuner with the stated goal and can be tested without a thermal simulator.

See docs/enabled-time-tuner-next-step.md in the repository for exact scope, persistence migration, tests and first-device deployment. Preserve accepted gains; invalidate evidence scored under the old objective. This pass did not edit firmware or device YAML.
