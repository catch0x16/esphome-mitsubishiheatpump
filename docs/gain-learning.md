# Experimental bidirectional gain learning

This implements the [learning requirements](tuner-learning-requirements.md) under
the [control vision](control-vision.md). It is an opt-in, on-device experimental
supervisor around the existing PID. It does not forecast room temperature, switch
power, change hysteresis, or replace the unit's onboard regulator. It is not a
certified optimizer or a guarantee of improved performance in this installation.

## Enable one room first

Start with Primary Bedroom only. Leave the other two rooms' deployed controllers
unchanged while observing their operation. The implementation does not coordinate
multiple learners over the network: do not enable simultaneous gain trials on
multiple heads of this shared outdoor system.

1. Load this repository's new component code. The existing `4f5db3d` pin does not
   contain this feature. For local validation, use an external component source
   with `type: local` and the absolute path to this checkout's `components` directory.
   For a deployed build, pin a reviewed commit containing these changes.
2. Copy the matching overlay from `examples/primary-bedroom-gain-learning.yaml`,
   `examples/hailees-bedroom-gain-learning.yaml`, or
   `examples/main-floor-gain-learning.yaml` into your ESPHome packages directory
   and include it from that room's YAML. It extends the existing `hvac` climate ID.
   The component already receives the room/fused temperature; it is not wired again. The example's Home Assistant entity names were checked against
   Prometheus, but confirm them again if entities have been renamed.
3. Record the deployment time and starting gains. Existing runtime gains may have
   grown away from their YAML starting values; consider that when selecting the
   first baseline. No existing learned gains are imported from the old heuristic.
4. Follow `PID Tuning State`, running-gain diagnostics and `PID Verified Revision`.
   Preserve the `PID_GAIN_*` log records alongside Prometheus history when auditing
   trials. The test YAMLs use fake inputs and are not deployment configurations.

Only a configured `gain_learning` block selects this supervisor and disables the
legacy instantaneous gradient surrogate. Without the block, the legacy updater
remains enabled for compatibility; its one-way proportional-gain growth is not
corrected in that path. Set `gain_learning.enabled: false` to freeze the new
learner at its restored accepted gains, rather than deleting the block.

## Inputs and automatic mode selection

The climate's current mode selects heating or cooling automatically. Each has its
own learner, accepted gains, evidence, confirmation budget and persistent record.
Switching modes pauses the old learner and cancels its incomplete pair; heating
and cooling observations are never mixed. Off, fan-only, dry and AUTO operation
pause gain trials. AUTO's heating/cooling action in this component is inferred from
temperature error, not reliable native direction feedback, so it is not used for
learning.

All references below are **local ESPHome sensor IDs**, not Home Assistant entity
IDs or climate-component references. A Home Assistant reading must first be
imported through a `platform: homeassistant` sensor. The example supplies those
adapters.

| Setting | Type and meaning |
|---|---|
| `outdoor_temperature_sensor` | Optional temperature-sensor ID in Celsius. Overrides the climate's own outdoor-temperature report; use this for the household aggregate. |
| `fireplace_active_sensor` | Optional binary-sensor ID for reported fireplace on/off state. |
| `fireplace_state_valid_sensor` | Optional binary-sensor ID asserting that the fireplace's reported state is known. Requires `fireplace_active_sensor`; useful for the RF receiver's State Valid signal. |
| `fusion_wall_weight_sensor` | Optional sensor ID reporting the wall sensor's contribution to the fused temperature, 0–100 percent. Analysis metadata; it does not configure or alter fusion. |
| `other_units_operating` | IDs of the two other heads' operating binary sensors in this installation. |

For example, the minimal shared-system configuration can use the unit's native
outdoor report:

```yaml
gain_learning:
  enabled: true
  other_units_operating: [other_head_a, other_head_b]
```

For the supplied aggregate and fireplace context:

```yaml
gain_learning:
  enabled: true
  outdoor_temperature_sensor: learning_outdoor_celsius
  fireplace_active_sensor: learning_fireplace
  other_units_operating: [other_head_a, other_head_b]
```

The fireplace-validity signal belongs specifically to the fireplace, not to the
climate as a whole. Omitted fireplace/fusion metadata is recorded as unavailable
and is not used for matching comparisons; it is not asserted to mean fireplace
off or a measured wall weight. Supplied but unavailable inputs pause learning,
with statuses identifying the missing input. The controller continues using its
accepted gains. Raw fused and bedroom sensor temperatures remain available for
interpreting measurement changes; the component cannot infer changing weights
from the fused temperature alone.

The Primary overlay imports the aggregate outdoor reading and its unit attribute,
converting Fahrenheit to Celsius explicitly. Unknown units produce NaN, not zero.
Its internal adapters create no additional public weather or fusion entities.
Fireplace signals describe remote settings, not independently measured combustion
or watts. The optional RF-validity flag and an active Home Assistant state
subscription provide connection/context checks, not proof of physical freshness.

The first experimental names `outdoor_temperature`, `external_heat`, `context_valid`
and `fusion_weight` are accepted with deprecation warnings and normalized to the
names above. An old `mode` setting is removed with a warning: it no longer restricts
learning to that mode. Updating the implementation changes record fingerprints;
old experimental records are not silently treated as verified under this revision.
Copy any previously verified values into the appropriate mode's YAML starting
gains before upgrading if they need to be preserved.

## Controller behavior

The new path retains the existing tanh output mapping, integral cap, per-room
setpoint limits and packet rounding. Hysteresis runs before the PID and remains
responsible for switching power. Trials never force a unit on or suppress a
required shutdown.

Two supporting corrections apply to the new path: off periods are reported to
the PID so an off/restart interval cannot become an integral update, and derivative
uses a 30-second first-order filtered measurement slope. Target changes reset PID
history, avoiding derivative kick. Gains are held fixed within an arm. Changing a
gain does not reset the integral or otherwise give a trial a different controller.
The legacy path keeps its unfiltered derivative. These corrections are not claims
that 30 seconds is the best filter for the real sensors.

## Candidate generation and command bounds

A deterministic schedule considers six coordinate directions, four wider
coordinated alternatives (including opposing P/I changes), six wider coordinate
alternatives, and rechecks of the configured and previous accepted triplets.
Randomness selects A/B order, not unbounded gain jumps.

Before starting discovery, the learner sizes an alternative using a read-only
snapshot of the actual PID error, integral accumulator and filtered derivative.
It uses exactly the controller's tanh mapping, output limits, raw adjustment
limits and 0.5°C command rounding. A bounded search finds the first different
rounded command along the selected gain direction. The initial command difference
must be exactly one native step (0.5°C), within `max_command_difference`. A small
percentage change is never timed as an experiment merely because its gain values
differ. A distinguishable command does not establish a distinguishable thermal
response or measured heating power.

`local_step` (default 0.10) seeds the numerical search bracket; it is no longer a
fixed percentage change applied to the equipment. `wider_step` (default 0.50)
also permits farther alternatives within the same one-step command difference.
Direction proportions use nonzero scales P=1, I=0.01, D=30 when gains are near zero.
Configured/previous triplet rechecks retain their exact values and are deferred
unless they currently produce a one-step command difference.

Each changed term must have useful excitation: absolute error at least 0.1°C for
P, absolute integral at least 0.1°C·s for I, and absolute filtered slope at least
0.1°C/min for D. These are engineering eligibility thresholds, not comfort limits
or identified process parameters. Flat temperature must not trigger a huge D
increase solely to manufacture command separation. Directions that cannot produce
one step within configured gain and command bounds are deferred; another direction
is considered without spending a trial period or confirmation budget. No feasible
alternative leaves the accepted gains active with `waiting_for_informative_command`.

Before applying any candidate, compare its rounded setpoint with the accepted
controller's rounded setpoint using the same current PID state and limits. A
candidate requiring more than `max_command_difference` (default 0.5°C) is abandoned
before that excessive command is sent. This check also runs during the trial.
It limits the incremental command effect relative to accepted gains; normal
scheduled target changes remain possible. Existing rounding can put packet
readback half a step beyond a raw offset limit, as it did before this feature.

Each arm starts only when the unit is enabled and the fixed candidate produces
command separation. Waiting states include `waiting_for_enabled_unit`,
`waiting_for_command_separation`, and `waiting_for_recovery`. Waiting is unscored
and accepted gains remain active. A frozen candidate is never resized between
arms or during confirmation. If an arm cannot start within one `period` duration,
the candidate becomes `inconclusive_start_timeout` and the search can move on.
Once an arm starts, power-off/coasting time and hysteresis shutdowns stay in its
normal outcome scoring; they do not pause the clock. A gain change whose commands
lose observable exposure is inconclusive, not proof that its direction is optimal. Confirmation requires actual
readback consistent with the selected arm and observable command differences
while the unit is enabled.

## Discovery, confirmation and acceptance

A pair consists of one accepted-gain period A and one candidate period B, with
randomized order. Each period has a settling interval followed by a scored
interval. Discovery and confirmation use separate pairs. Candidate gains remain
unchanged throughout both phases. Normal target changes remain in scoring.

Default engineering settings, chosen before field testing:

| Setting | Default |
|---|---:|
| Scored interval per arm (`period`) | 2 hours |
| Unscored settling per arm (`settling_time`) | 30 minutes |
| Observable command exposure per arm (`observable_time`) | 2 minutes |
| Comparable discovery pairs (`discovery_pairs`) | 4 |
| Comparable confirmation pairs (`confirmation_pairs`) | 24 |
| Attempt limit per phase (`max_pair_attempts`) | 48 pairs |
| Rest after promotion (`hold_time`) | 24 hours |
| Local search bracket / wider within-step alternative | 0.10 / 0.50 |
| Minimum meaningful relative improvement | 5% |
| Mean error/recovery guard | reference × 1.05 + 0.02°C |
| Peak overshoot guard | reference peak + 0.10°C |
| Allowed mean outdoor difference within a pair | 3°C |
| Allowed fireplace / each other-head duty difference | 0.25 |
| Allowed mean wall-weight difference | 5 percentage points |
| Allowed fusion-weight movement-rate difference | 0.01 percentage points/second |
| Gain bounds [P, I, D] | [0, 0, 0] … [250, 10, 7200] |

These are explicit experimental defaults, not new tolerances in the control
vision, measured response times, or values guaranteed by the research references.
`gain_learning` accepts the corresponding settings shown in the configuration
schema; the fusion matching thresholds and derivative filter are fixed in v1.
Minimum pair counts and hard gain bounds cannot be reduced past the schema limits.
No threshold is relaxed automatically to obtain a promotion.

For each scored period, record shutdown count, mean positive conditioning-side
overshoot, peak overshoot, mean absolute temperature error, and mean large unmet
demand `max(0, -directional_error - 0.5°C)`. The last metric measures degree-time
beyond a 0.5°C deficit, not elapsed time to recover and not an accepted comfort
band. It can reveal restricted recovery demand. Also retain operating time,
command exposure, outdoor mean, fireplace duty, both other-head duties, fusion
weight mean and movement, and starting error.

Classify a pair's direction lexicographically: shutdown rate first, then mean
overshoot, then large unmet demand, then absolute error. Differences must exceed
the 5% relative threshold; temperature-rate outcomes also have a 0.01°C meaningful
difference floor. Thresholds are symmetric between A and B. Discovery requires at
least three quarters of the pairs to favor B, aggregate improvement, and all guards.

For confirmation, make one decision at the fixed number of comparable pairs.
Use a one-sided exact sign probability over meaningful wins/losses; ties do not
count as wins. Before collecting confirmation data, reserve confirmation index k
in flash and allocate alpha `0.05 / (k * (k + 1))`. These allocations sum to at most
0.05 per device and conditioning-mode profile under the test assumptions. Repeated peeking cannot trigger promotion, and
reboots/manual gain reset do not replenish the persisted budget. A newly
incompatible configuration starts a new evidence family. There is no global
optimum certificate and no unqualified real-world 95% guarantee: these calculations
assume exchangeable pair directions under the null, adequate settling and useful
context control. Thermal dependence or unmeasured confounding can violate those
assumptions.

Promotion also requires aggregate improvement, no observed increase in shutdown
rate, and the error, unmet-demand and peak guards. No observed increase is not
proof of equal shutdown risk, particularly when events are rare. If both arms have
zero shutdowns, promotion must be supported by the other predeclared outcomes; it
must not be described as demonstrated reduction in shutdown risk. Full outcome
summaries remain available for checking which outcome actually improved.

Pairs with insufficient context overlap are recorded but do not count toward
confirmation. Invalid inputs, gaps exceeding 30 seconds, unsupported mode or
climate-off restore accepted gains and restart only the incomplete pair; complete
pairs remain. Exhausted evidence attempts yield an inconclusive result, not a
rejection or proof of optimality. After an unsuccessful/inconclusive candidate,
wait one settling interval before the next. A trial exceeding the configured
hysteresis shutdown threshold by another `peak_guard` triggers early rollback.
This does not override or move the actual hysteresis threshold.

## Saving and copying gains

Only accepted gains are persisted, never an active trial. Saves occur when reserving
a confirmation budget, promoting verified gains, or explicitly resetting/rolling
back. A save must succeed, flush to flash and read back successfully before the
learner reports a saved promotion. Failures keep the old accepted gains active
and retry no more frequently than once a minute.

The record contains configured, accepted and previous gains, revision, proposal
and confirmation counters, next search direction, PRNG state and the last accepted evidence summary.
Restore checks schema, finite/bounded gains and a fingerprint of the learner/PID
implementation, input identities, learning settings, mode and control limits.
Heating and cooling have separate records and mode-specific fingerprints; a record
from one mode cannot be restored into the other. An incompatible record falls back
to that mode's YAML starting gains. This implementation supports ESP32 flash storage.

The existing flat `control_parameters.kp`, `ki`, and `kd` remain shared starting
defaults. Optional `control_parameters.heat` and `control_parameters.cool` triplets
override them for each learning profile, so copying heating gains need not alter
the cooling baseline. These overrides require a `gain_learning` block (which may
be frozen with `enabled: false`). For example:

```yaml
control_parameters:
  kp: 1.25
  ki: 0.015
  kd: 0
  heat: {kp: 1.5, ki: 0.012, kd: 0}
  cool: {kp: 1.2, ki: 0.015, kd: 0}
  # Keep your existing hysteresis and adjustment limits here.
```

Changing a mode's YAML starting gains to a different triplet intentionally
establishes a new baseline for that mode. Copying the exact accepted triplet into YAML preserves compatible
saved history. Use the copyable nine-significant-digit values in `PID Tuning State`
for float round-trip precision. Disabling learning preserves compatible storage.

- `PID Kp`, `PID Ki`, `PID Kd`: **currently applied gains**, including temporary
  trials, for Prometheus history. Do not copy an arbitrary trial into YAML.
- `PID Verified Revision`: persisted revision of the current heating/cooling
  profile (unavailable outside those modes); promotions, reset and rollback advance it. A revision alone is not proof of improvement; inspect the status.
- `PID Tuning State`: current mode, progress/reason, both modes' accepted YAML
  values and separate revisions. `verified_saved_copy_to_yaml` identifies a promotion.
- `Reset Learned PID Gains`: reset both profiles to their configured starting
  gains; preserve each spent confirmation budget.
- `Rollback PID Gains`: restore the current heating/cooling profile's previous
  accepted triplet. Outside those modes, no profile is selected for rollback.
  This is an explicit user action, not a new successful trial.

The saved evidence retains sign probability and A/B shutdown counts, mean
overshoot, mean absolute error and large unmet demand. Detailed `PID_GAIN_PERIOD`,
`PID_GAIN_COMPARISON`, `PID_GAIN_SAVE` and `PID_GAIN_CONFIG` log lines retain trial
parameters, context, provenance and outcomes. Log retention must be configured by
the collector; Prometheus does not retain these text log lines. Intermediate
trial history is not all written to flash. This is a bounded on-device audit
summary plus telemetry, not an unlimited historical database.

## Validation and remaining field work

```sh
c++ -std=c++11 -O2 -Wall -Wextra -Werror -pedantic \
  -I components/mitsubishi_heatpump tests/test_pid_gain_learning.cpp \
  components/mitsubishi_heatpump/pid_gain_learner.cpp \
  components/mitsubishi_heatpump/adaptive_pid.cpp -o /tmp/test_pid_gain_learning
/tmp/test_pid_gain_learning
python -m unittest discover -s tests -p 'test_gain_learning_config.py' -v
esphome compile tests/gain-learning.yaml
esphome compile tests/gain-learning-primary-example.yaml
```

Tests exercise fresh confirmation, failed confirmation, no-response trials, context
mismatch, bounds, missing data, rollover, save failures, restore/copy/reset/rollback,
PID off/restart behavior, derivative steps, command-sized selection in both modes,
flat-signal/disabled/clipped deferral, unchanged output mapping, bounded inter-arm
waiting, fixed candidate gains, continued shutdown/coasting scoring, and synthetic heating/cooling with
thermal lag, packet quantization, fusion changes and fireplace disturbance. The
synthetic thermal runs deliberately do not assert that the tuner finds better
gains in every plant. They check bounded commands and behavior, not real-world
improvement. Static favorable-response fixtures verify promotion mechanics.
Older test/tuner files from the reverted implementation are not modified
or used by this test command or the new supervisor.

No field improvement is claimed yet. Before enabling more rooms, use the first
room's forward history to check settling adequacy, pair comparability, actual
command influence, evidence availability, shutdowns and recovery. Do not interpret
inconclusive or unobservable trials as evidence that the existing controller is bad.

## Research references

[NIST randomized block designs](https://www.itl.nist.gov/div898/handbook/pri/section3/pri332.htm)
motivates comparable blocks and randomized order. The sign test and alpha spending
are explicitly specified implementation choices with the assumptions above.
[Lequin et al.](https://perso.uclouvain.be/michel.gevers/PublisMig/P70.pdf)
discusses iterative PID tuning and local minima. This supervisor is a bounded
experimental search, not an implementation of that paper's IFT algorithm or a
model-based SIMC tuner.
