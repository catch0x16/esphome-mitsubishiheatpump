# References
https://github.com/echavet/MitsubishiCN105ESPHome

# Automatic PID tuning

Automatic tuning is enabled by default. The component runs bounded trials during
normal heating/cooling, evaluates them, restores the accepted gains after a bad
trial, and repeats promising trials before accepting new gains. It does not
force relay oscillations, change the requested temperature, or bypass the
existing correction limits or hysteresis.

Each scored window lasts one hour after ten minutes of settling. At least fifteen
minutes must be internally powered on during scoring, or the scored window must
contain at least two complete on/off cycles with five minutes of total enabled
time. Cycles beginning during settling do not count toward that alternative.
This allows low-duty-cycle operation to qualify without treating passive drift
or brief enable pulses as useful evidence. Hysteresis-off/coasting periods are
still included in the temperature score. A full successful comparison
is baseline → candidate → baseline recheck → candidate confirmation, normally
at least four hours forty minutes of sufficiently comparable operation.
Inconclusive comparisons are discarded automatically, so real learning can take
several nights. A window cannot be completed by replaying old history after a gap.
An inconclusive candidate is skipped until a later search; it does not repeatedly
send the search back to Ki-down. A fresh qualifying baseline is still required
before the next candidate. Target/direction changes start a new search.

The search tests Ki down/up, Kp down/up, then Kd down/up, changing one gain at a
time. Steps are 10% with output-aware absolute floors so probes do not disappear in
the unit’s half-degree rounding. Initially a probe represents roughly a 0.25°C
correction near the center of the existing output mapping, at a representative
0.5°C error (P), a saturated accumulator (I), or a 0.1°C/min filtered trend (D).
For an 18–25.5°C output range, those floors are approximately 13.35, 6.68, and
4006 raw gain units respectively, clipped to the gain bounds. After an accepted
trial, that coordinate’s floor halves, down to one eighth of its initial value.
These steps can be large percentages of tiny YAML starting gains; the applied
setpoint remains bounded by the existing correction limits. Default bounds are nonnegative gains, Kp up to
max(100, initial Kp × 100 + 1), Ki up to 10, and Kd up to 7200; configured initial
gains above these defaults remain representable. Derivative uses temperature
measurement (not target changes), with a thirty-second low-pass filter. The
larger D bound allows meaningful trials with that filtered derivative in °C/s.
The existing integral accumulator limit of ±1 is retained.

The score combines mean unmet demand + 3 × mean overshoot + 2 × peak overshoot,
plus 0.1 per shutdown/hour and 0.01 per degree of commanded setpoint movement/hour.
Lower is better. Average errors exclude settling, but peak overshoot is monitored
during settling too. Both candidate runs must improve by at least 5%, without
increasing peak overshoot by more than 0.1°C. Their starting directional errors
must be within 0.5°C of the reference windows. Repeated baseline scores and repeated
candidate scores must agree within 20% (or 0.05 score units, whichever is larger).
A complete unsuccessful search holds the accepted gains for six hours, then
starts a new search. Successful trials restart the search around the new gains.

Target/direction changes, climate-off periods, invalid temperatures, gaps over
thirty seconds, or abrupt measurement jumps over 1°C abandon a comparison and
restore the accepted gains. Excessive trial overshoot (> max(1.25°C, baseline peak
+ 0.25°C)) or unmet demand over 3°C rejects the trial immediately. The tuner does
not control doors or observe weather/occupancy directly; repeated reference
windows reduce these confounders but cannot eliminate them. This is an empirical
optimizer, not a guarantee of optimal gains or a safety controller.

No manual trial selection or acceptance is needed. To use fixed YAML gains:

```yaml
control_parameters:
  kp: 1.25
  ki: 0.015
  kd: 0.0
  autotune: false
```

Accepted gains and tuning progress are kept in RAM. Rebooting starts a fresh search
from the YAML gains; no flash writes or persistence are introduced. Existing
installed devices need updated firmware before this tuner replaces the previous
15-second error-driven adaptation. No Home Assistant automation is required.

# PID diagnostics

The component automatically creates these enabled diagnostic entities:

| Entity | Meaning |
| --- | --- |
| PID Kp / PID Ki / PID Kd | Gains used for the latest PID calculation |
| PID P Term | Kp × temperature error |
| PID I Term | Ki × accumulated error |
| PID D Term | Kd × filtered negative temperature rate |
| PID Tuning State | baseline, testing_ki/kp/kd_down/up, rechecking_baseline, confirming_trial, holding_best, or disabled |
| PID Baseline Score | Last completed baseline score; unavailable until measured |
| PID Trial Score | Last completed candidate score; unavailable until measured |
| PID Accepted Trials / PID Rejected Trials | Successful trials and rejected trials since boot |
| PID Discarded Windows | Windows discarded for insufficient activity, inconsistent references, or sample gaps/jumps |

The terms are raw controller-output values, before the nonlinear `tanh` mapping
to a temperature correction and before setpoint limits and device rounding.
They are not temperatures or power percentages. Gains and terms are published
from the same calculation; tuning changes take effect on the next calculation.
Scores refer to completed windows; the tuning state describes the current window.
Context changes discard scores. Inconclusive windows are not counted as rejects.

During hysteresis-off and climate-off periods, PID terms are zero and the integral
is reset. On re-enable, the first calculation has no derivative or elapsed-time
contribution from the disabled period. Hysteresis-off coasting remains observable
to the tuner while heating/cooling is enabled; climate-off aborts comparisons.

Run the portable regression tests and thermal simulation with:

```sh
for test in test_adaptive_pid test_pid_autotuner; do
  c++ -std=c++11 -Wall -Wextra -pedantic \
    -I components/mitsubishi_heatpump \
    "tests/$test.cpp" components/mitsubishi_heatpump/adaptive_pid.cpp \
    components/mitsubishi_heatpump/pid_autotuner.cpp -o "/tmp/$test"
  "/tmp/$test"
done
```

# Enable python
python3.12 -m venv --prompt '(.venv)' .venv
source .venv/bin/activate
pip3 install -r requirements.txt

# Upgrade esphome
```bash
pip index versions esphome
pip install --upgrade --force-reinstall -r requirements.txt
```
