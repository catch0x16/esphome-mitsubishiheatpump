# References
https://github.com/echavet/MitsubishiCN105ESPHome

# PID diagnostics

The component automatically creates six enabled diagnostic sensors:

| Sensor | Meaning |
| --- | --- |
| PID Kp / PID Ki / PID Kd | Running gains used for the latest PID calculation |
| PID P Term | Kp × temperature error |
| PID I Term | Ki × accumulated error |
| PID D Term | Kd × change in error per second |

The terms are raw controller-output values, before the nonlinear `tanh` mapping
to a temperature correction and before setpoint limits and device rounding.
They are not temperatures or power percentages. Gains and terms are published
from the same calculation; adaptations take effect on the next calculation.

During hysteresis-off and climate-off periods, the terms are zero, integration
and adaptation stop, and learned gains are retained in memory. On re-enable,
the integral starts at zero and the first calculation has no derivative or
elapsed-time contribution from the disabled period. Rebooting starts again
from the configured YAML gains. Adaptation remains enabled during operation.

Run the standalone PID regression tests with:

```sh
c++ -std=c++11 -Wall -Wextra -pedantic \
  -I components/mitsubishi_heatpump \
  tests/test_adaptive_pid.cpp components/mitsubishi_heatpump/adaptive_pid.cpp \
  -o /tmp/test_adaptive_pid
/tmp/test_adaptive_pid
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
