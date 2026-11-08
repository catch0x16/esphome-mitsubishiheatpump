# Controller and tuner vision

Read the [PID controller and tuner vision](docs/control-vision.md) for the control objectives, tuning priorities, and installation context that guide changes.

The [bidirectional tuner learning requirements](docs/tuner-learning-requirements.md) capture the agreed direction for exploration, independent confirmation, automatic persistence, and YAML export. The opt-in [experimental implementation and configuration guide](docs/gain-learning.md) describes defaults, diagnostics, persistence, validation, and limitations. It has not been deployed to the HVAC devices.

# References
https://github.com/echavet/MitsubishiCN105ESPHome

# Enable python
python3.12 -m venv --prompt '(.venv)' .venv
source .venv/bin/activate
pip3 install -r requirements.txt

# Upgrade esphome
```bash
pip index versions esphome
pip install --upgrade --force-reinstall -r requirements.txt
```
