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