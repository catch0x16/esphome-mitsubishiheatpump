# ESPHome Mitsubishi CN105 climate

This repository provides two ESPHome climate components:

- `mitsubishi_cn105` exposes the physical Mitsubishi heat-pump state and sends CN105 commands.
- `workflow_climate` wraps any compatible single-target climate and applies PID and/or hysteresis locally on the ESP.

The split keeps the serial protocol independent from room-control policy. Home Assistant is not involved in the control path between the two climates.

## Configuration

```yaml
sensor:
  - platform: homeassistant
    id: room_temperature
    entity_id: sensor.room_temperature

uart:
  id: hp_uart
  tx_pin: 17
  rx_pin: 16
  baud_rate: 2400

climate:
  - platform: mitsubishi_cn105
    id: cn105_device
    uart_id: hp_uart
    internal: true
    remote_temperature_sensor: room_temperature

  - platform: workflow_climate
    name: "Heat Pump"
    climate_id: cn105_device
    temperature_sensor: room_temperature
    update_interval: 2s
    command_ack_timeout: 30s
    pid:
      kp: 0.0
      ki: 0.0
      kd: 0.0
      max_adjustment_under: 2.0
      max_adjustment_over: 2.0
    hysteresis:
      on_threshold: 0.25
      off_threshold: 0.25
      min_cycle_duration: 60s
```

The same local temperature sensor may feed both layers. The raw component injects it into CN105; the wrapper uses it for PID and hysteresis.

The wrapper exposes every single-target mode supported by the inner climate. PID and hysteresis operate only in `HEAT` and `COOL`; other modes pass through. Two-point climates are rejected during setup.

When PID is configured, the wrapper automatically creates an enabled diagnostic sensor containing the physical corrected setpoint. CN105 connection, physical setpoint, power, compressor, energy, and related diagnostics belong to the raw climate.

## Migration

This is a breaking replacement for `platform: mitsubishi_heatpump`. There is no compatibility schema or combined C++ API. Replace the old entry with an inner `mitsubishi_cn105` climate and an outer `workflow_climate` as above.

Configuration names use corrected snake case. Remove `control_parameters`, camel-case keys, and legacy `hysterisis` spellings.

## Development

Validation targets ESPHome 2026.6.0 with ESP-IDF:

```bash
uv run --with esphome==2026.6.0 esphome config test.yaml
uv run --with esphome==2026.6.0 esphome compile test.yaml
```

CN105 protocol work is derived from [MitsubishiCN105ESPHome](https://github.com/echavet/MitsubishiCN105ESPHome).
