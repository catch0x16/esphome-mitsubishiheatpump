# ESPHome Workflow Climate

`workflow_climate` is a climate that wraps another ESPHome climate and runs a
PID controller and a hysteresis (on/off) controller on top of it, entirely on
the device.

It keeps two states apart:

- **What the user sees**: the outer climate's mode and target, e.g. `HEAT, 21°C`.
- **What the device is told**: the inner climate may be switched off while the
  outer one still reads `HEAT`, and its setpoint is the PID-corrected value
  (e.g. `22.5°C`) rather than the user's target.

The wrapper only uses the generic ESPHome climate API, so it works with any
single-target climate. It was written for Mitsubishi heat pumps driven by
[echavet/MitsubishiCN105ESPHome](https://github.com/echavet/MitsubishiCN105ESPHome).

## Configuration

```yaml
external_components:
  - source: github://catch0x16/esphome-mitsubishiheatpump
    components: [workflow_climate]
  - source: github://echavet/MitsubishiCN105ESPHome
    components: [cn105]

uart:
  id: hp_uart
  baud_rate: 2400
  tx_pin: 17
  rx_pin: 16

climate:
  # The device driver. Hidden, so Home Assistant only sees the workflow climate.
  - platform: cn105
    id: hp
    name: "Heat pump device"
    internal: true
    uart_id: hp_uart
    update_interval: 2s
    remote_temperature_source:
      sensor_id: room_temperature
    supports:
      mode: [COOL, HEAT, FAN_ONLY, DRY]
      # dual_setpoint must stay off (the default)

  - platform: workflow_climate
    name: "HVAC"
    climate_id: hp
    temperature_sensor: room_temperature
    update_interval: 2s
    pid:
      kp: 1.5
      ki: 0.015
      kd: 0.0
      max_adjustment_under: 2.0
      max_adjustment_over: 2.0
    hysteresis:
      on_threshold: 0.25
      off_threshold: 0.5
      min_cycle_duration: 60s
```

| Option | Default | Meaning |
|---|---|---|
| `climate_id` | required | The inner climate to drive. |
| `temperature_sensor` | inner climate's temperature | Room temperature used by the workflows and shown as the current temperature. |
| `update_interval` | `2s` | How often the workflows run. |
| `settle_time` | `2 × update_interval + 5s` | How long after a power command the inner climate's reported power is ignored. |
| `ready_sensor` | none | Optional binary sensor; while it is off the workflows are paused. |
| `pid` | disabled | `kp`, `ki`, `kd`, and how far the device setpoint may move below/above the target (`max_adjustment_under` / `max_adjustment_over`). See [PID gains](#pid-gains). |
| `hysteresis` | disabled | Switch the device off once the room is `off_threshold` past the target and back on once it is `on_threshold` short of it, at most once per `min_cycle_duration`. |

At least one of `pid` and `hysteresis` is required. The standard climate
options (`name`, `visual`, ...) apply; the visual range defaults to the inner
climate's and can only be narrowed.

Diagnostic entities are created automatically: `Internal power on` (what the
workflow last commanded), `Device power on`, `Operating`, `Current temperature`
(the device's own reading), `Set Point` (the device's setpoint) and
`PID Set Point`, plus `PID Kp`, `PID Ki` and `Response rate` when the PID is
used. Each can be renamed or reconfigured under its key
(`internal_power`, `device_power`, `operating`, `device_current_temperature`,
`device_set_point`, `pid_set_point`, `pid_kp`, `pid_ki`, `response_rate`). Anything device-specific, such as
compressor frequency or energy, comes from the inner climate's own sensors.

### PID gains

The PID output is a setpoint offset in degrees, added to the target:

```
offset = kp * error + ki * integral(error) + kd * d(error)/dt      error = target - room
```

- `kp`: degrees of setpoint per degree of error. `kp: 1.0` asks for a setpoint
  1°C above the target when the room is 1°C below it.
- `ki`: degrees of setpoint per degree-second of accumulated error. `ki: 0.001`
  adds 0.06°C per minute for each degree of error.
- `kd`: degrees of setpoint per degree-per-second of change.

The offset is limited to `-max_adjustment_under .. +max_adjustment_over` when
heating (mirrored when cooling) and then rounded to the device's step. The
rounded value never exceeds those limits, so a limit should be a multiple of
the step: on a 0.5°C device `max_adjustment_over: 0.25` allows no upward move.
A warning is logged at startup when a limit is not a multiple of the step.
The integral stops accumulating while the offset is at a limit, and it is
reset on a target change and while the device is switched off.

### Continuous tuning

With `adaptive: true` (the default) `kp` and `ki` are retuned while the
controller runs, and the configured values are only the starting point.

The controller measures how fast the room warms (or cools) while the device is
actively operating, as temperature change per hour of operating time. One
measurement is taken per 30 minutes of operation, however short the individual
runs, and each moves the estimate 20% of the way. The gains then follow the
SIMC tuning rule for an integrating process:

```
kp = full_output_gap / (rate * tc)      ki = kp / (4 * tc)      tc = response_time + dead_time
```

A room that responds quickly gets a lower gain, a slow one a higher gain.
Heating and cooling are learned separately, the learned rates are saved across
reboots, and `kd` is never changed.

| `pid` option | Default | Meaning |
|---|---|---|
| `adaptive` | `true` | Retune `kp` and `ki` continuously. `false` keeps the configured gains. |
| `response_time` | `10min` | How quickly the loop should correct an error. Longer is gentler. |
| `dead_time` | `5min` | Delay before the room temperature reacts to a setpoint change. |
| `full_output_gap` | `0.75` | How far the device setpoint must be past the room temperature for the device to run at full output. |
| `kp_min` / `kp_max` | `0.25` / `4.0` | Bounds on the tuned `kp`. |

The gains in use are published as `PID Kp` and `PID Ki`, and the learned rate
as `Response rate` (°C/h). To make learned values the new starting point, copy
`PID Kp` and `PID Ki` into `kp` and `ki`.

`tools/room_model/replay.cpp` runs recorded operation through the same
estimator, to choose starting values or to check a change against real data.

## Behaviour

- The workflows run in `HEAT` and `COOL`. In every other mode, and for fan,
  swing, presets and humidity, the outer climate passes straight through.
- In `HEAT`/`COOL` the outer target is the user's. It is stored per mode and is
  never overwritten by the device's setpoint.
- Two-point (low/high) inner climates are rejected at setup.

### Changes made at the device (e.g. the IR remote)

A generic climate cannot say who changed it, and it gives no acknowledgement
that a command arrived. The wrapper therefore ignores the inner climate's
reported power for `settle_time` after each of its own power commands, and
only treats a change as external once its own command has been seen to take
effect.

| Situation | Result |
|---|---|
| Device on and confirmed, then reports off | External off: the outer climate turns `OFF`. |
| Commanded on, device never came on | Lost command: sent again after `min_cycle_duration`. |
| Outer `OFF`, device comes on | The outer climate follows the device's mode. |
| Device changes between running modes | The outer climate follows, using that mode's stored target. |
| Workflow switched the device off, device is on anyway | Logged as an error and the internal state is resynced; the next workflow run decides what to do. |
| Setpoint changed at the device in `HEAT`/`COOL` | Overwritten by the controller. |

After a reboot, a restored `HEAT`/`COOL` with the device off is kept as is and
hysteresis decides when to start it. Otherwise the outer climate starts from
the device's state.

## Development

Requires ESPHome 2026.8.1 or newer.

```bash
# Power tracking rules, no ESPHome needed
c++ -std=c++17 -Wall -Wextra -o /tmp/test_power_tracker tests/test_power_tracker.cpp && /tmp/test_power_tracker

# PID controller, no ESPHome needed
c++ -std=c++17 -Wall -Wextra -Icomponents/workflow_climate -o /tmp/test_adaptive_pid \
  tests/test_adaptive_pid.cpp components/workflow_climate/adaptive_pid.cpp && /tmp/test_adaptive_pid

# Response-rate estimator and tuning rule, no ESPHome needed
c++ -std=c++17 -Wall -Wextra -Icomponents/workflow_climate -o /tmp/test_response_rate_estimator \
  tests/test_response_rate_estimator.cpp && /tmp/test_response_rate_estimator

# Host build against the demo climate
esphome compile test.yaml

# End-to-end scenario (about a minute; watch the log)
esphome compile tests/fixtures/scenario.yaml
tests/fixtures/.esphome/build/workflow-scenario/.pioenvs/workflow-scenario/program

# Compile check against the upstream CN105 driver
esphome compile tests/fixtures/cn105.yaml
```

Layout of `components/workflow_climate/`:

- `workflow_climate.*`: the outer climate.
- `climate_device_adapter.*`: presents the inner climate to the controller.
- `power_tracker.h`: commanded versus reported power; no ESPHome dependency.
- `hysterisis_workflowstep.*`, `pid_workflowstep.*`, `adaptive_pid.*`: the controller.
- `response_rate_estimator.h`: online room response estimate and the tuning rule; no ESPHome dependency.
- `controller_types.h`: the interface between the controller and the device.
