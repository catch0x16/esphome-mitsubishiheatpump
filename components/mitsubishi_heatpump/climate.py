import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (
    climate,
    text_sensor,
    button,
    select,
    binary_sensor,
    sensor,
    uart,
)
from esphome.components.uart import UARTParityOptions

from esphome.const import (
    CONF_ID,
    CONF_BAUD_RATE,
    CONF_RX_PIN,
    CONF_TX_PIN,
    CONF_UPDATE_INTERVAL,
    CONF_MODE,
    CONF_FAN_MODE,
    CONF_SWING_MODE,
    CONF_NAME,
    CONF_DISABLED_BY_DEFAULT,
    CONF_INTERNAL,
    CONF_ENTITY_CATEGORY,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_DEVICE_CLASS,
    CONF_STATE_CLASS,
    CONF_ACCURACY_DECIMALS,
    CONF_FORCE_UPDATE,
    CONF_UART_ID,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_DURATION,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    UNIT_WATT,
    UNIT_KILOWATT_HOURS,
    UNIT_HOUR,
)
from esphome.core import CORE, coroutine

sensor_ns = cg.esphome_ns.namespace("sensor")
StateClasses = sensor_ns.enum("StateClass")

AUTO_LOAD = ["climate", "select", "binary_sensor", "sensor", "text_sensor", "button", "uart"]
DEPENDENCIES = ["uart"]

CONF_SUPPORTS = "supports"
CONF_SUPPORTS_HORIZONTAL_VANE_MODE = "horizontal_vane_mode"
CONF_REMOTE_TEMP_TIMEOUT = "remote_temperature_timeout"
CONF_DEBOUNCE_DELAY = "debounce_delay"

CONF_CONTROL_PARAMETERS = "control_parameters"
CONF_KP = "kp"
CONF_KI = "ki"
CONF_KD = "kd"
CONF_MAX_ADJUSTMENT_UNDER = "maxAdjustmentUnder"
CONF_MAX_ADJUSTMENT_OVER = "maxAdjustmentOver"
CONF_HYSTERISIS_OFF = "hysterisisOff"
CONF_HYSTERISIS_ON = "hysterisisOn"

CONF_HORIZONTAL_SWING_SELECT = "horizontal_vane_select"
CONF_VERTICAL_SWING_SELECT = "vertical_vane_select"
DEFAULT_CLIMATE_MODES = ["HEAT_COOL", "COOL", "HEAT", "DRY", "FAN_ONLY"]
DEFAULT_FAN_MODES = ["AUTO", "DIFFUSE", "LOW", "MEDIUM", "MIDDLE", "HIGH"]
DEFAULT_SWING_MODES = ["OFF", "VERTICAL"]

# P=4.09 I=0.058 D=3.38
DEFAULT_KP = 4.0
DEFAULT_KI = 0.02
DEFAULT_KD = 0.01

HORIZONTAL_SWING_OPTIONS = [
    "auto",
    "swing",
    "left",
    "left_center",
    "center",
    "right_center",
    "right",
]
VERTICAL_SWING_OPTIONS = ["swing", "auto", "up", "up_center", "center", "down_center", "down"]

DEFAULT_CLIMATE_MODES = ["AUTO", "COOL", "HEAT", "DRY", "FAN_ONLY"]
DEFAULT_FAN_MODES = ["AUTO", "MIDDLE", "QUIET", "LOW", "MEDIUM", "HIGH"]
DEFAULT_SWING_MODES = ["OFF", "VERTICAL", "HORIZONTAL", "BOTH"]

# Remote temperature timeout configuration
CONF_REMOTE_OPERATING_TIMEOUT = "remote_temperature_operating_timeout_minutes"
CONF_REMOTE_IDLE_TIMEOUT = "remote_temperature_idle_timeout_minutes"
CONF_REMOTE_PING_TIMEOUT = "remote_temperature_ping_timeout_minutes"

MitsubishiHeatPump = cg.global_ns.class_(
    "MitsubishiHeatPump", climate.Climate, cg.Component, uart.UARTDevice
)

MitsubishiACSelect = cg.global_ns.class_(
    "MitsubishiACSelect", select.Select, cg.Component
)

InternalPowerOnSensor = cg.global_ns.class_("InternalPowerOn", binary_sensor.BinarySensor, cg.Component)

SELECT_SCHEMA = select.select_schema(MitsubishiACSelect).extend(
    {cv.GenerateID(CONF_ID): cv.declare_id(MitsubishiACSelect)}
)

# Explicit opt-in: without this block the legacy adaptive controller remains.
CONF_GAIN_LEARNING = "gain_learning"
PidLearningButton = cg.global_ns.class_("PidLearningButton", button.Button)

def normalize_learning_inputs(config):
    import logging
    if not isinstance(config, dict):
        raise cv.Invalid("gain_learning must be a mapping of settings and ESPHome sensor IDs")
    config = dict(config)
    for old, new in (("outdoor_temperature", "outdoor_temperature_sensor"),
                     ("external_heat", "fireplace_active_sensor"),
                     ("context_valid", "fireplace_state_valid_sensor"),
                     ("fusion_weight", "fusion_wall_weight_sensor")):
        if old in config:
            if new in config:
                raise cv.Invalid(f"Use only {new}, not both {old} and {new}")
            config[new] = config.pop(old)
            logging.getLogger(__name__).warning("gain_learning.%s is deprecated; use %s (an ESPHome sensor ID)", old, new)
    if "mode" in config:
        cv.one_of("heat", "cool", lower=True)(config.pop("mode"))
        logging.getLogger(__name__).warning("Remove gain_learning.mode: heating/cooling are now detected automatically")
    return config

def validate_learning(config):
    import math
    for value in config.values():
        if isinstance(value,float) and not math.isfinite(value):
            raise cv.Invalid("learning settings must be finite")
    for value in config["minimum_gains"] + config["maximum_gains"]:
        if not math.isfinite(value):
            raise cv.Invalid("gain bounds must be finite")
    if config["maximum_gains"][0] > 250 or config["maximum_gains"][1] > 10 or config["maximum_gains"][2] > 7200:
        raise cv.Invalid("maximum_gains must not exceed [250, 10, 7200]")
    if config["confirmation_pairs"] < 12:
        raise cv.Invalid("confirmation_pairs must be at least 12 for a useful fixed-look sign test")
    if config["max_pair_attempts"] < max(config["discovery_pairs"], config["confirmation_pairs"]):
        raise cv.Invalid("max_pair_attempts must cover both discovery and confirmation pairs")
    if config["observable_time"].total_milliseconds > config["period"].total_milliseconds:
        raise cv.Invalid("observable_time must not exceed period")
    for low, high in zip(config["minimum_gains"], config["maximum_gains"]):
        if low > high:
            raise cv.Invalid("minimum_gains must not exceed maximum_gains")
    if "fireplace_state_valid_sensor" in config and "fireplace_active_sensor" not in config:
        raise cv.Invalid("fireplace_state_valid_sensor requires fireplace_active_sensor")
    return config

def bounded_duration(value):
    value = cv.positive_time_period_milliseconds(value)
    if value.total_milliseconds > 604800000:
        raise cv.Invalid("learning durations must not exceed seven days")
    return value

GAIN_LEARNING_SCHEMA = cv.All(normalize_learning_inputs, cv.Schema({
    cv.Optional("enabled", default=True): cv.boolean,
    cv.Optional("outdoor_temperature_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("fireplace_active_sensor"): cv.use_id(binary_sensor.BinarySensor),
    cv.Optional("fireplace_state_valid_sensor"): cv.use_id(binary_sensor.BinarySensor),
    cv.Required("other_units_operating"): cv.All(cv.ensure_list(cv.use_id(binary_sensor.BinarySensor)), cv.Length(min=2, max=2)),
    cv.Optional("fusion_wall_weight_sensor"): cv.use_id(sensor.Sensor),
    cv.Optional("period", default="2h"): bounded_duration,
    cv.Optional("settling_time", default="30min"): bounded_duration,
    cv.Optional("hold_time", default="24h"): bounded_duration,
    cv.Optional("observable_time", default="2min"): bounded_duration,
    cv.Optional("discovery_pairs", default=4): cv.int_range(min=4, max=32),
    cv.Optional("confirmation_pairs", default=24): cv.int_range(min=12, max=64),
    cv.Optional("max_pair_attempts", default=48): cv.int_range(min=12, max=256),
    cv.Optional("local_step", default=.1): cv.float_range(min=.01, max=.25),
    cv.Optional("wider_step", default=.5): cv.float_range(min=.25, max=1),
    cv.Optional("minimum_improvement", default=.05): cv.float_range(min=.01, max=.5),
    cv.Optional("guard_relative", default=.05): cv.float_range(min=0, max=.25),
    cv.Optional("guard_absolute", default=.02): cv.float_range(min=0, max=.1),
    cv.Optional("peak_guard", default=.1): cv.float_range(min=0, max=.25),
    cv.Optional("max_command_difference", default=.5): cv.float_range(min=.5, max=1),
    cv.Optional("outdoor_pair_difference", default=3): cv.float_range(min=.1, max=5),
    cv.Optional("context_pair_difference", default=.25): cv.float_range(min=0, max=.5),
    cv.Optional("minimum_gains", default=[0,0,0]): cv.All(cv.ensure_list(cv.float_range(min=0)), cv.Length(min=3,max=3)),
    cv.Optional("maximum_gains", default=[250,10,7200]): cv.All(cv.ensure_list(cv.float_range(min=0)), cv.Length(min=3,max=3)),
}), validate_learning)

CONFIG_SCHEMA = climate.climate_schema(MitsubishiHeatPump).extend(
    {
        cv.GenerateID(): cv.declare_id(MitsubishiHeatPump),
        cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_BAUD_RATE): cv.positive_int,
        cv.Optional(CONF_REMOTE_OPERATING_TIMEOUT): cv.positive_int,
        cv.Optional(CONF_REMOTE_IDLE_TIMEOUT): cv.positive_int,
        cv.Optional(CONF_REMOTE_PING_TIMEOUT): cv.positive_int,
        cv.Optional(CONF_RX_PIN): cv.positive_int,
        cv.Optional(CONF_TX_PIN): cv.positive_int,
        # If polling interval is greater than 9 seconds, the HeatPump library
        # reconnects, but doesn't then follow up with our data request.
        cv.Optional(CONF_UPDATE_INTERVAL, default="2000ms"): cv.All(
            cv.update_interval, cv.Range(max=cv.TimePeriod(milliseconds=8500))
        ),
        cv.Optional(CONF_REMOTE_TEMP_TIMEOUT, default="never"): cv.All(
            cv.update_interval
        ),
        cv.Optional(CONF_DEBOUNCE_DELAY, default="100ms"): cv.All(
            cv.update_interval
        ),
        # Add selects for vertical and horizontal vane positions
        cv.Optional(CONF_HORIZONTAL_SWING_SELECT): SELECT_SCHEMA,
        cv.Optional(CONF_VERTICAL_SWING_SELECT): SELECT_SCHEMA,
        cv.Optional(CONF_GAIN_LEARNING): GAIN_LEARNING_SCHEMA,
        cv.Required(CONF_CONTROL_PARAMETERS): cv.Schema(
            {
                cv.Optional(CONF_KP, default=0.0): cv.float_,
                cv.Optional(CONF_KI, default=0.0): cv.float_,
                cv.Optional(CONF_KD, default=0.0): cv.float_,
                cv.Optional("heat"): cv.Schema({cv.Required(k): cv.float_range(min=0) for k in (CONF_KP,CONF_KI,CONF_KD)}),
                cv.Optional("cool"): cv.Schema({cv.Required(k): cv.float_range(min=0) for k in (CONF_KP,CONF_KI,CONF_KD)}),
                cv.Optional(CONF_MAX_ADJUSTMENT_UNDER, default=2.0): cv.float_,
                cv.Optional(CONF_MAX_ADJUSTMENT_OVER, default=2.0): cv.float_,
                cv.Optional(CONF_HYSTERISIS_OFF, default=0.25): cv.float_,
                cv.Optional(CONF_HYSTERISIS_ON, default=0.25): cv.float_,
            }
        ),

        # Optionally override the supported ClimateTraits.
        cv.Optional(CONF_SUPPORTS, default={}): cv.Schema(
            {
                cv.Optional(CONF_MODE, default=DEFAULT_CLIMATE_MODES):
                    cv.ensure_list(climate.validate_climate_mode),
                cv.Optional(CONF_FAN_MODE, default=DEFAULT_FAN_MODES):
                    cv.ensure_list(climate.validate_climate_fan_mode),
                cv.Optional(CONF_SWING_MODE, default=DEFAULT_SWING_MODES):
                    cv.ensure_list(climate.validate_climate_swing_mode),
                cv.Optional(CONF_SUPPORTS_HORIZONTAL_VANE_MODE):
                    cv.ensure_list(cv.string),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

def validate_profile_configuration(config):
    params = config[CONF_CONTROL_PARAMETERS]
    if ("heat" in params or "cool" in params) and CONF_GAIN_LEARNING not in config:
        raise cv.Invalid("per-mode PID gains require gain_learning (enabled: false is allowed)")
    return config

CONFIG_SCHEMA = cv.All(CONFIG_SCHEMA, validate_profile_configuration)

@coroutine
def to_code(config):
    uart_id_object = config[CONF_UART_ID]
    uart_var = yield cg.get_variable(uart_id_object)
    var = cg.new_Pvariable(config[CONF_ID], uart_var)

    #cg.add(var.set_installer_mode(config[CONF_INSTALLER_MODE]))

    cg.add(uart_var.set_data_bits(8))
    cg.add(uart_var.set_parity(UARTParityOptions.UART_CONFIG_PARITY_EVEN))
    cg.add(uart_var.set_stop_bits(1))

    if CONF_REMOTE_OPERATING_TIMEOUT in config:
        cg.add(var.set_remote_operating_timeout_minutes(config[CONF_REMOTE_OPERATING_TIMEOUT]))

    if CONF_REMOTE_IDLE_TIMEOUT in config:
        cg.add(var.set_remote_idle_timeout_minutes(config[CONF_REMOTE_IDLE_TIMEOUT]))

    if CONF_REMOTE_PING_TIMEOUT in config:
        cg.add(var.set_remote_ping_timeout_minutes(config[CONF_REMOTE_PING_TIMEOUT]))

    supports = config[CONF_SUPPORTS]
    traits = var.config_traits()

    for mode in supports[CONF_MODE]:
        if mode == "OFF":
            continue
        cg.add(traits.add_supported_mode(climate.CLIMATE_MODES[mode]))

    for mode in supports[CONF_FAN_MODE]:
        cg.add(traits.add_supported_fan_mode(climate.CLIMATE_FAN_MODES[mode]))

    for mode in supports[CONF_SWING_MODE]:
        cg.add(traits.add_supported_swing_mode(
            climate.CLIMATE_SWING_MODES[mode]
        ))

    horizontal_vane_options = []

    if CONF_SUPPORTS in config:
        supports = config[CONF_SUPPORTS]
        traits = var.config_traits()

        # Configurer les modes supportés
        supported_modes = supports.get(CONF_MODE, DEFAULT_CLIMATE_MODES)
        for mode_str in supported_modes:
            if mode_str == "OFF":
                continue
            if mode_str in climate.CLIMATE_MODES:
                cg.add(traits.add_supported_mode(climate.CLIMATE_MODES[mode_str]))

        # Configure the horizontal vane options
        horizontal_vane_options = supports.get(CONF_SUPPORTS_HORIZONTAL_VANE_MODE, [])

        for fan_mode_str in supports.get(CONF_FAN_MODE, DEFAULT_FAN_MODES):
            if fan_mode_str in climate.CLIMATE_FAN_MODES:
                cg.add(
                    traits.add_supported_fan_mode(
                        climate.CLIMATE_FAN_MODES[fan_mode_str]
                    )
                )
        for swing_mode_str in supports.get(CONF_SWING_MODE, DEFAULT_SWING_MODES):
            if swing_mode_str in climate.CLIMATE_SWING_MODES:
                cg.add(
                    traits.add_supported_swing_mode(
                        climate.CLIMATE_SWING_MODES[swing_mode_str]
                    )
                )

    cg.add(var.set_remote_temp_timeout(config[CONF_REMOTE_TEMP_TIMEOUT]))
    cg.add(var.set_debounce_delay(config[CONF_DEBOUNCE_DELAY]))

    if CONF_HORIZONTAL_SWING_SELECT in config:
        conf_item = config[CONF_HORIZONTAL_SWING_SELECT]

        swing_select_var = yield select.new_select(conf_item, options=[])
        if horizontal_vane_options:
            options_vector = cg.RawExpression(
                "std::vector<std::string>{"
                + ", ".join([f'"{opt}"' for opt in horizontal_vane_options])
                + "}"
            )
        else:
            options_vector = cg.RawExpression("std::vector<std::string>{}")
        cg.add(var.set_horizontal_vane_select(swing_select_var, options_vector))

    if CONF_VERTICAL_SWING_SELECT in config:
        conf = config[CONF_VERTICAL_SWING_SELECT]
        swing_select = yield select.new_select(conf, options=VERTICAL_SWING_OPTIONS)
        yield cg.register_component(swing_select, conf)
        cg.add(var.set_vertical_vane_select(swing_select))

    internal_power_on_sensor_var = yield binary_sensor.new_binary_sensor({
        CONF_ID: cv.declare_id(binary_sensor.BinarySensor)("internal_power_on"),
        CONF_NAME: "Internal power on",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_internal_power_on_sensor(internal_power_on_sensor_var))

    device_state_connected_sensor_var = yield binary_sensor.new_binary_sensor({
        CONF_ID: cv.declare_id(binary_sensor.BinarySensor)("device_state_connected"),
        CONF_NAME: "Connected",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_state_connected_sensor(device_state_connected_sensor_var))

    device_state_active_sensor_var = yield binary_sensor.new_binary_sensor({
        CONF_ID: cv.declare_id(binary_sensor.BinarySensor)("device_state_active"),
        CONF_NAME: "Device power on",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_state_active_sensor(device_state_active_sensor_var))

    device_status_operating_sensor_var = yield binary_sensor.new_binary_sensor({
        CONF_ID: cv.declare_id(binary_sensor.BinarySensor)("device_status_operating"),
        CONF_NAME: "Operating",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_status_operating_sensor(device_status_operating_sensor_var))

    device_status_current_temperature_sensor_var = yield sensor.new_sensor({
        CONF_ID: cv.declare_id(sensor.Sensor)("device_status_current_temperature"),
        CONF_NAME: "Current temperature",
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
        CONF_STATE_CLASS: StateClasses.STATE_CLASS_MEASUREMENT,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_FORCE_UPDATE: False,
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_status_current_temperature_sensor(device_status_current_temperature_sensor_var))

    device_status_outside_temperature_sensor_var = yield sensor.new_sensor({
        CONF_ID: cv.declare_id(sensor.Sensor)("device_status_outside_temperature"),
        CONF_NAME: "Outside temperature",
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
        CONF_STATE_CLASS: StateClasses.STATE_CLASS_MEASUREMENT,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_FORCE_UPDATE: False,
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_status_outside_temperature_sensor(device_status_outside_temperature_sensor_var))

    device_status_compressor_frequency_sensor_var = yield sensor.new_sensor({
        CONF_ID: cv.declare_id(sensor.Sensor)("device_status_compressor_frequency"),
        CONF_NAME: "Compressor frequency",
        CONF_UNIT_OF_MEASUREMENT: UNIT_HERTZ,
        CONF_DEVICE_CLASS: DEVICE_CLASS_FREQUENCY,
        CONF_STATE_CLASS: StateClasses.STATE_CLASS_MEASUREMENT,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_FORCE_UPDATE: False,
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_status_compressor_frequency_sensor(device_status_compressor_frequency_sensor_var))

    device_status_input_power_sensor_var = yield sensor.new_sensor({
        CONF_ID: cv.declare_id(sensor.Sensor)("device_status_input_power"),
        CONF_NAME: "Input power",
        CONF_UNIT_OF_MEASUREMENT: UNIT_WATT,
        CONF_DEVICE_CLASS: DEVICE_CLASS_POWER,
        CONF_STATE_CLASS: StateClasses.STATE_CLASS_MEASUREMENT,
        CONF_ACCURACY_DECIMALS: 0,
        CONF_FORCE_UPDATE: False,
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_status_input_power_sensor(device_status_input_power_sensor_var))

    device_status_kwh_sensor_var = yield sensor.new_sensor({
        CONF_ID: cv.declare_id(sensor.Sensor)("device_status_kwh"),
        CONF_NAME: "kWh",
        CONF_UNIT_OF_MEASUREMENT: UNIT_KILOWATT_HOURS,
        CONF_DEVICE_CLASS: DEVICE_CLASS_ENERGY,
        CONF_STATE_CLASS: StateClasses.STATE_CLASS_TOTAL_INCREASING,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_FORCE_UPDATE: False,
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_status_kwh_sensor(device_status_kwh_sensor_var))

    device_status_runtime_hours_sensor_var = yield sensor.new_sensor({
        CONF_ID: cv.declare_id(sensor.Sensor)("device_status_runtime_hours"),
        CONF_NAME: "Runtime hours",
        CONF_UNIT_OF_MEASUREMENT: UNIT_HOUR,
        CONF_DEVICE_CLASS: DEVICE_CLASS_DURATION,
        CONF_STATE_CLASS: StateClasses.STATE_CLASS_TOTAL_INCREASING,
        CONF_ACCURACY_DECIMALS: 2,
        CONF_FORCE_UPDATE: False,
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_status_runtime_hours_sensor(device_status_runtime_hours_sensor_var))

    pid_set_point_correction_sensor_var = yield sensor.new_sensor({
        CONF_ID: cv.declare_id(sensor.Sensor)("pid_set_point_correction"),
        CONF_NAME: "PID Set Point",
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
        CONF_STATE_CLASS: StateClasses.STATE_CLASS_MEASUREMENT,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_FORCE_UPDATE: False,
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_pid_set_point_correction_sensor(pid_set_point_correction_sensor_var))

    device_set_point_sensor_var = yield sensor.new_sensor({
        CONF_ID: cv.declare_id(sensor.Sensor)("device_set_point"),
        CONF_NAME: "Set Point",
        CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
        CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
        CONF_STATE_CLASS: StateClasses.STATE_CLASS_MEASUREMENT,
        CONF_ACCURACY_DECIMALS: 1,
        CONF_FORCE_UPDATE: False,
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_set_point_sensor(device_set_point_sensor_var))

    if CONF_GAIN_LEARNING in config:
        import hashlib
        import json
        from pathlib import Path
        if not CORE.is_esp32:
            raise cv.Invalid("gain_learning currently requires ESP32 flash preferences")
        learning = config[CONF_GAIN_LEARNING]
        for gain, low, high in zip((config[CONF_CONTROL_PARAMETERS][k] for k in (CONF_KP,CONF_KI,CONF_KD)), learning["minimum_gains"],learning["maximum_gains"]):
            if not low <= gain <= high:
                raise cv.Invalid("configured PID gains must lie within gain_learning bounds")
        fields = {
            "period_ms": "period", "settling_ms": "settling_time", "hold_ms": "hold_time", "observable_ms": "observable_time",
            "discovery_pairs": "discovery_pairs", "confirmation_pairs": "confirmation_pairs", "max_pair_attempts": "max_pair_attempts",
            "step": "local_step", "wider_step": "wider_step", "improvement": "minimum_improvement",
            "guard_relative": "guard_relative", "guard_absolute": "guard_absolute", "peak_guard": "peak_guard",
            "max_command_difference": "max_command_difference", "outdoor_pair_difference": "outdoor_pair_difference",
            "context_pair_difference": "context_pair_difference",
        }
        assignments = []
        for cpp, yaml in fields.items():
            value = learning[yaml]
            if hasattr(value,"total_milliseconds"):
                value = value.total_milliseconds
            assignments.append(f"c.{cpp} = {value};")
        for cpp, yaml in (("minimum","minimum_gains"),("maximum","maximum_gains")):
            for i, value in enumerate(learning[yaml]):
                assignments.append(f"c.{cpp}[{i}] = {value};")
        expression = "[](){ PidGainLearner::Config c; " + " ".join(assignments) + " return c; }()"
        # Exclude enabled and initial gains so freezing and copying a verified
        # triplet to YAML preserve compatible records. Record restore separately
        # rejects an unrelated change of YAML starting gains.
        identity = {k: str(v) for k,v in learning.items() if k != "enabled"}
        identity["control_limits"] = {k: str(v) for k,v in config[CONF_CONTROL_PARAMETERS].items() if k not in (CONF_KP,CONF_KI,CONF_KD,"heat","cool")}
        identity["visual"] = str(config.get("visual",{}))
        digest = hashlib.sha256(json.dumps(identity,sort_keys=True).encode())
        for source in ("pid_gain_learner.cpp","pid_gain_learner.h","adaptive_pid.cpp","adaptive_pid.h","pid_workflowstep.cpp","pid_workflowstep.h"):
            digest.update((Path(__file__).parent/source).read_bytes())
        fingerprint = int.from_bytes(digest.digest()[:4],"little")
        cg.add(var.configure_gain_learning(cg.RawExpression(expression), learning["enabled"], fingerprint))
        for index, mode in enumerate(("heat","cool")):
            params = config[CONF_CONTROL_PARAMETERS]
            values = [params.get(mode, params)[key] for key in (CONF_KP,CONF_KI,CONF_KD)]
            if not all(low <= gain <= high for gain,low,high in zip(values,learning["minimum_gains"],learning["maximum_gains"])):
                raise cv.Invalid(f"control_parameters.{mode} gains must lie within gain_learning bounds")
            cg.add(var.set_learning_initial(index,*values))
        for key, method in (("outdoor_temperature_sensor","set_learning_outdoor_sensor"),("fireplace_active_sensor","set_learning_fireplace_sensor"),("fireplace_state_valid_sensor","set_learning_fireplace_valid_sensor"),("fusion_wall_weight_sensor","set_learning_fusion_sensor")):
            if key in learning:
                source = yield cg.get_variable(learning[key])
                cg.add(getattr(var,method)(source))
        for key in learning["other_units_operating"]:
            source = yield cg.get_variable(key)
            cg.add(var.add_learning_other_sensor(source))
        for i,name in enumerate(("Kp","Ki","Kd","Revision")):
            diagnostic = yield sensor.new_sensor({
                CONF_ID: cv.declare_id(sensor.Sensor)(f"pid_learning_{name.lower()}"),
                CONF_NAME: f"PID {name}" if i < 3 else "PID Verified Revision",
                CONF_STATE_CLASS: StateClasses.STATE_CLASS_MEASUREMENT,
                CONF_ACCURACY_DECIMALS: 9 if i < 3 else 0,
                CONF_FORCE_UPDATE: False,
                CONF_DISABLED_BY_DEFAULT: False, CONF_INTERNAL: False,
                CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
            })
            cg.add(var.set_learning_diagnostic(i,diagnostic))
        status = yield text_sensor.new_text_sensor({
            CONF_ID: cv.declare_id(text_sensor.TextSensor)("pid_learning_status"),
            CONF_NAME: "PID Tuning State", CONF_INTERNAL: False, CONF_DISABLED_BY_DEFAULT: False,
            CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
        })
        cg.add(var.set_learning_status(status))
        for rollback,label in ((False,"Reset Learned PID Gains"),(True,"Rollback PID Gains")):
            control = yield button.new_button({
                CONF_ID: cv.declare_id(PidLearningButton)("pid_rollback" if rollback else "pid_reset"),
                CONF_NAME: label, CONF_INTERNAL: False, CONF_DISABLED_BY_DEFAULT: False,
                CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_CONFIG,
            })
            cg.add(control.set_parent(var)); cg.add(control.set_rollback(rollback))

    yield cg.register_component(var, config)
    yield climate.register_climate(var, config)

    params = config[CONF_CONTROL_PARAMETERS]
    cg.add(var.set_kp(params[CONF_KP]))
    cg.add(var.set_ki(params[CONF_KI]))
    cg.add(var.set_kd(params[CONF_KD]))
    cg.add(var.set_max_adjustment_under(params[CONF_MAX_ADJUSTMENT_UNDER]))
    cg.add(var.set_max_adjustment_over(params[CONF_MAX_ADJUSTMENT_OVER]))
    cg.add(var.set_hysterisis_off(params[CONF_HYSTERISIS_OFF]))
    cg.add(var.set_hysterisis_on(params[CONF_HYSTERISIS_ON]))
