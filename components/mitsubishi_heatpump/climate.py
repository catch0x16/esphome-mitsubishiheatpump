import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import (
    climate,
    select,
    binary_sensor,
    sensor,
)

from esphome.components.logger import HARDWARE_UART_TO_SERIAL
from esphome.const import (
    CONF_ID,
    CONF_HARDWARE_UART,
    CONF_BAUD_RATE,
    CONF_RX_PIN,
    CONF_TX_PIN,
    CONF_UPDATE_INTERVAL,
    CONF_MODE,
    CONF_FAN_MODE,
    CONF_SWING_MODE,
    PLATFORM_ESP8266,
    CONF_NAME,
    CONF_DISABLED_BY_DEFAULT,
    CONF_INTERNAL,
    CONF_ENTITY_CATEGORY,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_DEVICE_CLASS,
    CONF_STATE_CLASS,
    CONF_ACCURACY_DECIMALS,
    CONF_FORCE_UPDATE,
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

AUTO_LOAD = ["climate", "select", "binary_sensor"]

CONF_SUPPORTS = "supports"

CONF_CONTROL_PARAMETERS = "control_parameters"
CONF_KP = "kp"
CONF_KI = "ki"
CONF_KD = "kd"
CONF_MAX_ADJUSTMENT_UNDER = "maxAdjustmentUnder"
CONF_MAX_ADJUSTMENT_OVER = "maxAdjustmentOver"
CONF_HYSTERISIS_OFF = "hysterisisOff"
CONF_HYSTERISIS_ON = "hysterisisOn"
CONF_OFFSET_ADJUSTMENT = "offsetAdjustment"

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

# Remote temperature timeout configuration
CONF_REMOTE_OPERATING_TIMEOUT = "remote_temperature_operating_timeout_minutes"
CONF_REMOTE_IDLE_TIMEOUT = "remote_temperature_idle_timeout_minutes"
CONF_REMOTE_PING_TIMEOUT = "remote_temperature_ping_timeout_minutes"

MitsubishiHeatPump = cg.global_ns.class_(
    "MitsubishiHeatPump", climate.Climate, cg.PollingComponent
)

MitsubishiACSelect = cg.global_ns.class_(
    "MitsubishiACSelect", select.Select, cg.Component
)

def valid_uart(uart):
    if CORE.is_esp8266:
        uarts = ["UART0"]  # UART1 is tx-only
    elif CORE.is_esp32:
        uarts = ["UART0", "UART1", "UART2"]
    else:
        raise NotImplementedError

    return cv.one_of(*uarts, upper=True)(uart)


InternalPowerOnSensor = cg.global_ns.class_("InternalPowerOn", binary_sensor.BinarySensor, cg.Component)

SELECT_SCHEMA = select.select_schema(MitsubishiACSelect).extend(
    {cv.GenerateID(CONF_ID): cv.declare_id(MitsubishiACSelect)}
)

CONFIG_SCHEMA = climate.climate_schema(MitsubishiHeatPump).extend(
    {
        cv.GenerateID(): cv.declare_id(MitsubishiHeatPump),
        cv.Optional(CONF_HARDWARE_UART, default="UART0"): valid_uart,
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
        # Add selects for vertical and horizontal vane positions
        cv.Optional(CONF_HORIZONTAL_SWING_SELECT): SELECT_SCHEMA,
        cv.Optional(CONF_VERTICAL_SWING_SELECT): SELECT_SCHEMA,
        cv.Required(CONF_CONTROL_PARAMETERS): cv.Schema(
            {
                cv.Optional(CONF_KP, default=0.0): cv.float_,
                cv.Optional(CONF_KI, default=0.0): cv.float_,
                cv.Optional(CONF_KD, default=0.0): cv.float_,
                cv.Optional(CONF_MAX_ADJUSTMENT_UNDER, default=2.0): cv.float_,
                cv.Optional(CONF_MAX_ADJUSTMENT_OVER, default=2.0): cv.float_,
                cv.Optional(CONF_HYSTERISIS_OFF, default=0.25): cv.float_,
                cv.Optional(CONF_HYSTERISIS_ON, default=0.25): cv.float_,
                cv.Optional(CONF_OFFSET_ADJUSTMENT, default=0.00): cv.float_,
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
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)

@coroutine
def to_code(config):
    serial = HARDWARE_UART_TO_SERIAL[PLATFORM_ESP8266][config[CONF_HARDWARE_UART]]
    var = cg.new_Pvariable(config[CONF_ID], cg.RawExpression(f"&{serial}"))

    if CONF_BAUD_RATE in config:
        cg.add(var.set_baud_rate(config[CONF_BAUD_RATE]))

    if CONF_RX_PIN in config:
        cg.add(var.set_rx_pin(config[CONF_RX_PIN]))

    if CONF_TX_PIN in config:
        cg.add(var.set_tx_pin(config[CONF_TX_PIN]))

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

    if CONF_HORIZONTAL_SWING_SELECT in config:
        conf = config[CONF_HORIZONTAL_SWING_SELECT]
        swing_select = yield select.new_select(conf, options=HORIZONTAL_SWING_OPTIONS)
        yield cg.register_component(swing_select, conf)
        cg.add(var.set_horizontal_vane_select(swing_select))

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
        CONF_NAME: "Device state connected",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_state_connected_sensor(device_state_connected_sensor_var))

    device_state_active_sensor_var = yield binary_sensor.new_binary_sensor({
        CONF_ID: cv.declare_id(binary_sensor.BinarySensor)("device_state_active"),
        CONF_NAME: "Device state active",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_state_active_sensor(device_state_active_sensor_var))

    device_status_operating_sensor_var = yield binary_sensor.new_binary_sensor({
        CONF_ID: cv.declare_id(binary_sensor.BinarySensor)("device_status_operating"),
        CONF_NAME: "Device status operating",
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    })
    cg.add(var.set_device_status_operating_sensor(device_status_operating_sensor_var))

    device_status_current_temperature_sensor_var = yield sensor.new_sensor({
        CONF_ID: cv.declare_id(sensor.Sensor)("device_status_current_temperature"),
        CONF_NAME: "Device current temperature",
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

    device_status_compressor_frequency_sensor_var = yield sensor.new_sensor({
        CONF_ID: cv.declare_id(sensor.Sensor)("device_status_compressor_frequency"),
        CONF_NAME: "Device status compressor frequency",
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
        CONF_NAME: "Device status input power",
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
        CONF_NAME: "Device status kWh",
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
        CONF_NAME: "Device status runtime hours",
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
        CONF_NAME: "PID Set Point Correction",
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
        CONF_NAME: "Device Set Point",
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
    cg.add(var.set_offset_adjustment(params[CONF_OFFSET_ADJUSTMENT]))

    cg.add_library(
        name="HeatPump",
        repository="https://github.com/catch0x16/HeatPump#1b7edd3acca721ed1630a7973a4b19e60b3dd9d6",
        version=None, # this appears to be ignored?
    )
