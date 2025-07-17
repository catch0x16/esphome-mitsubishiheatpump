import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, climate, select, sensor, uart

from esphome.components.logger import HARDWARE_UART_TO_SERIAL
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_INTERNAL,
    CONF_HARDWARE_UART,
    CONF_BAUD_RATE,
    CONF_RX_PIN,
    CONF_TX_PIN,
    CONF_UPDATE_INTERVAL,
    CONF_MODE,
    CONF_FAN_MODE,
    CONF_SWING_MODE,
    PLATFORM_ESP8266
)
from esphome.core import CORE, coroutine

AUTO_LOAD = ["climate", "select", "binary_sensor"]

CONF_SUPPORTS = "supports"

CONF_INTERNAL_POWER_ON = "internal_power_on"
CONF_DEVICE_STATE_CONNECTED = "device_state_connected"
CONF_DEVICE_STATE_ACTIVE = "device_state_active"
CONF_DEVICE_STATUS_OPERATING = "device_status_operating"
CONF_DEVICE_STATUS_CURRENT_TEMPERATURE = "device_status_current_temperature"
CONF_DEVICE_STATUS_COMPRESSOR_FREQUENCY = "device_status_compressor_frequency"
CONF_DEVICE_STATUS_INPUT_POWER = "device_status_input_power"
CONF_DEVICE_STATUS_KWH = "device_status_kwh"
CONF_DEVICE_STATUS_RUNTIME_HOURS = "device_status_runtime_hours"
CONF_DEVICE_SET_POINT = "device_set_point"
CONF_PID_SET_POINT_CORRECTION = "pid_set_point_correction"

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
    "MitsubishiHeatPump", climate.Climate, cg.Component, uart.UARTDevice
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


SELECT_SCHEMA = select.select_schema(MitsubishiACSelect).extend(
    {cv.GenerateID(CONF_ID): cv.declare_id(MitsubishiACSelect)}
)

InternalPowerOnSensor = cg.global_ns.class_("InternalPowerOnSensor", binary_sensor.BinarySensor, cg.Component)
INTERNAL_POWER_ON_SCHEMA = binary_sensor.binary_sensor_schema(InternalPowerOnSensor, entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC).extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(InternalPowerOnSensor),
        cv.Optional(CONF_NAME): "Internal power on",
        cv.Optional(CONF_INTERNAL, default="false"): cv.boolean
    }
)

DeviceStateConnectedSensor = cg.global_ns.class_("DeviceStateConnectedSensor", binary_sensor.BinarySensor, cg.Component)
DEVICE_STATE_CONNECTED_SCHEMA = binary_sensor.binary_sensor_schema(DeviceStateConnectedSensor, entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC).extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(DeviceStateConnectedSensor),
        cv.Optional(CONF_NAME): "Device state connected",
        cv.Optional(CONF_INTERNAL, default="false"): cv.boolean
    }
)

DeviceStateActiveSensor = cg.global_ns.class_("DeviceStateActiveSensor", binary_sensor.BinarySensor, cg.Component)
DEVICE_STATE_ACTIVE_SCHEMA = binary_sensor.binary_sensor_schema(DeviceStateActiveSensor, entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC).extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(DeviceStateActiveSensor),
        cv.Optional(CONF_NAME): "Device state active",
        cv.Optional(CONF_INTERNAL, default="false"): cv.boolean
    }
)

PidSetPointCorrectionSensor = cg.global_ns.class_("PidSetPointCorrectionSensor", sensor.Sensor, cg.Component)
PID_SET_POINT_CORRECTION_SCHEMA = sensor.sensor_schema(PidSetPointCorrectionSensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC,
    unit_of_measurement="°C"
).extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(PidSetPointCorrectionSensor),
        cv.Optional(CONF_NAME): "PID Set Point Correction",
        cv.Optional(CONF_INTERNAL, default="false"): cv.boolean
    }
)

DeviceStatusOperatingSensor = cg.global_ns.class_("DeviceStatusOperatingSensor", binary_sensor.BinarySensor, cg.Component)
DEVICE_STATUS_OPERATING_SCHEMA = binary_sensor.binary_sensor_schema(DeviceStatusOperatingSensor, entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC).extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(DeviceStatusOperatingSensor),
        cv.Optional(CONF_NAME): "Device status operating",
        cv.Optional(CONF_INTERNAL, default="false"): cv.boolean
    }
)

DeviceStatusCurrentTemperatureSensor = cg.global_ns.class_("DeviceStatusCurrentTemperatureSensor", sensor.Sensor, cg.Component)
DEVICE_STATUS_CURRENT_TEMPERATURE_SCHEMA = sensor.sensor_schema(DeviceStatusCurrentTemperatureSensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC,
    unit_of_measurement="°C"
).extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(DeviceStatusCurrentTemperatureSensor),
        cv.Optional(CONF_NAME): "Device current temperature",
        cv.Optional(CONF_INTERNAL, default="false"): cv.boolean
    }
)

DeviceStatusCompressorFrequencySensor = cg.global_ns.class_("DeviceStatusCompressorFrequencySensor", sensor.Sensor, cg.Component)
DEVICE_STATUS_COMPRESSOR_FREQUENCY_SCHEMA = sensor.sensor_schema(DeviceStatusCompressorFrequencySensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC,
    unit_of_measurement="Hz",
    state_class="measurement"
).extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(DeviceStatusCompressorFrequencySensor),
        cv.Optional(CONF_NAME): "Device status compressor frequency",
        cv.Optional(CONF_INTERNAL, default="false"): cv.boolean
    }
)

DeviceStatusInputPowerSensor = cg.global_ns.class_("DeviceStatusInputPowerSensor", sensor.Sensor, cg.Component)
DEVICE_STATUS_INPUT_POWER_SCHEMA = sensor.sensor_schema(DeviceStatusInputPowerSensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC
).extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(DeviceStatusInputPowerSensor),
        cv.Optional(CONF_NAME): "Device status input power",
        cv.Optional(CONF_INTERNAL, default="false"): cv.boolean
    }
)

DeviceStatusKwhSensor = cg.global_ns.class_("DeviceStatusKwhSensor", sensor.Sensor, cg.Component)
DEVICE_STATUS_KWH_SCHEMA = sensor.sensor_schema(DeviceStatusKwhSensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC,
    unit_of_measurement="kWh",
    device_class='energy',
    state_class="total_increasing"
).extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(DeviceStatusKwhSensor),
        cv.Optional(CONF_NAME): "Device status kWh",
        cv.Optional(CONF_INTERNAL, default="false"): cv.boolean
    }
)

DeviceStatusRuntimeHoursSensor = cg.global_ns.class_("DeviceStatusRuntimeHoursSensor", sensor.Sensor, cg.Component)
DEVICE_STATUS_RUNTIME_HOURS_SCHEMA = sensor.sensor_schema(DeviceStatusRuntimeHoursSensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC,
    unit_of_measurement="h",
    device_class='duration',
    state_class="total_increasing"
).extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(DeviceStatusRuntimeHoursSensor),
        cv.Optional(CONF_NAME): "Device status runtime hours",
        cv.Optional(CONF_INTERNAL, default="false"): cv.boolean
    }
)

DeviceSetPointSensor = cg.global_ns.class_("DeviceSetPointSensor", sensor.Sensor, cg.Component)
DEVICE_SET_POINT_SCHEMA = sensor.sensor_schema(DeviceSetPointSensor,
    entity_category=cv.ENTITY_CATEGORY_DIAGNOSTIC,
    unit_of_measurement="°C"
).extend(
    {
        cv.GenerateID(CONF_ID): cv.declare_id(DeviceStatusRuntimeHoursSensor),
        cv.Optional(CONF_NAME): "Device Set Point",
        cv.Optional(CONF_INTERNAL, default="false"): cv.boolean
    }
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
        cv.Optional(CONF_INTERNAL_POWER_ON): INTERNAL_POWER_ON_SCHEMA,
        cv.Optional(CONF_DEVICE_STATE_CONNECTED): DEVICE_STATE_CONNECTED_SCHEMA,
        cv.Optional(CONF_DEVICE_STATE_ACTIVE): DEVICE_STATE_ACTIVE_SCHEMA,
        cv.Optional(CONF_DEVICE_STATUS_OPERATING): DEVICE_STATUS_OPERATING_SCHEMA,
        cv.Optional(CONF_DEVICE_STATUS_CURRENT_TEMPERATURE): DEVICE_STATUS_CURRENT_TEMPERATURE_SCHEMA,
        cv.Optional(CONF_DEVICE_STATUS_COMPRESSOR_FREQUENCY): DEVICE_STATUS_COMPRESSOR_FREQUENCY_SCHEMA,
        cv.Optional(CONF_DEVICE_STATUS_INPUT_POWER): DEVICE_STATUS_INPUT_POWER_SCHEMA,
        cv.Optional(CONF_DEVICE_STATUS_KWH): DEVICE_STATUS_KWH_SCHEMA,
        cv.Optional(CONF_DEVICE_STATUS_RUNTIME_HOURS): DEVICE_STATUS_RUNTIME_HOURS_SCHEMA,
        cv.Optional(CONF_PID_SET_POINT_CORRECTION): PID_SET_POINT_CORRECTION_SCHEMA,
        cv.Optional(CONF_DEVICE_SET_POINT): DEVICE_SET_POINT_SCHEMA,
        
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

    
        x_conf_item = config[CONF_INTERNAL_POWER_ON] if CONF_INTERNAL_POWER_ON in config else {}
        sensor_var = yield sensor.new_sensor(x_conf_item)
        cg.add(var.set_input_power_sensor(sensor_var))

    if CONF_DEVICE_STATE_CONNECTED in config:
        conf_item = config[CONF_DEVICE_STATE_CONNECTED]
        sensor_var = yield sensor.new_sensor(conf_item)
        cg.add(var.set_device_state_connected_sensor(sensor_var))

    if CONF_DEVICE_STATE_ACTIVE in config:
        conf_item = config[CONF_DEVICE_STATE_ACTIVE]
        sensor_var = yield sensor.new_sensor(conf_item)
        cg.add(var.set_device_state_active_sensor(sensor_var))
    
    if CONF_DEVICE_STATUS_OPERATING in config:
        conf_item = config[CONF_DEVICE_STATUS_OPERATING]
        sensor_var = yield sensor.new_sensor(conf_item)
        cg.add(var.set_device_status_operating_sensor(sensor_var))

    if CONF_PID_SET_POINT_CORRECTION in config:
        conf_item = config[CONF_PID_SET_POINT_CORRECTION]
        sensor_var = yield sensor.new_sensor(conf_item)
        cg.add(var.set_pid_set_point_correction_sensor(sensor_var))
    
    if CONF_DEVICE_STATUS_CURRENT_TEMPERATURE in config:
        conf_item = config[CONF_DEVICE_STATUS_CURRENT_TEMPERATURE]
        sensor_var = yield sensor.new_sensor(conf_item)
        cg.add(var.set_device_status_current_temperature_sensor(sensor_var))

    if CONF_DEVICE_STATUS_COMPRESSOR_FREQUENCY in config:
        conf_item = config[CONF_DEVICE_STATUS_COMPRESSOR_FREQUENCY]
        sensor_var = yield sensor.new_sensor(conf_item)
        cg.add(var.set_device_status_compressor_frequency_sensor(sensor_var))

    if CONF_DEVICE_STATUS_INPUT_POWER in config:
        conf_item = config[CONF_DEVICE_STATUS_INPUT_POWER]
        sensor_var = yield sensor.new_sensor(conf_item)
        cg.add(var.set_device_status_input_power_sensor(sensor_var))

    if CONF_DEVICE_STATUS_KWH in config:
        conf_item = config[CONF_DEVICE_STATUS_KWH]
        sensor_var = yield sensor.new_sensor(conf_item)
        cg.add(var.set_device_status_kwh_sensor(sensor_var))

    if CONF_DEVICE_STATUS_RUNTIME_HOURS in config:
        conf_item = config[CONF_DEVICE_STATUS_RUNTIME_HOURS]
        sensor_var = yield sensor.new_sensor(conf_item)
        cg.add(var.set_device_status_runtime_hours_sensor(sensor_var))

    if CONF_DEVICE_SET_POINT in config:
        conf_item = config[CONF_DEVICE_SET_POINT]
        sensor_var = yield sensor.new_sensor(conf_item)
        cg.add(var.set_device_set_point_sensor(sensor_var))

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
