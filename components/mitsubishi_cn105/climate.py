import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, climate, select, sensor, uart
from esphome.components.uart import UARTParityOptions
from esphome.const import (
    CONF_ACCURACY_DECIMALS,
    CONF_DEVICE_CLASS,
    CONF_DISABLED_BY_DEFAULT,
    CONF_ENTITY_CATEGORY,
    CONF_FAN_MODE,
    CONF_FORCE_UPDATE,
    CONF_ID,
    CONF_INTERNAL,
    CONF_MODE,
    CONF_NAME,
    CONF_STATE_CLASS,
    CONF_SWING_MODE,
    CONF_UART_ID,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_DURATION,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    UNIT_HOUR,
    UNIT_KILOWATT_HOURS,
    UNIT_WATT,
)

AUTO_LOAD = ["binary_sensor", "climate", "select", "sensor", "uart"]
DEPENDENCIES = ["uart"]

CONF_SUPPORTS = "supports"
CONF_HORIZONTAL_VANE_SELECT = "horizontal_vane_select"
CONF_VERTICAL_VANE_SELECT = "vertical_vane_select"
CONF_HORIZONTAL_VANE_MODES = "horizontal_vane_modes"
CONF_REMOTE_TEMPERATURE_SENSOR = "remote_temperature_sensor"
CONF_REMOTE_TEMPERATURE_TIMEOUT = "remote_temperature_timeout"
CONF_DEBOUNCE_DELAY = "debounce_delay"

DEFAULT_MODES = ["HEAT_COOL", "COOL", "HEAT", "DRY", "FAN_ONLY"]
DEFAULT_FAN_MODES = ["AUTO", "MIDDLE", "QUIET", "LOW", "MEDIUM", "HIGH"]
DEFAULT_SWING_MODES = ["OFF", "VERTICAL", "HORIZONTAL", "BOTH"]
VERTICAL_VANE_OPTIONS = ["swing", "auto", "up", "up_center", "center", "down_center", "down"]
DEFAULT_HORIZONTAL_VANE_OPTIONS = ["auto", "swing", "left", "left_center", "center", "right_center", "right"]

MitsubishiCN105Climate = cg.global_ns.class_(
    "MitsubishiCN105Climate", climate.Climate, cg.Component, uart.UARTDevice
)
MitsubishiACSelect = cg.global_ns.class_("MitsubishiACSelect", select.Select, cg.Component)

SELECT_SCHEMA = select.select_schema(MitsubishiACSelect).extend(
    {cv.GenerateID(): cv.declare_id(MitsubishiACSelect)}
)

CONFIG_SCHEMA = climate.climate_schema(MitsubishiCN105Climate).extend(
    {
        cv.GenerateID(): cv.declare_id(MitsubishiCN105Climate),
        cv.GenerateID(CONF_UART_ID): cv.use_id(uart.UARTComponent),
        cv.Optional(CONF_INTERNAL, default=True): cv.boolean,
        cv.Optional(CONF_UPDATE_INTERVAL, default="2s"): cv.All(
            cv.update_interval, cv.Range(max=cv.TimePeriod(milliseconds=8500))
        ),
        cv.Optional(CONF_REMOTE_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
        cv.Optional(CONF_REMOTE_TEMPERATURE_TIMEOUT, default="never"): cv.update_interval,
        cv.Optional(CONF_DEBOUNCE_DELAY, default="100ms"): cv.update_interval,
        cv.Optional(CONF_HORIZONTAL_VANE_SELECT): SELECT_SCHEMA,
        cv.Optional(CONF_VERTICAL_VANE_SELECT): SELECT_SCHEMA,
        cv.Optional(CONF_SUPPORTS, default={}): cv.Schema(
            {
                cv.Optional(CONF_MODE, default=DEFAULT_MODES): cv.ensure_list(climate.validate_climate_mode),
                cv.Optional(CONF_FAN_MODE, default=DEFAULT_FAN_MODES): cv.ensure_list(climate.validate_climate_fan_mode),
                cv.Optional(CONF_SWING_MODE, default=DEFAULT_SWING_MODES): cv.ensure_list(climate.validate_climate_swing_mode),
                cv.Optional(CONF_HORIZONTAL_VANE_MODES, default=DEFAULT_HORIZONTAL_VANE_OPTIONS): cv.ensure_list(cv.string),
            }
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def _diagnostics(var, climate_id):
    common = {
        CONF_DISABLED_BY_DEFAULT: False,
        CONF_INTERNAL: False,
        CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
    }
    def diagnostic_id(suffix, entity_type):
        return cv.declare_id(entity_type)(f"{climate_id.id}_{suffix}")

    connected = await binary_sensor.new_binary_sensor(
        {CONF_ID: diagnostic_id("connected", binary_sensor.BinarySensor), CONF_NAME: "Connected", **common}
    )
    cg.add(var.set_device_state_connected_sensor(connected))
    active = await binary_sensor.new_binary_sensor(
        {CONF_ID: diagnostic_id("device_power_on", binary_sensor.BinarySensor), CONF_NAME: "Device power on", **common}
    )
    cg.add(var.set_device_state_active_sensor(active))
    operating = await binary_sensor.new_binary_sensor(
        {CONF_ID: diagnostic_id("operating", binary_sensor.BinarySensor), CONF_NAME: "Operating", **common}
    )
    cg.add(var.set_device_status_operating_sensor(operating))

    async def new_sensor(suffix, name, unit, device_class, state_class, decimals):
        return await sensor.new_sensor(
            {
                CONF_ID: diagnostic_id(suffix, sensor.Sensor),
                CONF_NAME: name,
                CONF_UNIT_OF_MEASUREMENT: unit,
                CONF_DEVICE_CLASS: device_class,
                CONF_STATE_CLASS: sensor.STATE_CLASSES[state_class],
                CONF_ACCURACY_DECIMALS: decimals,
                CONF_FORCE_UPDATE: False,
                **common,
            }
        )

    current = await new_sensor("current_temperature", "Current temperature", UNIT_CELSIUS, DEVICE_CLASS_TEMPERATURE, "measurement", 1)
    cg.add(var.set_device_status_current_temperature_sensor(current))
    outside = await new_sensor("outside_temperature", "Outside temperature", UNIT_CELSIUS, DEVICE_CLASS_TEMPERATURE, "measurement", 1)
    cg.add(var.set_device_status_outside_temperature_sensor(outside))
    frequency = await new_sensor("compressor_frequency", "Compressor frequency", UNIT_HERTZ, DEVICE_CLASS_FREQUENCY, "measurement", 1)
    cg.add(var.set_device_status_compressor_frequency_sensor(frequency))
    power = await new_sensor("input_power", "Input power", UNIT_WATT, DEVICE_CLASS_POWER, "measurement", 0)
    cg.add(var.set_device_status_input_power_sensor(power))
    energy = await new_sensor("energy", "kWh", UNIT_KILOWATT_HOURS, DEVICE_CLASS_ENERGY, "total_increasing", 1)
    cg.add(var.set_device_status_kwh_sensor(energy))
    runtime = await new_sensor("runtime_hours", "Runtime hours", UNIT_HOUR, DEVICE_CLASS_DURATION, "total_increasing", 2)
    cg.add(var.set_device_status_runtime_hours_sensor(runtime))
    setpoint = await new_sensor("setpoint", "Set Point", UNIT_CELSIUS, DEVICE_CLASS_TEMPERATURE, "measurement", 1)
    cg.add(var.set_device_set_point_sensor(setpoint))


async def to_code(config):
    uart_var = await cg.get_variable(config[CONF_UART_ID])
    var = cg.new_Pvariable(config[CONF_ID], uart_var)
    cg.add(uart_var.set_data_bits(8))
    cg.add(uart_var.set_parity(UARTParityOptions.UART_CONFIG_PARITY_EVEN))
    cg.add(uart_var.set_stop_bits(1))

    traits = var.config_traits()
    supports = config[CONF_SUPPORTS]
    for mode in supports[CONF_MODE]:
        if mode != "OFF":
            cg.add(traits.add_supported_mode(climate.CLIMATE_MODES[mode]))
    for mode in supports[CONF_FAN_MODE]:
        cg.add(traits.add_supported_fan_mode(climate.CLIMATE_FAN_MODES[mode]))
    for mode in supports[CONF_SWING_MODE]:
        cg.add(traits.add_supported_swing_mode(climate.CLIMATE_SWING_MODES[mode]))

    cg.add(var.set_remote_temp_timeout(config[CONF_REMOTE_TEMPERATURE_TIMEOUT]))
    cg.add(var.set_debounce_delay(config[CONF_DEBOUNCE_DELAY]))
    if CONF_REMOTE_TEMPERATURE_SENSOR in config:
        remote_sensor = await cg.get_variable(config[CONF_REMOTE_TEMPERATURE_SENSOR])
        cg.add(var.set_remote_temperature_sensor(remote_sensor))

    if CONF_VERTICAL_VANE_SELECT in config:
        conf = config[CONF_VERTICAL_VANE_SELECT]
        vane = await select.new_select(conf, options=VERTICAL_VANE_OPTIONS)
        await cg.register_component(vane, conf)
        cg.add(var.set_vertical_vane_select(vane))
    if CONF_HORIZONTAL_VANE_SELECT in config:
        conf = config[CONF_HORIZONTAL_VANE_SELECT]
        vane = await select.new_select(conf, options=supports[CONF_HORIZONTAL_VANE_MODES])
        await cg.register_component(vane, conf)
        cg.add(var.set_horizontal_vane_select(vane))

    await _diagnostics(var, config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)
