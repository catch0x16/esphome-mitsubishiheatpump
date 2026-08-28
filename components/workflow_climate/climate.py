import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, sensor
from esphome.const import (
    CONF_ACCURACY_DECIMALS,
    CONF_DEVICE_CLASS,
    CONF_DISABLED_BY_DEFAULT,
    CONF_ENTITY_CATEGORY,
    CONF_FORCE_UPDATE,
    CONF_ID,
    CONF_NAME,
    CONF_STATE_CLASS,
    CONF_UNIT_OF_MEASUREMENT,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_TEMPERATURE,
    UNIT_CELSIUS,
)

AUTO_LOAD = ["climate", "sensor"]

CONF_CLIMATE_ID = "climate_id"
CONF_TEMPERATURE_SENSOR = "temperature_sensor"
CONF_COMMAND_ACK_TIMEOUT = "command_ack_timeout"
CONF_PID = "pid"
CONF_HYSTERESIS = "hysteresis"
CONF_KP = "kp"
CONF_KI = "ki"
CONF_KD = "kd"
CONF_MAX_ADJUSTMENT_UNDER = "max_adjustment_under"
CONF_MAX_ADJUSTMENT_OVER = "max_adjustment_over"
CONF_ON_THRESHOLD = "on_threshold"
CONF_OFF_THRESHOLD = "off_threshold"
CONF_MIN_CYCLE_DURATION = "min_cycle_duration"

WorkflowClimate = cg.global_ns.class_("WorkflowClimate", climate.Climate, cg.Component)

PID_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_KP, default=0.0): cv.float_,
        cv.Optional(CONF_KI, default=0.0): cv.float_,
        cv.Optional(CONF_KD, default=0.0): cv.float_,
        cv.Optional(CONF_MAX_ADJUSTMENT_UNDER, default=2.0): cv.positive_float,
        cv.Optional(CONF_MAX_ADJUSTMENT_OVER, default=2.0): cv.positive_float,
    }
)

HYSTERESIS_SCHEMA = cv.Schema(
    {
        cv.Optional(CONF_ON_THRESHOLD, default=0.25): cv.positive_float,
        cv.Optional(CONF_OFF_THRESHOLD, default=0.25): cv.positive_float,
        cv.Optional(CONF_MIN_CYCLE_DURATION, default="60s"): cv.positive_time_period_milliseconds,
    }
)

CONFIG_SCHEMA = cv.All(
    climate.climate_schema(WorkflowClimate).extend(
        {
            cv.GenerateID(): cv.declare_id(WorkflowClimate),
            cv.Required(CONF_CLIMATE_ID): cv.use_id(climate.Climate),
            cv.Optional(CONF_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_UPDATE_INTERVAL, default="2s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_COMMAND_ACK_TIMEOUT, default="30s"): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_PID): PID_SCHEMA,
            cv.Optional(CONF_HYSTERESIS): HYSTERESIS_SCHEMA,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.has_at_least_one_key(CONF_PID, CONF_HYSTERESIS),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    inner = await cg.get_variable(config[CONF_CLIMATE_ID])
    cg.add(var.set_inner_climate(inner))
    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))
    cg.add(var.set_command_ack_timeout(config[CONF_COMMAND_ACK_TIMEOUT]))

    if CONF_TEMPERATURE_SENSOR in config:
        temperature = await cg.get_variable(config[CONF_TEMPERATURE_SENSOR])
        cg.add(var.set_temperature_sensor(temperature))

    if CONF_PID in config:
        conf = config[CONF_PID]
        cg.add(var.set_pid_enabled(True))
        cg.add(var.set_kp(conf[CONF_KP]))
        cg.add(var.set_ki(conf[CONF_KI]))
        cg.add(var.set_kd(conf[CONF_KD]))
        cg.add(var.set_max_adjustment_under(conf[CONF_MAX_ADJUSTMENT_UNDER]))
        cg.add(var.set_max_adjustment_over(conf[CONF_MAX_ADJUSTMENT_OVER]))
        object_name = config.get(CONF_NAME, "Workflow Climate")
        pid_sensor = await sensor.new_sensor(
            {
                CONF_ID: cv.declare_id(sensor.Sensor)(f"{config[CONF_ID].id}_pid_setpoint"),
                CONF_NAME: f"{object_name} PID Set Point",
                CONF_UNIT_OF_MEASUREMENT: UNIT_CELSIUS,
                CONF_DEVICE_CLASS: DEVICE_CLASS_TEMPERATURE,
                CONF_STATE_CLASS: sensor.STATE_CLASSES["measurement"],
                CONF_ACCURACY_DECIMALS: 1,
                CONF_FORCE_UPDATE: False,
                CONF_DISABLED_BY_DEFAULT: False,
                CONF_ENTITY_CATEGORY: cg.EntityCategory.ENTITY_CATEGORY_DIAGNOSTIC,
            }
        )
        cg.add(var.set_pid_setpoint_sensor(pid_sensor))

    if CONF_HYSTERESIS in config:
        conf = config[CONF_HYSTERESIS]
        cg.add(var.set_hysteresis_enabled(True))
        cg.add(var.set_hysteresis_on(conf[CONF_ON_THRESHOLD]))
        cg.add(var.set_hysteresis_off(conf[CONF_OFF_THRESHOLD]))
        cg.add(var.set_min_cycle_duration(conf[CONF_MIN_CYCLE_DURATION]))

    await cg.register_component(var, config)
    await climate.register_climate(var, config)
