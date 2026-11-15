import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, climate, sensor
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_UPDATE_INTERVAL,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_RUNNING,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

AUTO_LOAD = ["binary_sensor", "climate", "sensor"]

CONF_CLIMATE_ID = "climate_id"
CONF_TEMPERATURE_SENSOR = "temperature_sensor"
CONF_READY_SENSOR = "ready_sensor"
CONF_SETTLE_TIME = "settle_time"

CONF_PID = "pid"
CONF_KP = "kp"
CONF_KI = "ki"
CONF_KD = "kd"
CONF_MAX_ADJUSTMENT_UNDER = "max_adjustment_under"
CONF_MAX_ADJUSTMENT_OVER = "max_adjustment_over"

CONF_HYSTERESIS = "hysteresis"
CONF_ON_THRESHOLD = "on_threshold"
CONF_OFF_THRESHOLD = "off_threshold"
CONF_MIN_CYCLE_DURATION = "min_cycle_duration"

CONF_INTERNAL_POWER = "internal_power"
CONF_DEVICE_POWER = "device_power"
CONF_OPERATING = "operating"
CONF_DEVICE_CURRENT_TEMPERATURE = "device_current_temperature"
CONF_DEVICE_SET_POINT = "device_set_point"
CONF_PID_SET_POINT = "pid_set_point"

workflow_climate_ns = cg.esphome_ns.namespace("workflow_climate")
WorkflowClimate = workflow_climate_ns.class_(
    "WorkflowClimate", climate.Climate, cg.Component
)

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
        cv.Optional(
            CONF_MIN_CYCLE_DURATION, default="60s"
        ): cv.positive_time_period_milliseconds,
    }
)


def _temperature_sensor_schema():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_TEMPERATURE,
        state_class=STATE_CLASS_MEASUREMENT,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )


def _power_sensor_schema(device_class):
    return binary_sensor.binary_sensor_schema(
        device_class=device_class,
        entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
    )


# Diagnostic entities are created by default. The names match the entities the
# integrated mitsubishi_heatpump component used to publish.
CONFIG_SCHEMA = cv.All(
    climate.climate_schema(WorkflowClimate)
    .extend(
        {
            cv.Required(CONF_CLIMATE_ID): cv.use_id(climate.Climate),
            cv.Optional(CONF_TEMPERATURE_SENSOR): cv.use_id(sensor.Sensor),
            cv.Optional(CONF_READY_SENSOR): cv.use_id(binary_sensor.BinarySensor),
            cv.Optional(
                CONF_UPDATE_INTERVAL, default="2s"
            ): cv.positive_time_period_milliseconds,
            # Defaults to 2 x update_interval + 5s.
            cv.Optional(CONF_SETTLE_TIME): cv.positive_time_period_milliseconds,
            cv.Optional(CONF_PID): PID_SCHEMA,
            cv.Optional(CONF_HYSTERESIS): HYSTERESIS_SCHEMA,
            cv.Optional(
                CONF_INTERNAL_POWER, default={CONF_NAME: "Internal power on"}
            ): _power_sensor_schema(DEVICE_CLASS_POWER),
            cv.Optional(
                CONF_DEVICE_POWER, default={CONF_NAME: "Device power on"}
            ): _power_sensor_schema(DEVICE_CLASS_POWER),
            cv.Optional(
                CONF_OPERATING, default={CONF_NAME: "Operating"}
            ): _power_sensor_schema(DEVICE_CLASS_RUNNING),
            cv.Optional(
                CONF_DEVICE_CURRENT_TEMPERATURE,
                default={CONF_NAME: "Current temperature"},
            ): _temperature_sensor_schema(),
            cv.Optional(
                CONF_DEVICE_SET_POINT, default={CONF_NAME: "Set Point"}
            ): _temperature_sensor_schema(),
            cv.Optional(
                CONF_PID_SET_POINT, default={CONF_NAME: "PID Set Point"}
            ): _temperature_sensor_schema(),
        }
    )
    .extend(cv.COMPONENT_SCHEMA),
    cv.has_at_least_one_key(CONF_PID, CONF_HYSTERESIS),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await climate.register_climate(var, config)

    inner = await cg.get_variable(config[CONF_CLIMATE_ID])
    cg.add(var.set_inner_climate(inner))

    if CONF_TEMPERATURE_SENSOR in config:
        temperature = await cg.get_variable(config[CONF_TEMPERATURE_SENSOR])
        cg.add(var.set_temperature_sensor(temperature))
    if CONF_READY_SENSOR in config:
        ready = await cg.get_variable(config[CONF_READY_SENSOR])
        cg.add(var.set_ready_sensor(ready))

    update_interval = config[CONF_UPDATE_INTERVAL]
    cg.add(var.set_update_interval(update_interval))
    if CONF_SETTLE_TIME in config:
        settle_ms = config[CONF_SETTLE_TIME].total_milliseconds
    else:
        settle_ms = 2 * update_interval.total_milliseconds + 5000
    cg.add(var.set_settle_time(settle_ms))

    if CONF_PID in config:
        conf = config[CONF_PID]
        cg.add(var.set_pid_enabled(True))
        cg.add(var.set_kp(conf[CONF_KP]))
        cg.add(var.set_ki(conf[CONF_KI]))
        cg.add(var.set_kd(conf[CONF_KD]))
        cg.add(var.set_max_adjustment_under(conf[CONF_MAX_ADJUSTMENT_UNDER]))
        cg.add(var.set_max_adjustment_over(conf[CONF_MAX_ADJUSTMENT_OVER]))

    if CONF_HYSTERESIS in config:
        conf = config[CONF_HYSTERESIS]
        cg.add(var.set_hysteresis_enabled(True))
        cg.add(var.set_hysteresis_on(conf[CONF_ON_THRESHOLD]))
        cg.add(var.set_hysteresis_off(conf[CONF_OFF_THRESHOLD]))
        cg.add(var.set_min_cycle_duration(conf[CONF_MIN_CYCLE_DURATION]))

    for key, setter in (
        (CONF_INTERNAL_POWER, var.set_internal_power_sensor),
        (CONF_DEVICE_POWER, var.set_device_power_sensor),
        (CONF_OPERATING, var.set_operating_sensor),
    ):
        cg.add(setter(await binary_sensor.new_binary_sensor(config[key])))

    for key, setter in (
        (CONF_DEVICE_CURRENT_TEMPERATURE, var.set_device_current_temperature_sensor),
        (CONF_DEVICE_SET_POINT, var.set_device_set_point_sensor),
    ):
        cg.add(setter(await sensor.new_sensor(config[key])))
    if CONF_PID in config:
        cg.add(
            var.set_pid_set_point_sensor(
                await sensor.new_sensor(config[CONF_PID_SET_POINT])
            )
        )
