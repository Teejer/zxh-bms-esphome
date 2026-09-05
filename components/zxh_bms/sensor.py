import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_MILLIVOLT,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
)

from . import CONF_ZXH_BMS_ID, ZxhBMS

DEPENDENCIES = ["zxh_bms"]

CONF_PROBE = "probe"
UNIT_AMPERE_HOURS = "Ah"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ZXH_BMS_ID): cv.use_id(ZxhBMS),
        cv.Optional("voltage"): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon="mdi:battery",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_VOLTAGE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("current"): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE,
            icon="mdi:flash",
            accuracy_decimals=3,
            device_class=DEVICE_CLASS_CURRENT,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("power"): sensor.sensor_schema(
            unit_of_measurement=UNIT_WATT,
            icon="mdi:flash",
            accuracy_decimals=2,
            device_class=DEVICE_CLASS_POWER,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("soc"): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon="mdi:battery",
            accuracy_decimals=0,
            device_class=DEVICE_CLASS_BATTERY,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("cell_min"): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIVOLT,
            icon="mdi:battery",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("cell_max"): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIVOLT,
            icon="mdi:battery",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("cell_delta"): sensor.sensor_schema(
            unit_of_measurement=UNIT_MILLIVOLT,
            icon="mdi:battery",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("remaining_capacity"): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE_HOURS,
            icon="mdi:battery",
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("cycles"): sensor.sensor_schema(
            icon="mdi:counter",
            accuracy_decimals=0,
            state_class=STATE_CLASS_TOTAL_INCREASING,
        ),
        cv.Optional("health"): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon="mdi:battery",
            accuracy_decimals=0,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("mos_temperature"): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon="mdi:thermometer",
            accuracy_decimals=1,
            device_class=DEVICE_CLASS_TEMPERATURE,
            state_class=STATE_CLASS_MEASUREMENT,
        ),
        cv.Optional("cell_count"): sensor.sensor_schema(
            icon="mdi:counter",
            accuracy_decimals=0,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional("nominal_voltage"): sensor.sensor_schema(
            unit_of_measurement=UNIT_VOLT,
            icon="mdi:battery",
            accuracy_decimals=1,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional("nominal_capacity"): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE_HOURS,
            icon="mdi:battery",
            accuracy_decimals=1,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional("full_capacity"): sensor.sensor_schema(
            unit_of_measurement=UNIT_AMPERE_HOURS,
            icon="mdi:battery",
            accuracy_decimals=1,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional("temperature"): cv.All(
            cv.ensure_list(
                sensor.sensor_schema(
                    unit_of_measurement=UNIT_CELSIUS,
                    icon="mdi:thermometer",
                    accuracy_decimals=1,
                    device_class=DEVICE_CLASS_TEMPERATURE,
                    state_class=STATE_CLASS_MEASUREMENT,
                ).extend({cv.Required(CONF_PROBE): cv.int_range(min=1, max=4)})
            ),
            cv.Length(max=4),
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_ZXH_BMS_ID])

    if (conf := config.get("voltage")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_voltage_sensor(sens))
    if (conf := config.get("current")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_current_sensor(sens))
    if (conf := config.get("power")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_power_sensor(sens))
    if (conf := config.get("soc")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_soc_sensor(sens))
    if (conf := config.get("cell_min")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_cell_min_sensor(sens))
    if (conf := config.get("cell_max")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_cell_max_sensor(sens))
    if (conf := config.get("cell_delta")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_cell_delta_sensor(sens))
    if (conf := config.get("remaining_capacity")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_remaining_capacity_sensor(sens))
    if (conf := config.get("cycles")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_cycles_sensor(sens))
    if (conf := config.get("health")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_health_sensor(sens))
    if (conf := config.get("mos_temperature")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_mos_temperature_sensor(sens))
    if (conf := config.get("cell_count")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_cell_count_sensor(sens))
    if (conf := config.get("nominal_voltage")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_nominal_voltage_sensor(sens))
    if (conf := config.get("nominal_capacity")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_nominal_capacity_sensor(sens))
    if (conf := config.get("full_capacity")) is not None:
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_full_capacity_sensor(sens))
    for conf in config.get("temperature", []):
        sens = await sensor.new_sensor(conf)
        cg.add(paren.set_temperature_sensor(conf[CONF_PROBE], sens))
