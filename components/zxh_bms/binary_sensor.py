import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor

from . import CONF_ZXH_BMS_ID, ZxhBMS

DEPENDENCIES = ["zxh_bms"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ZXH_BMS_ID): cv.use_id(ZxhBMS),
        cv.Optional("protection_active"): binary_sensor.binary_sensor_schema(
            icon="mdi:alert",
        ),
        cv.Optional("balancing"): binary_sensor.binary_sensor_schema(
            icon="mdi:scale-balance",
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_ZXH_BMS_ID])

    if (conf := config.get("protection_active")) is not None:
        sens = await binary_sensor.new_binary_sensor(conf)
        cg.add(paren.set_protection_binary_sensor(sens))
    if (conf := config.get("balancing")) is not None:
        sens = await binary_sensor.new_binary_sensor(conf)
        cg.add(paren.set_balancing_binary_sensor(sens))
