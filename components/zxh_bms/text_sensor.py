import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import CONF_ZXH_BMS_ID, ZxhBMS

DEPENDENCIES = ["zxh_bms"]

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_ZXH_BMS_ID): cv.use_id(ZxhBMS),
        cv.Optional("cells"): text_sensor.text_sensor_schema(
            icon="mdi:battery",
        ),
        cv.Optional("protection"): text_sensor.text_sensor_schema(
            icon="mdi:information",
        ),
        cv.Optional("firmware_version"): text_sensor.text_sensor_schema(
            icon="mdi:information",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional("manufacture_date"): text_sensor.text_sensor_schema(
            icon="mdi:information",
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
        ),
        cv.Optional("device_name"): text_sensor.text_sensor_schema(
            icon="mdi:bluetooth",
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    paren = await cg.get_variable(config[CONF_ZXH_BMS_ID])

    if (conf := config.get("cells")) is not None:
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(paren.set_cells_text_sensor(sens))
    if (conf := config.get("protection")) is not None:
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(paren.set_protection_text_sensor(sens))
    if (conf := config.get("firmware_version")) is not None:
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(paren.set_firmware_version_text_sensor(sens))
    if (conf := config.get("manufacture_date")) is not None:
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(paren.set_manufacture_date_text_sensor(sens))
    if (conf := config.get("device_name")) is not None:
        sens = await text_sensor.new_text_sensor(conf)
        cg.add(paren.set_device_name_text_sensor(sens))
