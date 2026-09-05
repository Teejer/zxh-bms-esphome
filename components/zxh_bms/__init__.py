import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client
from esphome.const import CONF_ADDRESS, CONF_ID

CODEOWNERS = ["@teejer"]
DEPENDENCIES = ["ble_client"]
MULTI_CONF = True

zxh_bms_ns = cg.esphome_ns.namespace("zxh_bms")
ZxhBMS = zxh_bms_ns.class_("ZxhBMS", cg.PollingComponent, ble_client.BLEClientNode)

CONF_ZXH_BMS_ID = "zxh_bms_id"
CONF_MODE = "mode"
CONF_CYCLE_TIMEOUT = "cycle_timeout"

MODES = {
    # Keep the BLE link up and poll over it (best for a single battery).
    "persistent": False,
    # Round-robin: connect, read, disconnect, hand the radio to the next hub.
    "sequential": True,
}

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(ZxhBMS),
            cv.Optional(
                CONF_ADDRESS,
                default=0,
            ): cv.int_range(
                min=0,
                max=247,
            ),
            cv.Optional(CONF_MODE, default="persistent"): cv.enum(MODES),
            cv.Optional(CONF_CYCLE_TIMEOUT, default="90s"): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.polling_component_schema("30s"))
    .extend(ble_client.BLE_CLIENT_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)
    cg.add(var.set_modbus_address(config[CONF_ADDRESS]))
    cg.add(var.set_sequential(config[CONF_MODE]))
    cg.add(var.set_cycle_timeout(config[CONF_CYCLE_TIMEOUT]))
