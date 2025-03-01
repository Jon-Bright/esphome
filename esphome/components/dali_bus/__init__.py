from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_SCAN

CODEOWNERS = ["@Jon-Bright"]

dali_ns = cg.esphome_ns.namespace("dali_bus")
DALIBusComponent = dali_ns.class_("DALIBusComponent", cg.Component)

CONF_DALI_OUT_PIN = "dali_out_pin"
CONF_DALI_IN_PIN = "dali_in_pin"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DALIBusComponent),
            cv.Required(CONF_DALI_OUT_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_DALI_IN_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_SCAN, default=True): cv.boolean,
        }
    ).extend(cv.COMPONENT_SCHEMA),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    dali_out_pin = await cg.gpio_pin_expression(config[CONF_DALI_OUT_PIN])
    cg.add(var.set_dali_out_pin(dali_out_pin))
    dali_in_pin = await cg.gpio_pin_expression(config[CONF_DALI_IN_PIN])
    cg.add(var.set_dali_in_pin(dali_in_pin))
    cg.add(var.set_scan(config[CONF_SCAN]))
