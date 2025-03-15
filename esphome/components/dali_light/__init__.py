import esphome.codegen as cg
from esphome.components import dali_bus, light
import esphome.config_validation as cv
from esphome.const import CONF_OUTPUT_ID

CODEOWNERS = ["@Jon-Bright"]

dali_light_ns = cg.esphome_ns.namespace("dali_light")
DALILight = dali_light_ns.class_(
    "DALILight",
    light.LightOutput,
    cg.Component,
)

CONF_DALI_ID = "dali_id"
CONF_LIGHT_ID = "light_id"

CONFIG_SCHEMA = light.BRIGHTNESS_ONLY_LIGHT_SCHEMA.extend(
    {
        cv.GenerateID(CONF_OUTPUT_ID): cv.declare_id(DALILight),
        cv.GenerateID(CONF_DALI_ID): cv.use_id(dali_bus.DALIBusComponent),
        cv.Required(CONF_LIGHT_ID): cv.int_range(min=0, max=63),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_OUTPUT_ID])
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_DALI_ID])
    cg.add(var.set_dali_bus(parent))
    cg.add(var.set_light_id(config[CONF_LIGHT_ID]))
