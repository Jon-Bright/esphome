from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_SCAN, PLATFORM_ESP32, PLATFORM_ESP8266

CODEOWNERS = ["@Jon-Bright"]

dali_ns = cg.esphome_ns.namespace("dali_bus")
DALIBusComponent = dali_ns.class_("DALIBusComponent", cg.Component)

CONF_DALI_OUT_PIN = "dali_out_pin"
CONF_DALI_OUT_INVERT = "dali_out_invert"
CONF_DALI_IN_PIN = "dali_in_pin"
CONF_DALI_IN_INVERT = "dali_in_invert"
CONF_DALI_SCAN_DELAY = "dali_scan_delay"

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DALIBusComponent),
            cv.Required(CONF_DALI_OUT_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_DALI_OUT_INVERT, default=False): cv.boolean,
            cv.Required(CONF_DALI_IN_PIN): pins.internal_gpio_input_pin_schema,
            cv.Optional(CONF_DALI_IN_INVERT, default=False): cv.boolean,
            cv.Optional(CONF_SCAN, default=False): cv.boolean,
            cv.Optional(
                CONF_DALI_SCAN_DELAY, default="5s"
            ): cv.positive_time_period_seconds,
        }
    ).extend(cv.COMPONENT_SCHEMA),
    cv.only_on([PLATFORM_ESP32, PLATFORM_ESP8266]),
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    dali_out_pin = await cg.gpio_pin_expression(config[CONF_DALI_OUT_PIN])
    cg.add(var.set_dali_out_pin(dali_out_pin))
    cg.add(var.set_dali_out_invert(config[CONF_DALI_OUT_INVERT]))
    dali_in_pin = await cg.gpio_pin_expression(config[CONF_DALI_IN_PIN])
    cg.add(var.set_dali_in_pin(dali_in_pin))
    cg.add(var.set_dali_in_invert(config[CONF_DALI_IN_INVERT]))
    cg.add(var.set_scan(config[CONF_SCAN]))
    cg.add(var.set_dali_scan_delay(config[CONF_DALI_SCAN_DELAY]))
