from esphome import pins
import esphome.codegen as cg
from esphome.components import i2c, touchscreen
import esphome.config_validation as cv
from esphome.const import (
    CONF_ID,
    CONF_INTERRUPT_PIN,
    CONF_RESET_PIN,
)

# Progi filtra probek, w pikselach ekranu. Zero wylacza dany filtr.
CONF_PROG_POTWIERDZENIA = "prog_potwierdzenia"
CONF_PROG_SKOKU = "prog_skoku"
# Logowanie kazdej probki z kontrolera — tylko do diagnostyki.
CONF_DIAGNOSTYKA = "diagnostyka"

ns_ = cg.esphome_ns.namespace("gsl3680")

cls_ = ns_.class_(
    "GSL3680",
    touchscreen.Touchscreen,
    i2c.I2CDevice,
)

CONFIG_SCHEMA = (
    touchscreen.touchscreen_schema()
    .extend(
        {
            cv.GenerateID(): cv.declare_id(cls_),
            cv.Required(CONF_INTERRUPT_PIN): pins.internal_gpio_input_pin_schema,
            cv.Required(CONF_RESET_PIN): pins.internal_gpio_output_pin_schema,
            cv.Optional(CONF_PROG_POTWIERDZENIA, default=40): cv.int_range(min=0, max=1000),
            cv.Optional(CONF_PROG_SKOKU, default=150): cv.int_range(min=0, max=2000),
            cv.Optional(CONF_DIAGNOSTYKA, default=False): cv.boolean,
        }
    )
    .extend(i2c.i2c_device_schema(0x40))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await touchscreen.register_touchscreen(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_interrupt_pin(await cg.gpio_pin_expression(config.get(CONF_INTERRUPT_PIN))))
    cg.add(var.set_reset_pin(await cg.gpio_pin_expression(config.get(CONF_RESET_PIN))))
    cg.add(var.set_prog_potwierdzenia(config[CONF_PROG_POTWIERDZENIA]))
    cg.add(var.set_prog_skoku(config[CONF_PROG_SKOKU]))
    cg.add(var.set_diagnostyka(config[CONF_DIAGNOSTYKA]))
