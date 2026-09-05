"""Strumien MJPEG dekodowany sprzetowo i wyswietlany w LVGL (ESP32-P4).

Zalozenie: pobieranie i dekodowanie dzieje sie we wlasnym zadaniu FreeRTOS,
poza glowna petla ESPHome. Glowna petla wyłącznie przepina gotowa ramke,
bo LVGL nie jest bezpieczne watkowo.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ID, CONF_URL, CONF_WIDTH, CONF_HEIGHT

CODEOWNERS = ["@mariuszczarnasiak"]
DEPENDENCIES = ["esp32", "lvgl"]

CONF_FPS = "fps"
CONF_BUFFER_SIZE = "buffer_size"

mjpeg_lvgl_ns = cg.esphome_ns.namespace("mjpeg_lvgl")
MjpegLvgl = mjpeg_lvgl_ns.class_("MjpegLvgl", cg.Component)


def _tylko_p4(config):
    from esphome.components.esp32 import get_esp32_variant
    from esphome.components.esp32.const import VARIANT_ESP32P4

    if get_esp32_variant() != VARIANT_ESP32P4:
        raise cv.Invalid("mjpeg_lvgl wymaga ESP32-P4 (sprzetowy dekoder JPEG)")
    return config


# Komponent moze miec kilka instancji: osobna na strumien z kamery i osobna
# na okladki plyt, kazda z wlasnym zadaniem i wlasnymi buforami.
_INSTANCJA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MjpegLvgl),
            # bez url komponent dziala w trybie pojedynczych obrazow (pobierz())
            cv.Optional(CONF_URL, default=""): cv.string,
            cv.Required(CONF_WIDTH): cv.int_range(16, 1280),
            cv.Required(CONF_HEIGHT): cv.int_range(16, 1280),
            cv.Optional(CONF_FPS, default=10): cv.int_range(1, 30),
            # bufor na pojedyncza ramke JPEG przed dekodowaniem
            cv.Optional(CONF_BUFFER_SIZE, default=131072): cv.int_range(16384, 1048576),
        }
    ).extend(cv.COMPONENT_SCHEMA),
    _tylko_p4,
)

CONFIG_SCHEMA = cv.ensure_list(_INSTANCJA)


async def to_code(config):
    for conf in config:
        var = cg.new_Pvariable(conf[CONF_ID])
        await cg.register_component(var, conf)
        cg.add(var.set_url(conf[CONF_URL]))
        cg.add(var.set_size(conf[CONF_WIDTH], conf[CONF_HEIGHT]))
        cg.add(var.set_fps(conf[CONF_FPS]))
        cg.add(var.set_buffer_size(conf[CONF_BUFFER_SIZE]))
    # sprzetowy dekoder JPEG i akcelerator PPA z ESP-IDF
    cg.add_build_flag("-DMJPEG_LVGL")
