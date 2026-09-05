#pragma once

// Sterownik pochodzi z https://github.com/jtenniswood/esphome-media-player
// (licencja ESPHome, patrz LICENSE.md). Dolozony zostal filtr wspolrzednych —
// kontroler GSL potrafi zglosic pojedyncza probke daleko od miejsca dotkniecia,
// przez co LVGL wybieral sasiedni klawisz albo gubil klikniecie.

#include "esphome/core/application.h"
#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/touchscreen/touchscreen.h"
#include "esphome/core/hal.h"

#include "gsl3680_firmware.h"
#include "gsl_point_id.h"

namespace esphome {
namespace gsl3680 {

#define TOUCH_MAX_POINTS 5

constexpr static const char *const TAG = "touchscreen.gsl3680";

class GSL3680 : public touchscreen::Touchscreen, public i2c::I2CDevice {
    public:
        void setup() override;
        void update_touches() override;

        void set_interrupt_pin(InternalGPIOPin *pin) { this->interrupt_pin_ = pin; }
        void set_prog_potwierdzenia(uint16_t v) { this->prog_potwierdzenia_ = v; }
        void set_prog_skoku(uint16_t v) { this->prog_skoku_ = v; }
        void set_diagnostyka(bool v) { this->diagnostyka_ = v; }
        void set_reset_pin(InternalGPIOPin *pin) { this->reset_pin_ = pin; }

    protected:
        // Filtr probek: przed zgloszeniem nowego dotkniecia czekamy na druga
        // probke w poblizu pierwszej, a w trakcie dotkniecia pomijamy
        // pojedyncze skoki. Zero wylacza dany filtr.
        void zglos_probke_(int16_t x, int16_t y, uint8_t palcow);
        uint16_t prog_potwierdzenia_{40};
        uint16_t prog_skoku_{150};
        bool diagnostyka_{false};
        uint32_t ost_ms_{0};
        bool dotyk_aktywny_{false};
        bool kandydat_{false};
        bool skok_{false};
        int16_t kand_x_{0}, kand_y_{0};
        int16_t ost_x_{0}, ost_y_{0};
        int16_t skok_x_{0}, skok_y_{0};

        InternalGPIOPin *interrupt_pin_{};
        InternalGPIOPin *reset_pin_{};
        // size_t width_ = 1280;
        // size_t height_ = 800;
        // esp_lcd_touch_handle_t tp_{};
        // esp_lcd_panel_io_handle_t tp_io_handle_{};

        esphome::i2c::ErrorCode reset();
        esphome::i2c::ErrorCode init();
        esphome::i2c::ErrorCode read_configuration();
        esphome::i2c::ErrorCode clear_registers();
        esphome::i2c::ErrorCode load_firmware();
        esphome::i2c::ErrorCode start();
        esphome::i2c::ErrorCode read_ram();
        void power_cycle();
};

}
}
