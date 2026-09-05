#include "gsl3680.h"

namespace esphome {
namespace gsl3680 {

#define STOP_ON_I2C_ERROR(code, value) code = value; if (code != esphome::i2c::ERROR_OK) { ESP_LOGE(TAG, "I2C Error: %d", code); return code; }

void GSL3680::setup() {

    ESP_LOGD(TAG, "Setup start");
    this->x_raw_max_ = this->swap_x_y_? this->get_display()->get_native_height(): this->get_display()->get_native_width() ;
    this->y_raw_max_ = this->swap_x_y_? this->get_display()->get_native_width() : this->get_display()->get_native_height();

    this->reset_pin_->pin_mode(esphome::gpio::FLAG_OUTPUT);
    this->reset_pin_->setup();

    this->reset_pin_->digital_write(false);
    this->reset_pin_->digital_write(true);

    auto err = this->init();
    if (err != esphome::i2c::ERROR_OK) {
        this->mark_failed(LOG_STR("I2C init error"));
        return;
    }

    this->interrupt_pin_->pin_mode(gpio::FLAG_INPUT | gpio::FLAG_PULLUP);
    this->interrupt_pin_->setup();
    this->attach_interrupt_(this->interrupt_pin_, gpio::INTERRUPT_FALLING_EDGE);

    gsl_DataInit(gsl_config_data_id);
    this->dotyk_aktywny_ = false;
    this->kandydat_ = false;
    this->skok_ = false;
    ESP_LOGI(TAG, "Setup complete");
}

esphome::i2c::ErrorCode GSL3680::init() {
    auto err = esphome::i2c::ERROR_OK;

    STOP_ON_I2C_ERROR(err, this->read_configuration());

    for (uint8_t attempt = 1; attempt <= 2; attempt++) {
        STOP_ON_I2C_ERROR(err, this->clear_registers());
        STOP_ON_I2C_ERROR(err, this->reset());
        STOP_ON_I2C_ERROR(err, this->load_firmware());
        STOP_ON_I2C_ERROR(err, this->start());

        err = this->read_ram();
        if (err == esphome::i2c::ERROR_OK) {
            return err;
        }

        if (attempt < 2) {
            ESP_LOGW(TAG, "Startup status check failed; power-cycling touchscreen and retrying firmware load");
            this->power_cycle();
        }
    }

    return err;
}

void GSL3680::power_cycle() {
    this->reset_pin_->digital_write(false);
    esphome::delay(50);
    this->reset_pin_->digital_write(true);
    esphome::delay(30);
    this->reset_pin_->digital_write(false);
    esphome::delay(5);
    this->reset_pin_->digital_write(true);
    esphome::delay(20);
    ESP_LOGD(TAG, "Power cycle complete");
}

esphome::i2c::ErrorCode GSL3680::reset() {
    this->reset_pin_->digital_write(false);
    esphome::delay(20);
    this->reset_pin_->digital_write(true);
    esphome::delay(20);

    auto err = esphome::i2c::ERROR_OK;
    uint8_t write_buf[4];

    write_buf[0] = 0x88;
    STOP_ON_I2C_ERROR(err, this->write_register(0xe0, (uint8_t *)&write_buf, 1));
    esphome::delay(10);
    write_buf[0] = 0x04;
    STOP_ON_I2C_ERROR(err, this->write_register(0xe4, (uint8_t *)&write_buf, 1));
    esphome::delay(10);

    write_buf[0] = 0x00;
    write_buf[1] = 0x00;
    write_buf[2] = 0x00;
    write_buf[3] = 0x00;
    STOP_ON_I2C_ERROR(err, this->write_register(0xbc, (uint8_t *)&write_buf, 4));
    esphome::delay(10);

    ESP_LOGD(TAG, "Reset complete");
    return err;

}

esphome::i2c::ErrorCode GSL3680::read_configuration() {
    auto err = esphome::i2c::ERROR_OK;

    uint8_t buf[4];
    uint8_t write[4] = {0x12, 0x34, 0x56, 0x00};

    esphome::delay(50);
    STOP_ON_I2C_ERROR(err, this->read_register(0xf0, (uint8_t *)&buf, 4));
    ESP_LOGD(TAG, "Read configuration #1: %x, %x, %x, %x", buf[0], buf[1], buf[2], buf[3]);
    esphome::delay(20);
    STOP_ON_I2C_ERROR(err, this->write_register(0xf0, (uint8_t *)&write, 4));
    esphome::delay(20);
    STOP_ON_I2C_ERROR(err, this->read_register(0xf0, (uint8_t *)&buf, 4));
    esphome::delay(20);
    ESP_LOGD(TAG, "Read configuration #2: %x, %x, %x, %x", buf[0], buf[1], buf[2], buf[3]);
    if (buf[0] != write[0]) {
        ESP_LOGE(TAG, "Invalid configuration byte returned, got 0x%x, expected 0x12", buf[0]);
        return esphome::i2c::ERROR_UNKNOWN;
    }
    return err;
}

esphome::i2c::ErrorCode GSL3680::clear_registers() {
    uint8_t clear_reg_regs[4] = {0xe0, 0x80, 0xe4, 0xe0};
    uint8_t clear_reg_data[4] = {0x88, TOUCH_MAX_POINTS, 0x04, 0x00};

    auto err = esphome::i2c::ERROR_OK;

    for (int i = 0; i < 4; i++) {
        STOP_ON_I2C_ERROR(err, this->write_register(clear_reg_regs[i], (uint8_t *)&clear_reg_data[i], 1));
        esphome::delay(20);
    }
    ESP_LOGD(TAG, "Clear registers complete");
    return err;

}

esphome::i2c::ErrorCode GSL3680::load_firmware() {
    ESP_LOGD(TAG,"Load firmware start");
    uint8_t addr;
    uint8_t wrbuf[4];
    uint16_t source_len = sizeof(GSLX680_FW) / sizeof(struct fw_data);
    auto err = esphome::i2c::ERROR_OK;

    for(int i = 0; i < source_len; i++) {
        addr = GSLX680_FW[i].offset;
        wrbuf[0] = (uint8_t)(GSLX680_FW[i].val & 0x000000ff);
        wrbuf[1] = (uint8_t)((GSLX680_FW[i].val & 0x0000ff00) >> 8);
        wrbuf[2] = (uint8_t)((GSLX680_FW[i].val & 0x00ff0000) >> 16);
        wrbuf[3] = (uint8_t)((GSLX680_FW[i].val & 0xff000000) >> 24);
        STOP_ON_I2C_ERROR(err, this->write_register(addr, (uint8_t *)&wrbuf, addr == 0xf0? 1: 4));
    }
    ESP_LOGD(TAG,"Load firmware complete");
    return err;
}

esphome::i2c::ErrorCode GSL3680::start() {
    uint8_t write_buf[1] = {0x00};
    uint8_t addr = 0xe0;

    auto err = esphome::i2c::ERROR_OK;
    STOP_ON_I2C_ERROR(err, this->write_register(addr, (uint8_t *)&write_buf, 1));
    esphome::delay(10);
    ESP_LOGD(TAG,"Start chipset complete");

    return err;

}

esphome::i2c::ErrorCode GSL3680::read_ram() {
    uint8_t buf[4];

    auto err = esphome::i2c::ERROR_OK;
    esphome::delay(30);
    STOP_ON_I2C_ERROR(err, this->read_register(0xb0, (uint8_t *)&buf, 4));
    ESP_LOGD(TAG, "Read RAM: %x, %x, %x, %x", buf[0], buf[1], buf[2], buf[3]);
    for (int i = 0; i < 4; i++) {
        if (buf[i] != 0x5a) {
            ESP_LOGE(TAG, "Unexpected byte in read_ram: got 0x%x, expected 0x5a", buf[i]);
            return esphome::i2c::ERROR_UNKNOWN;
        }
    }
    return err;
}


void GSL3680::update_touches() {
    uint8_t touch_data[24];
    struct gsl_touch_info cinfo = {0};

    auto err = this->read_register(0x80, (uint8_t *)&touch_data, 24);
    if (err != esphome::i2c::ERROR_OK) {
        ESP_LOGE(TAG, "I2C error in update_touches");
        return;
    }
    ESP_LOGV(TAG, "update_touches: %x %x %x %x %x %x %x %x", touch_data[0], touch_data[1], touch_data[2], touch_data[3], touch_data[4], touch_data[5], touch_data[6], touch_data[7]);

    uint8_t touch_count = touch_data[0] & 0x0f;
    if (touch_count > 2) {
        ESP_LOGV(TAG, "GSL3680 reported %u touches; clamping to the two points read by this driver", touch_count);
        touch_count = 2;
    }
    uint16_t x1 = ((touch_data[7] & 0x0f) << 8) | touch_data[6];
	uint16_t y1 = (touch_data[5] << 8) | touch_data[4];
    uint16_t x2 = ((touch_data[11] & 0x0f) << 8) | touch_data[10];
	uint16_t y2 = (touch_data[9] << 8) | touch_data[8];

    cinfo.x[0] = x1;
    cinfo.y[0] = y1;
    cinfo.id[0] = ((touch_data[7] & 0xf0) >> 4);
    cinfo.x[1] = x2;
    cinfo.y[1] = y2;
    cinfo.id[1] = ((touch_data[11] & 0xf0) >> 4);
    cinfo.finger_num = touch_count;

    gsl_alg_id_main(&cinfo);
    unsigned int mask = gsl_mask_tiaoping();

    if ((mask > 0) && (mask < 0xffffffff)) {
        uint8_t buf[4] = {0xa, 0x0, 0x0, 0x0};
        this->write_register(0xf0, (uint8_t *)&buf, 4);
        buf[0] = (uint8_t)(mask & 0xff);
        buf[1] = (uint8_t)((mask >> 8) & 0xff);
        buf[2] = (uint8_t)((mask >> 16) & 0xff);
        buf[3] = (uint8_t)((mask >> 24) & 0xff);
        this->write_register(0x8, (uint8_t *)&buf, 4);
    }

    ESP_LOGV(TAG, "update_touches: touch [%d] %dx%d (%d)", cinfo.finger_num, cinfo.x[0], cinfo.y[0], mask);

    // Report only first finger for now
    this->zglos_probke_(cinfo.x[0], cinfo.y[0], cinfo.finger_num);
}

// Kontroler GSL od czasu do czasu zglasza pojedyncza probke daleko od miejsca,
// w ktorym faktycznie lezy palec. LVGL wybiera klawisz w chwili wcisniecia, wiec
// bledna pierwsza probka to wpisana nie ta litera; bledna probka w trakcie
// dotkniecia anuluje klikniecie ("dotyk nie zadzialal"). Stad dwa filtry:
//  - nowe dotkniecie zglaszamy dopiero, gdy druga probka potwierdzi pierwsza,
//  - w trakcie dotkniecia pojedynczy skok pomijamy, a ruch przyjmujemy dopiero
//    gdy potwierdzi go kolejna probka (przeciaganie traci najwyzej jedna probke).
void GSL3680::zglos_probke_(int16_t x, int16_t y, uint8_t palcow) {
    uint32_t teraz = millis();
    uint32_t przerwa = teraz - this->ost_ms_;
    this->ost_ms_ = teraz;

    if (this->diagnostyka_) {
        ESP_LOGI(TAG, "probka palcow=%u %dx%d (przerwa %u ms)", palcow, x, y, przerwa);
    }

    // Zabezpieczenie: jesli kontroler nie zglosil podniesienia palca (brak
    // przerwania i brak odpytania), stan potrafil przetrwac sekundy i kolejne
    // dotkniecie bylo brane za ciag dalszy poprzedniego. Dluga przerwa zawsze
    // zaczyna nowe dotkniecie.
    if (przerwa > 300) {
        this->dotyk_aktywny_ = false;
        this->kandydat_ = false;
        this->skok_ = false;
    }

    if (palcow != 1) {
        this->dotyk_aktywny_ = false;
        this->kandydat_ = false;
        this->skok_ = false;
        return;
    }

    if (this->prog_potwierdzenia_ == 0 && this->prog_skoku_ == 0) {
        this->add_raw_touch_position_(0, x, y);
        return;
    }

    if (!this->dotyk_aktywny_) {
        if (this->prog_potwierdzenia_ > 0) {
            if (!this->kandydat_) {
                this->kandydat_ = true;
                this->kand_x_ = x;
                this->kand_y_ = y;
                // Bez zgloszenia dotkniecia baza nie ustawi swojego timeoutu,
                // wiec sami wymuszamy ponowny odczyt — inaczej nieruchomy palec
                // moglby czekac na przerwanie, ktore juz nie przyjdzie.
                this->set_timeout("gsl_potw", 20, [this]() { this->store_.touched = true; });
                return;
            }
            if (abs(x - this->kand_x_) > this->prog_potwierdzenia_ ||
                abs(y - this->kand_y_) > this->prog_potwierdzenia_) {
                ESP_LOGI(TAG, "odrzucona probka poczatkowa %dx%d (kandydat %dx%d)", x, y, this->kand_x_,
                         this->kand_y_);
                this->kand_x_ = x;
                this->kand_y_ = y;
                this->set_timeout("gsl_potw", 20, [this]() { this->store_.touched = true; });
                return;
            }
        }
        this->cancel_timeout("gsl_potw");
        this->kandydat_ = false;
        this->dotyk_aktywny_ = true;
        this->skok_ = false;
        this->ost_x_ = x;
        this->ost_y_ = y;
        this->add_raw_touch_position_(0, this->ost_x_, this->ost_y_);
        return;
    }

    if (this->prog_skoku_ > 0 &&
        (abs(x - this->ost_x_) > this->prog_skoku_ || abs(y - this->ost_y_) > this->prog_skoku_)) {
        if (!this->skok_ || abs(x - this->skok_x_) > this->prog_skoku_ ||
            abs(y - this->skok_y_) > this->prog_skoku_) {
            ESP_LOGI(TAG, "pominieto skok %dx%d (ostatnio %dx%d)", x, y, this->ost_x_, this->ost_y_);
            this->skok_ = true;
            this->skok_x_ = x;
            this->skok_y_ = y;
            // Podtrzymujemy dotkniecie w poprzednim miejscu, zeby LVGL nie uznal
            // go za zwolnione i nie anulowal klikniecia.
            this->add_raw_touch_position_(0, this->ost_x_, this->ost_y_);
            return;
        }
    }

    this->skok_ = false;
    this->ost_x_ = x;
    this->ost_y_ = y;
    this->add_raw_touch_position_(0, this->ost_x_, this->ost_y_);
}

}
}
