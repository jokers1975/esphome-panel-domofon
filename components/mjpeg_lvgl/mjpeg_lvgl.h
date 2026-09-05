#pragma once
#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include <atomic>
#include <string>
#include "driver/jpeg_decode.h"
#include "lvgl.h"

namespace esphome {
namespace mjpeg_lvgl {

// Etap 1: pobieranie strumienia MJPEG we wlasnym zadaniu i rozbior ramek.
// Dekodowanie sprzetowe i przekazanie do LVGL dochodza w kolejnych etapach.
class MjpegLvgl : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::LATE; }

  void set_url(const std::string &url) { this->url_ = url; }
  void set_size(uint16_t w, uint16_t h) { this->width_ = w; this->height_ = h; }
  void set_fps(uint8_t fps) { this->fps_ = fps; }
  void set_buffer_size(uint32_t n) { this->buffer_size_ = n; }

  void start_stream();
  void stop_stream();

  // Wolane z glownej petli: czy czeka nowa zdekodowana klatka.
  bool nowa_klatka();
  // Opis obrazu dla LVGL. Wskazuje na bufor, ktory wlasnie zostal odslonięty.
  const lv_image_dsc_t *opis_obrazu() const { return &this->opis_; }

 protected:
  static void task_trampoline(void *arg);
  void task_loop();
  bool czytaj_strumien();

  std::string url_;
  uint16_t width_{0};
  uint16_t height_{0};
  uint8_t fps_{10};
  uint32_t buffer_size_{131072};

  bool przygotuj_dekoder();
  void dekoduj(uint32_t dlugosc);

  uint8_t *jpeg_buf_{nullptr};       // surowa ramka JPEG (PSRAM)
  uint8_t *rgb_[2]{nullptr, nullptr};  // dwa bufory RGB565: rysowany i wypelniany
  size_t rgb_rozmiar_{0};
  std::atomic<int> gotowy_{-1};      // indeks bufora z kompletna klatka
  int wypelniany_{0};
  jpeg_decoder_handle_t dekoder_{nullptr};
  lv_image_dsc_t opis_{};
  std::atomic<uint32_t> zdekodowanych_{0};
  void *task_handle_{nullptr};
  std::atomic<bool> biegnie_{false};
  std::atomic<uint32_t> ramek_{0};   // licznik odebranych ramek
  std::atomic<uint32_t> bledow_{0};
  std::atomic<uint32_t> ostatnia_dl_{0};  // rozmiar ostatniej ramki
  uint32_t poprzednio_{0};                // licznik z poprzedniego raportu
  uint32_t ostatni_raport_{0};
};

}  // namespace mjpeg_lvgl
}  // namespace esphome

#endif  // USE_ESP32
