#include "mjpeg_lvgl.h"
#ifdef USE_ESP32

#include "esphome/core/log.h"
#include "esp_http_client.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

namespace esphome {
namespace mjpeg_lvgl {

static const char *const TAG = "mjpeg_lvgl";

void MjpegLvgl::setup() {
  // Bufor ramki trzymamy w PSRAM — w pamieci wewnetrznej nie ma na to miejsca.
  this->jpeg_buf_ = static_cast<uint8_t *>(
      heap_caps_malloc(this->buffer_size_, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (this->jpeg_buf_ == nullptr) {
    ESP_LOGE(TAG, "Brak pamieci na bufor ramki (%u B)", this->buffer_size_);
    this->mark_failed();
    return;
  }
  ESP_LOGCONFIG(TAG, "Bufor ramki: %u B w PSRAM", this->buffer_size_);
}

void MjpegLvgl::dump_config() {
  ESP_LOGCONFIG(TAG, "Strumien MJPEG:");
  ESP_LOGCONFIG(TAG, "  Adres: %s", this->url_.c_str());
  ESP_LOGCONFIG(TAG, "  Rozmiar: %ux%u", this->width_, this->height_);
  ESP_LOGCONFIG(TAG, "  Klatek na sekunde: %u", this->fps_);
}

void MjpegLvgl::start_stream() {
  if (this->biegnie_.load())
    return;
  this->biegnie_.store(true);
  this->ramek_.store(0);
  this->bledow_.store(0);
  // Wlasne zadanie: pobieranie nie moze blokowac glownej petli, bo to
  // wlasnie ono zawieszalo panel przy okladkach.
  xTaskCreatePinnedToCore(MjpegLvgl::task_trampoline, "mjpeg", 6144, this,
                          tskIDLE_PRIORITY + 2,
                          reinterpret_cast<TaskHandle_t *>(&this->task_handle_), 1);
}

void MjpegLvgl::stop_stream() { this->biegnie_.store(false); }

void MjpegLvgl::task_trampoline(void *arg) {
  static_cast<MjpegLvgl *>(arg)->task_loop();
  vTaskDelete(nullptr);
}

void MjpegLvgl::task_loop() {
  while (this->biegnie_.load()) {
    if (!this->czytaj_strumien())
      vTaskDelay(pdMS_TO_TICKS(1000));   // po bledzie odczekaj przed ponowieniem
  }
  this->task_handle_ = nullptr;
}

// Rozbior odpowiedzi multipart/x-mixed-replace: szukamy znacznikow SOI (FFD8)
// i EOI (FFD9), bo naglowki czesci roznia sie miedzy serwerami.
bool MjpegLvgl::czytaj_strumien() {
  esp_http_client_config_t cfg = {};
  cfg.url = this->url_.c_str();
  cfg.timeout_ms = 8000;
  cfg.buffer_size = 4096;
  esp_http_client_handle_t klient = esp_http_client_init(&cfg);
  if (klient == nullptr)
    return false;

  bool ok = false;
  if (esp_http_client_open(klient, 0) == ESP_OK) {
    esp_http_client_fetch_headers(klient);
    uint8_t kawalek[2048];
    uint32_t dl = 0;
    bool w_ramce = false;
    ok = true;
    while (this->biegnie_.load()) {
      int n = esp_http_client_read(klient, reinterpret_cast<char *>(kawalek), sizeof(kawalek));
      if (n <= 0)
        break;
      for (int i = 0; i < n; i++) {
        uint8_t b = kawalek[i];
        if (!w_ramce) {
          if (b == 0xD8 && i > 0 && kawalek[i - 1] == 0xFF) {
            w_ramce = true;
            dl = 0;
            this->jpeg_buf_[dl++] = 0xFF;
            this->jpeg_buf_[dl++] = 0xD8;
          }
          continue;
        }
        if (dl >= this->buffer_size_) {   // ramka wieksza niz bufor — odrzuc
          w_ramce = false;
          this->bledow_.fetch_add(1);
          continue;
        }
        this->jpeg_buf_[dl++] = b;
        if (b == 0xD9 && dl >= 2 && this->jpeg_buf_[dl - 2] == 0xFF) {
          w_ramce = false;
          this->ramek_.fetch_add(1);
          // TODO etap 2: sprzetowe dekodowanie jpeg_buf_[0..dl) do RGB565
        }
      }
    }
  }
  esp_http_client_close(klient);
  esp_http_client_cleanup(klient);
  return ok;
}

void MjpegLvgl::loop() {
  // Raport co 5 s, na razie tylko licznik ramek — dowod, ze rozbior dziala.
  const uint32_t teraz = millis();
  if (this->biegnie_.load() && teraz - this->ostatni_raport_ > 5000) {
    this->ostatni_raport_ = teraz;
    ESP_LOGI(TAG, "ramek: %u, odrzuconych: %u", this->ramek_.load(), this->bledow_.load());
  }
}

}  // namespace mjpeg_lvgl
}  // namespace esphome

#endif  // USE_ESP32
