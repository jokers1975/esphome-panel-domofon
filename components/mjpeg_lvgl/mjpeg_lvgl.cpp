#include "mjpeg_lvgl.h"
#ifdef USE_ESP32

#include "esphome/core/log.h"
#include "esp_http_client.h"
#include "esp_heap_caps.h"
#include "driver/jpeg_decode.h"
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

  if (!this->przygotuj_dekoder()) {
    this->mark_failed();
    return;
  }

  // Opis obrazu dla LVGL wypelniamy raz — potem zmienia sie tylko wskaznik
  // na dane, gdy odslaniamy swiezo zdekodowana klatke.
  this->opis_.header.magic = LV_IMAGE_HEADER_MAGIC;
  this->opis_.header.cf = LV_COLOR_FORMAT_RGB565;
  this->opis_.header.w = this->width_;
  this->opis_.header.h = this->height_;
  this->opis_.header.stride = this->width_ * 2;
  this->opis_.data_size = this->rgb_rozmiar_;
  this->opis_.data = this->rgb_[0];
}

bool MjpegLvgl::przygotuj_dekoder() {
  jpeg_decode_engine_cfg_t cfg = {};
  cfg.intr_priority = 0;
  cfg.timeout_ms = 120;     // przy 15 kl/s na ramke jest ok. 66 ms
  esp_err_t err = jpeg_new_decoder_engine(&cfg, &this->dekoder_);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Nie udalo sie uruchomic sprzetowego dekodera JPEG: %s", esp_err_to_name(err));
    return false;
  }

  // Bufory wyjsciowe musi przydzielic sterownik — wymaga wyrownania pod DMA.
  jpeg_decode_memory_alloc_cfg_t mem = {};
  mem.buffer_direction = JPEG_DEC_ALLOC_OUTPUT_BUFFER;
  const size_t potrzeba = static_cast<size_t>(this->width_) * this->height_ * 2;
  for (int i = 0; i < 2; i++) {
    size_t przydzielono = 0;
    this->rgb_[i] = static_cast<uint8_t *>(jpeg_alloc_decoder_mem(potrzeba, &mem, &przydzielono));
    if (this->rgb_[i] == nullptr) {
      ESP_LOGE(TAG, "Brak pamieci na bufor obrazu %d (%u B)", i, (unsigned) potrzeba);
      return false;
    }
    this->rgb_rozmiar_ = przydzielono;
  }
  ESP_LOGCONFIG(TAG, "Dekoder sprzetowy gotowy, 2 bufory po %u B", (unsigned) this->rgb_rozmiar_);
  return true;
}

// Wolane z zadania strumienia — dekodowanie nie moze isc w glownej petli.
void MjpegLvgl::dekoduj(uint32_t dlugosc) {
  jpeg_decode_cfg_t cfg = {};
  cfg.output_format = JPEG_DECODE_OUT_FORMAT_RGB565;
  cfg.rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR;   // LVGL pracuje na BGR565
  uint32_t wynik = 0;
  esp_err_t err = jpeg_decoder_process(this->dekoder_, &cfg, this->jpeg_buf_, dlugosc,
                                       this->rgb_[this->wypelniany_], this->rgb_rozmiar_, &wynik);
  if (err != ESP_OK) {
    this->bledow_.fetch_add(1);
    return;
  }
  // Odslon wypelniony bufor i przelacz sie na drugi.
  this->gotowy_.store(this->wypelniany_);
  this->wypelniany_ = 1 - this->wypelniany_;
  this->zdekodowanych_.fetch_add(1);
}

bool MjpegLvgl::nowa_klatka() {
  const int i = this->gotowy_.exchange(-1);
  if (i < 0)
    return false;
  this->opis_.data = this->rgb_[i];
  return true;
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
  if (klient == nullptr) {
    ESP_LOGE(TAG, "Nie udalo sie utworzyc klienta HTTP");
    return false;
  }

  esp_err_t err = esp_http_client_open(klient, 0);
  if (err != ESP_OK) {
    // Brak tego logu kosztowal jedna runde diagnostyki: komponent milczal,
    // a licznik klatek stal na zerze bez zadnej wskazowki dlaczego.
    ESP_LOGW(TAG, "Polaczenie nieudane: %s (%s)", esp_err_to_name(err), this->url_.c_str());
    esp_http_client_cleanup(klient);
    return false;
  }

  const int dl_naglowkow = esp_http_client_fetch_headers(klient);
  const int status = esp_http_client_get_status_code(klient);
  ESP_LOGI(TAG, "Polaczono, HTTP %d, dlugosc %d", status, dl_naglowkow);
  if (status != 200) {
    ESP_LOGW(TAG, "Serwer odpowiedzial %d — przerywam", status);
    esp_http_client_close(klient);
    esp_http_client_cleanup(klient);
    return false;
  }

  uint8_t kawalek[2048];
  uint32_t dl = 0;
  bool w_ramce = false;
  uint8_t poprzedni = 0;       // ostatni bajt z poprzedniej porcji: znacznik
                               // FFD8 potrafi wypasc na styku dwoch odczytow
  uint32_t bajtow = 0;
  while (this->biegnie_.load()) {
    int n = esp_http_client_read(klient, reinterpret_cast<char *>(kawalek), sizeof(kawalek));
    if (n <= 0) {
      ESP_LOGW(TAG, "Strumien przerwany po %u B", bajtow);
      break;
    }
    bajtow += n;
    for (int i = 0; i < n; i++) {
      const uint8_t b = kawalek[i];
      if (!w_ramce) {
        if (b == 0xD8 && poprzedni == 0xFF) {
          w_ramce = true;
          dl = 0;
          this->jpeg_buf_[dl++] = 0xFF;
          this->jpeg_buf_[dl++] = 0xD8;
        }
      } else if (dl >= this->buffer_size_) {
        w_ramce = false;                       // ramka nie miesci sie w buforze
        this->bledow_.fetch_add(1);
      } else {
        this->jpeg_buf_[dl++] = b;
        if (b == 0xD9 && poprzedni == 0xFF) {
          w_ramce = false;
          this->ostatnia_dl_.store(dl);
          this->ramek_.fetch_add(1);
          this->dekoduj(dl);
        }
      }
      poprzedni = b;
    }
  }
  esp_http_client_close(klient);
  esp_http_client_cleanup(klient);
  return true;
}

void MjpegLvgl::loop() {
  // Raport co 5 s, na razie tylko licznik ramek — dowod, ze rozbior dziala.
  const uint32_t teraz = millis();
  if (this->biegnie_.load() && teraz - this->ostatni_raport_ > 5000) {
    this->ostatni_raport_ = teraz;
    const uint32_t n = this->ramek_.load();
    ESP_LOGI(TAG, "ramek: %u (%.1f/s), ostatnia %u B, odrzuconych: %u", n,
             (n - this->poprzednio_) / 5.0f, this->ostatnia_dl_.load(), this->bledow_.load());
    ESP_LOGI(TAG, "zdekodowanych: %u", this->zdekodowanych_.load());
    this->poprzednio_ = n;
  }
}

}  // namespace mjpeg_lvgl
}  // namespace esphome

#endif  // USE_ESP32
