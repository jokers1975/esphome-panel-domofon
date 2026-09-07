#pragma once
#ifdef USE_ESP32

#include "esphome/core/component.h"
#include "esphome/core/helpers.h"
#include <atomic>
#include <string>
#include "driver/jpeg_decode.h"
#include "driver/ppa.h"
#include "lvgl.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace esphome {
namespace mjpeg_lvgl {

// Pobieranie i sprzetowe dekodowanie JPEG poza glowna petla ESPHome.
// Dwa tryby, zaleznie od tego, czy w konfiguracji podano "url":
//   - strumien: ciagly MJPEG (kamera domofonu),
//   - pojedyncze obrazy: adres podawany w locie przez pobierz() (okladki plyt).
// Glowna petla dostaje wylacznie gotowy wskaznik — LVGL nie jest watkowo bezpieczne.
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
  // Przelaczenie na inna kamere bez tworzenia drugiej instancji. Zatrzymuje
  // biezace zadanie i zapamietuje nowy adres; wznowienie robi loop(), gdy
  // stare zadanie faktycznie sie zakonczy — inaczej dwa zadania pisalyby
  // do tych samych buforow.
  void przelacz_strumien(const std::string &url);
  // Wstrzymanie DEKODOWANIA bez rozlaczania strumienia. Polaczenie zostaje
  // otwarte, wiec go2rtc trzyma proces ffmpega przy zyciu i po powrocie na
  // ekran obraz jest natychmiast, zamiast po kilku sekundach rozruchu.
  // Klatki sa nadal odbierane i skladane, ale ida do kosza bez dekodowania.
  void wstrzymaj(bool tak) { this->wstrzymane_.store(tak); }
  bool wstrzymane() const { return this->wstrzymane_.load(); }

  // Tryb pojedynczych obrazow: zleca pobranie i zdekodowanie jednego JPEG.
  void pobierz(const std::string &url);

  // Wolane z glownej petli: czy czeka nowa zdekodowana klatka.
  bool nowa_klatka();
  // Adres ostatniego POMYSLNIE pobranego i zdekodowanego obrazu. Konfiguracja
  // porownuje z nim, zeby nieudane pobranie bylo ponawiane przy kolejnej okazji.
  std::string ostatni_udany() const { return this->ostatni_ok_; }
  // Pomiary z ostatniego okna 5 s — wystawiane do Home Assistanta, zeby dalo
  // sie zdalnie stwierdzic, czy panel nadaza za strumieniem. Gdy obciazenie
  // dobija do 100%, dekodowanie trwa dluzej niz odstep miedzy klatkami i
  // opoznienie obrazu rosnie samo z siebie.
  float klatek_na_s() const { return this->fps_ost_; }
  float obciazenie() const { return this->obc_ost_; }

  // Opis obrazu dla LVGL. Wskazuje na bufor, ktory wlasnie zostal odslonięty.
  const lv_image_dsc_t *opis_obrazu() const { return &this->opis_; }

 protected:
  static void task_trampoline(void *arg);
  void task_loop();
  bool czytaj_strumien();

  std::string url_;
  std::string url_oczekujacy_;
  uint16_t width_{0};
  uint16_t height_{0};
  uint8_t fps_{10};
  uint32_t buffer_size_{131072};

  bool przygotuj_dekoder();
  bool dekoduj(uint32_t dlugosc);
  // Programowa sciezka dla PNG — sprzetowy dekoder zna tylko JPEG.
  bool dekoduj_png(uint32_t dlugosc);
  bool skaluj_i_odslon(uint32_t szer, uint32_t wys, uint32_t wiersz_px, uint32_t wys_zrodla);
  bool pobierz_jeden(const std::string &url);

  uint8_t *jpeg_buf_{nullptr};       // surowa ramka JPEG (PSRAM)
  // Bufor odczytu z gniazda. Byl tablica lokalna w zadaniu — 2 kB z 6 kB stosu.
  uint8_t *kawalek_{nullptr};
  std::string ostatni_ok_;
  uint8_t *rgb_[2]{nullptr, nullptr};  // dwa bufory RGB565: rysowany i wypelniany
  size_t rgb_rozmiar_{0};
  std::atomic<int> gotowy_{-1};      // indeks bufora z kompletna klatka
  int wypelniany_{0};
  jpeg_decoder_handle_t dekoder_{nullptr};
  ppa_client_handle_t ppa_{nullptr};   // sprzetowe skalowanie do docelowego rozmiaru
  uint8_t *dekod_buf_{nullptr};        // obraz w rozmiarze zrodlowym, przed skalowaniem
  size_t dekod_rozmiar_{0};
  // Uklad ma JEDEN sprzetowy dekoder JPEG, a instancji komponentu jest kilka
  // (okladki i strumien z kamery). Bez wspolnej blokady wchodzily sobie w droge
  // i sterownik zwracal ESP_ERR_TIMEOUT — okladka radia nie pojawiala sie
  // dokladnie wtedy, gdy leciał obraz z domofonu.
  static SemaphoreHandle_t blokada_dekodera_;
  QueueHandle_t kolejka_{nullptr};   // adresy do pobrania w trybie pojedynczym
  bool tryb_strumienia_{false};
  // Opis, ktory oglada LVGL. Wskaznik na niego jest STALY — widget dostaje go
  // raz przez lv_image_set_src. Wypelnia go wylacznie petla glowna
  // (w nowa_klatka), nigdy zadanie dekodujace.
  lv_image_dsc_t opis_{};

  // Wymiary gotowej klatki, po jednym komplecie na bufor. Zadanie dekodujace
  // zapisuje je TU, a nie w opis_, bo tamten czyta w tym samym czasie petla
  // glowna. Wspolny opis byl bezpieczny dopoki obraz mial staly rozmiar; odkad
  // rozmiar zmienia sie przy przelaczeniu kamery, LVGL trafial na nowa
  // wysokosc przy starym wskazniku i czytal poza buforem: exception/panic.
  struct Ksztalt {
    uint32_t szer{0}, wys{0}, wiersz_b{0}, rozmiar{0};
  };
  Ksztalt ksztalt_[2];
  std::atomic<uint32_t> zdekodowanych_{0};
  std::atomic<uint32_t> us_dekod_{0};   // suma czasu dekodowania w oknie pomiaru
  // Obraz w formacie, ktorego sprzetowy dekoder nie zna (np. PNG). Ponawianie
  // pobrania nic tu nie da — plik bedzie taki sam.
  bool blad_formatu_{false};
  uint32_t ost_dekod_ms_{0};           // kiedy ostatnio dekodowalismy klatke
  float fps_ost_{0.0f};
  float obc_ost_{0.0f};
  uint32_t ost_szer_{0};   // ostatnia wyrownana szerokosc — do logu przy zmianie
  // Czytany takze z glownej petli (loop) w chwili, gdy zadanie sam sobie
  // zeruje uchwyt na wyjsciu — stad atomowy dostep zamiast zwyklego wskaznika.
  std::atomic<void *> task_handle_{nullptr};
  std::atomic<bool> biegnie_{false};
  std::atomic<bool> wstrzymane_{false};
  std::atomic<uint32_t> ramek_{0};   // licznik odebranych ramek
  std::atomic<uint32_t> bledow_{0};
  std::atomic<uint32_t> ostatnia_dl_{0};  // rozmiar ostatniej ramki
  uint32_t poprzednio_{0};                // licznik z poprzedniego raportu
  uint32_t ostatni_raport_{0};
};

}  // namespace mjpeg_lvgl
}  // namespace esphome

#endif  // USE_ESP32
