#include "mjpeg_lvgl.h"
#include <algorithm>
#ifdef USE_ESP32

#include "esphome/core/log.h"
#include "esp_http_client.h"
#include "esp_heap_caps.h"
#include "esp_cache.h"
#include "esp_timer.h"
#include "rom/miniz.h"
#include "driver/jpeg_decode.h"
#include "driver/ppa.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>
#include <string>

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
  // Bufor odczytu z gniazda w pamieci wewnetrznej. Wczesniej byl tablica
  // lokalna w zadaniu i zabieral 2 kB z 6 kB stosu.
  this->kawalek_ = static_cast<uint8_t *>(heap_caps_malloc(2048, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
  if (this->kawalek_ == nullptr) {
    ESP_LOGE(TAG, "Brak pamieci na bufor odczytu");
    this->mark_failed();
    return;
  }
  ESP_LOGCONFIG(TAG, "Bufor ramki: %u B w PSRAM", this->buffer_size_);

  if (!this->przygotuj_dekoder()) {
    this->mark_failed();
    return;
  }

  this->opis_.header.magic = LV_IMAGE_HEADER_MAGIC;
  this->opis_.header.cf = LV_COLOR_FORMAT_RGB565;
  this->opis_.header.w = this->width_;
  this->opis_.header.h = this->height_;
  this->opis_.header.stride = this->width_ * 2;
  this->opis_.data_size = this->rgb_rozmiar_;
  this->opis_.data = this->rgb_[0];

  this->tryb_strumienia_ = !this->url_.empty();
  if (!this->tryb_strumienia_) {
    this->kolejka_ = xQueueCreate(1, sizeof(std::string *));
    this->biegnie_.store(true);
    TaskHandle_t uchwyt = nullptr;
    xTaskCreatePinnedToCore(MjpegLvgl::task_trampoline, "jpeg1", 12288, this,
                            tskIDLE_PRIORITY + 2, &uchwyt, 1);
    this->task_handle_.store(uchwyt);
    ESP_LOGCONFIG(TAG, "Tryb pojedynczych obrazow");
  }
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
  // Szerokosc wiersza bywa zaokraglana w gore do 4 pikseli (patrz dekoduj),
  // wiec bufor ma zapas na te trzy dodatkowe kolumny.
  const size_t potrzeba =
      static_cast<size_t>((this->width_ + 3u) & ~3u) * this->height_ * 2;
  for (int i = 0; i < 2; i++) {
    size_t przydzielono = 0;
    this->rgb_[i] = static_cast<uint8_t *>(jpeg_alloc_decoder_mem(potrzeba, &mem, &przydzielono));
    if (this->rgb_[i] == nullptr) {
      ESP_LOGE(TAG, "Brak pamieci na bufor obrazu %d (%u B)", i, (unsigned) potrzeba);
      return false;
    }
    this->rgb_rozmiar_ = przydzielono;
  }
  // Bufor na obraz w rozmiarze zrodlowym; z niego PPA skaluje do docelowego.
  size_t przydzielono = 0;
  this->dekod_buf_ = static_cast<uint8_t *>(jpeg_alloc_decoder_mem(potrzeba, &mem, &przydzielono));
  if (this->dekod_buf_ == nullptr) {
    ESP_LOGE(TAG, "Brak pamieci na bufor dekodowania");
    return false;
  }
  this->dekod_rozmiar_ = przydzielono;

  ppa_client_config_t ppa_cfg = {};
  ppa_cfg.oper_type = PPA_OPERATION_SRM;
  ppa_cfg.max_pending_trans_num = 1;
  if (ppa_register_client(&ppa_cfg, &this->ppa_) != ESP_OK) {
    ESP_LOGE(TAG, "Nie udalo sie zarejestrowac klienta PPA");
    return false;
  }

  ESP_LOGCONFIG(TAG, "Dekoder sprzetowy gotowy, 2 bufory po %u B + skalowanie PPA",
                (unsigned) this->rgb_rozmiar_);
  return true;
}


// ---------------------------------------------------------------------------
// Programowy dekoder PNG.
//
// Sprzetowy dekoder ESP32-P4 obsluguje wylacznie JPEG, a czesc stacji radiowych
// podaje okladki jako PNG. Rozpakowanie robi miniz z ROM-u ukladu (tinfl), wiec
// firmware nie rosnie o zadna biblioteke. Obslugujemy 8 bitow na kanal bez
// przeplotu — tak zapisana jest praktycznie kazda okladka.
// ---------------------------------------------------------------------------
static inline uint32_t png_be32(const uint8_t *p) {
  return ((uint32_t) p[0] << 24) | ((uint32_t) p[1] << 16) | ((uint32_t) p[2] << 8) | p[3];
}

// Rekonstrukcja bajtu wedlug filtra PNG (rozdzial 9 specyfikacji).
static inline uint8_t png_paeth(int a, int b, int c) {
  const int p = a + b - c;
  const int pa = abs(p - a), pb = abs(p - b), pc = abs(p - c);
  if (pa <= pb && pa <= pc)
    return (uint8_t) a;
  return pb <= pc ? (uint8_t) b : (uint8_t) c;
}

bool MjpegLvgl::dekoduj_png(uint32_t dlugosc) {
  const uint8_t *d = this->jpeg_buf_;
  if (dlugosc < 45 || png_be32(d + 12) != 0x49484452u /* IHDR */) {
    ESP_LOGW(TAG, "PNG bez naglowka IHDR (%u B)", (unsigned) dlugosc);
    this->bledow_.fetch_add(1);
    return false;
  }
  const uint32_t szer = png_be32(d + 16);
  const uint32_t wys = png_be32(d + 20);
  const uint8_t glebia = d[24], typ = d[25], przeplot = d[28];
  // Glebia 1/2/4 wystepuje w prostych logo stacji, 16 w skanach. Obslugujemy
  // wszystkie; przeplot Adam7 juz nie — to inny uklad danych i rzadkosc.
  if ((glebia != 1 && glebia != 2 && glebia != 4 && glebia != 8 && glebia != 16) ||
      przeplot != 0) {
    ESP_LOGW(TAG, "PNG %ux%u: glebia %u bit, przeplot %u — nieobslugiwane",
             (unsigned) szer, (unsigned) wys, glebia, przeplot);
    this->blad_formatu_ = true;
    this->bledow_.fetch_add(1);
    return false;
  }
  if (glebia < 8 && typ != 0 && typ != 3) {
    ESP_LOGW(TAG, "PNG: glebia %u bit dozwolona tylko dla szarosci i palety", glebia);
    this->blad_formatu_ = true;
    this->bledow_.fetch_add(1);
    return false;
  }
  int kanaly;
  switch (typ) {
    case 0: kanaly = 1; break;   // szarosc
    case 2: kanaly = 3; break;   // RGB
    case 3: kanaly = 1; break;   // paleta
    case 4: kanaly = 2; break;   // szarosc + alfa
    case 6: kanaly = 4; break;   // RGBA
    default:
      ESP_LOGW(TAG, "PNG: nieznany typ koloru %u", typ);
      this->blad_formatu_ = true;
      this->bledow_.fetch_add(1);
      return false;
  }
  if (szer == 0 || wys == 0 || szer > 4096 || wys > 4096) {
    ESP_LOGW(TAG, "PNG o niedorzecznym rozmiarze %ux%u", (unsigned) szer, (unsigned) wys);
    this->bledow_.fetch_add(1);
    return false;
  }

  // Przejscie po blokach: sklejamy IDAT i zapamietujemy palete.
  uint8_t paleta[256 * 3] = {};
  uint32_t poz = 8, dl_idat = 0;
  const uint8_t *idat_pocz = nullptr;
  bool ciagle = true;      // czy bloki IDAT leza obok siebie
  while (poz + 12 <= dlugosc) {
    const uint32_t dl = png_be32(d + poz);
    const uint32_t typ_bloku = png_be32(d + poz + 4);
    const uint8_t *dane = d + poz + 8;
    if (poz + 12 + (size_t) dl > dlugosc)
      break;
    if (typ_bloku == 0x504C5445u && dl <= sizeof(paleta)) {        // PLTE
      memcpy(paleta, dane, dl);
    } else if (typ_bloku == 0x49444154u) {                          // IDAT
      if (idat_pocz == nullptr)
        idat_pocz = dane;
      else if (idat_pocz + dl_idat != dane)
        ciagle = false;
      dl_idat += dl;
    } else if (typ_bloku == 0x49454E44u) {                          // IEND
      break;
    }
    poz += 12 + dl;
  }
  if (idat_pocz == nullptr || dl_idat == 0) {
    ESP_LOGW(TAG, "PNG bez danych obrazu");
    this->bledow_.fetch_add(1);
    return false;
  }

  // Bloki IDAT bywaja rozbite; tinfl chce jednego ciaglego wejscia. Sklejamy
  // je w miejscu, przesuwajac dane do przodu — bufor jest nasz, a oryginal
  // nie jest juz potrzebny.
  if (!ciagle) {
    uint8_t *cel = this->jpeg_buf_ + dlugosc;   // sklejamy ZA obrazem
    if (dlugosc + dl_idat > this->buffer_size_) {
      ESP_LOGW(TAG, "PNG: brak miejsca na sklejenie blokow IDAT");
      this->bledow_.fetch_add(1);
      return false;
    }
    uint32_t p2 = 8, zapisane = 0;
    while (p2 + 12 <= dlugosc) {
      const uint32_t dl = png_be32(d + p2);
      if (png_be32(d + p2 + 4) == 0x49444154u) {
        memcpy(cel + zapisane, d + p2 + 8, dl);
        zapisane += dl;
      }
      p2 += 12 + dl;
    }
    idat_pocz = cel;
  }

  // Rozpakowanie. Wynik to dla kazdego wiersza bajt filtra + piksele.
  // Przy glebi ponizej 8 bitow piksele sa upakowane, a filtr i tak dziala na
  // calych bajtach — stad osobno dlugosc wiersza i krok filtra.
  const size_t wiersz_b = ((size_t) szer * kanaly * glebia + 7) / 8;
  const int krok_filtra = (kanaly * glebia) / 8 > 0 ? (kanaly * glebia) / 8 : 1;
  const size_t surowy_b = (wiersz_b + 1) * wys;
  uint8_t *surowy = static_cast<uint8_t *>(
      heap_caps_malloc(surowy_b, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
  if (surowy == nullptr)
    surowy = static_cast<uint8_t *>(heap_caps_malloc(surowy_b, MALLOC_CAP_8BIT));
  if (surowy == nullptr) {
    ESP_LOGW(TAG, "PNG %ux%u: brak %u B na dane po rozpakowaniu",
             (unsigned) szer, (unsigned) wys, (unsigned) surowy_b);
    this->bledow_.fetch_add(1);
    return false;
  }
  const size_t wyszlo = tinfl_decompress_mem_to_mem(
      surowy, surowy_b, idat_pocz, dl_idat, TINFL_FLAG_PARSE_ZLIB_HEADER);
  if (wyszlo != surowy_b) {
    ESP_LOGW(TAG, "PNG: rozpakowanie dalo %u B zamiast %u",
             (unsigned) (wyszlo == TINFL_DECOMPRESS_MEM_TO_MEM_FAILED ? 0 : wyszlo),
             (unsigned) surowy_b);
    heap_caps_free(surowy);
    this->bledow_.fetch_add(1);
    return false;
  }

  // Odfiltrowanie w miejscu: kazdy wiersz odwoluje sie do sasiada z lewej
  // i do wiersza wyzej, wiec idziemy od gory i nadpisujemy dane rozpakowane.
  for (uint32_t y = 0; y < wys; y++) {
    uint8_t *w = surowy + (size_t) y * (wiersz_b + 1);
    const uint8_t filtr = w[0];
    uint8_t *biez = w + 1;
    const uint8_t *gora = (y == 0) ? nullptr : surowy + (size_t)(y - 1) * (wiersz_b + 1) + 1;
    for (size_t i = 0; i < wiersz_b; i++) {
      const int a = (i >= (size_t) krok_filtra) ? biez[i - krok_filtra] : 0;
      const int b = gora ? gora[i] : 0;
      const int c = (gora && i >= (size_t) krok_filtra) ? gora[i - krok_filtra] : 0;
      switch (filtr) {
        case 1: biez[i] = (uint8_t)(biez[i] + a); break;
        case 2: biez[i] = (uint8_t)(biez[i] + b); break;
        case 3: biez[i] = (uint8_t)(biez[i] + ((a + b) >> 1)); break;
        case 4: biez[i] = (uint8_t)(biez[i] + png_paeth(a, b, c)); break;
        default: break;   // 0 = bez filtra
      }
    }
  }

  // Zejscie z rozmiarem, gdy obraz nie miesci sie w buforze posrednim.
  // Bierzemy co n-ty piksel — reszte skalowania i tak zrobi PPA.
  uint32_t krok = 1;
  while (((size_t)((szer / krok + 15) & ~15u)) * (wys / krok) * 2 > this->dekod_rozmiar_)
    krok++;
  const uint32_t szer_c = szer / krok, wys_c = wys / krok;
  const uint32_t wiersz_px = (szer_c + 15u) & ~15u;   // PPA lubi rowne wiersze
  if (krok > 1)
    ESP_LOGI(TAG, "PNG %ux%u nie miesci sie w buforze — biore co %u piksel (%ux%u)",
             (unsigned) szer, (unsigned) wys, (unsigned) krok,
             (unsigned) szer_c, (unsigned) wys_c);

  // Konwersja na BGR565 (w takiej kolejnosci pracuje LVGL na tym ekranie).
  // Alfa nakladamy na czern — tlo kafla i tak jest ciemne.
  uint16_t *cel = reinterpret_cast<uint16_t *>(this->dekod_buf_);
  for (uint32_t y = 0; y < wys_c; y++) {
    const uint8_t *zr = surowy + (size_t)(y * krok) * (wiersz_b + 1) + 1;
    uint16_t *wy = cel + (size_t) y * wiersz_px;
    for (uint32_t x = 0; x < szer_c; x++) {
      const uint32_t xz = x * krok;
      uint8_t pr[4] = {0, 0, 0, 255};   // probki, juz rozwiniete do 8 bitow
      if (glebia == 8) {
        const uint8_t *p = zr + (size_t) xz * kanaly;
        for (int k = 0; k < kanaly; k++) pr[k] = p[k];
      } else if (glebia == 16) {
        const uint8_t *p = zr + (size_t) xz * kanaly * 2;
        for (int k = 0; k < kanaly; k++) pr[k] = p[k * 2];   // starszy bajt
      } else {
        // Upakowane probki: glebia 1, 2 lub 4 bity, zawsze jeden kanal.
        const uint32_t na_bajt = 8u / glebia;
        const uint8_t bajt = zr[xz / na_bajt];
        const uint32_t nr = xz % na_bajt;
        const uint32_t przes = 8u - glebia * (nr + 1);
        const uint32_t maska = (1u << glebia) - 1u;
        const uint32_t v = (bajt >> przes) & maska;
        // dla szarosci rozciagamy do pelnej skali, dla palety to numer koloru
        pr[0] = (typ == 3) ? (uint8_t) v : (uint8_t)(v * 255u / maska);
      }
      uint8_t r, g, b, alfa = 255;
      switch (typ) {
        case 0: r = g = b = pr[0]; break;
        case 4: r = g = b = pr[0]; alfa = pr[1]; break;
        case 2: r = pr[0]; g = pr[1]; b = pr[2]; break;
        case 6: r = pr[0]; g = pr[1]; b = pr[2]; alfa = pr[3]; break;
        default: {                       // paleta
          const uint8_t *k = paleta + (size_t) pr[0] * 3;
          r = k[0]; g = k[1]; b = k[2];
          break;
        }
      }
      if (alfa != 255) {
        r = (uint8_t)((r * alfa) / 255);
        g = (uint8_t)((g * alfa) / 255);
        b = (uint8_t)((b * alfa) / 255);
      }
      wy[x] = (uint16_t)(((b & 0xF8) << 8) | ((g & 0xFC) << 3) | (r >> 3));
    }
    for (uint32_t x = szer_c; x < wiersz_px; x++)
      wy[x] = 0;
  }
  heap_caps_free(surowy);

  // PPA czyta bufor przez DMA, a my pisalismy do niego procesorem — bez
  // zapisu cache'u do pamieci sterownik zobaczylby stare dane.
  const size_t uzyte = (size_t) wiersz_px * wys_c * 2;
  esp_cache_msync(this->dekod_buf_, uzyte,
                  ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
  return this->skaluj_i_odslon(szer_c, wys_c, wiersz_px, wys_c);
}

// Wolane z zadania strumienia — dekodowanie nie moze isc w glownej petli.
bool MjpegLvgl::dekoduj(uint32_t dlugosc) {
  // Sprzetowy dekoder P4 obsluguje wylacznie JPEG. Czesc stacji radiowych
  // podaje okladki jako PNG — bez tego sprawdzenia dekoder brnal przez dane
  // PNG, bral przypadkowe bajty za znaczniki i sypal w log bledami
  // "Truncated/invalid segment for marker 0xff69".
  if (dlugosc < 4 || this->jpeg_buf_[0] != 0xFF || this->jpeg_buf_[1] != 0xD8) {
    const uint8_t *b = this->jpeg_buf_;
    const char *format = "nieznany";
    if (dlugosc >= 8 && b[0] == 0x89 && b[1] == 'P' && b[2] == 'N' && b[3] == 'G')
      format = "PNG";
    else if (dlugosc >= 6 && b[0] == 'G' && b[1] == 'I' && b[2] == 'F')
      format = "GIF";
    else if (dlugosc >= 12 && b[8] == 'W' && b[9] == 'E' && b[10] == 'B' && b[11] == 'P')
      format = "WEBP";
    if (format[0] == 'P')                 // PNG ma wlasna, programowa sciezke
      return this->dekoduj_png(dlugosc);
    ESP_LOGW(TAG, "Obraz nie jest JPEG (%s, %u B, pierwsze bajty %02X %02X %02X %02X)",
             format, (unsigned) dlugosc, b[0], b[1], b[2], b[3]);
    this->blad_formatu_ = true;
    this->bledow_.fetch_add(1);
    return false;
  }
  // Okladki plyt przychodza w roznych rozmiarach, wiec wymiary czytamy
  // z naglowka kazdej ramki zamiast ufac konfiguracji.
  jpeg_decode_picture_info_t info = {};
  esp_err_t blad = jpeg_decoder_get_info(this->jpeg_buf_, dlugosc, &info);
  if (blad != ESP_OK) {
    // Bylo ciche. Okladki radia (145x145) nie pojawialy sie, a w logu nie bylo
    // po nich sladu — nie dalo sie odroznic nieudanego odczytu naglowka od
    // nieudanego dekodowania czy od tego, ze pobranie w ogole nie ruszylo.
    ESP_LOGW(TAG, "Odczyt naglowka JPEG nieudany (%u B): %s",
             (unsigned) dlugosc, esp_err_to_name(blad));
    this->bledow_.fetch_add(1);
    return false;
  }
  // Sterownik zapisuje obraz o wymiarach WYROWNANYCH do bloku MCU, nie o
  // rzeczywistych. Przy szerokosci niebedacej wielokrotnoscia bloku wiersze sa
  // dluzsze, niz wynika z naglowka — LVGL musi dostac te dluzsza wartosc jako
  // stride, inaczej obraz rozjezdza sie w pionowe pasy.
  uint16_t mcu_w = 8, mcu_h = 8;
  switch (info.sample_method) {
    case JPEG_DOWN_SAMPLING_YUV420: mcu_w = 16; mcu_h = 16; break;
    case JPEG_DOWN_SAMPLING_YUV422: mcu_w = 16; mcu_h = 8;  break;
    default: break;
  }
  const uint32_t szer_wyr = (info.width + mcu_w - 1) / mcu_w * mcu_w;
  const uint32_t wys_wyr = (info.height + mcu_h - 1) / mcu_h * mcu_h;

  if (static_cast<size_t>(szer_wyr) * wys_wyr * 2 > this->rgb_rozmiar_) {
    ESP_LOGW(TAG, "Obraz %ux%u (wyrownany %ux%u) nie miesci sie w buforze",
             (unsigned) info.width, (unsigned) info.height, (unsigned) szer_wyr, (unsigned) wys_wyr);
    this->bledow_.fetch_add(1);
    return false;
  }

  // Sprzetowy dekoder P4 odrzuca obrazy o wymiarach niepodzielnych przez 8:
  //   "Picture sizes not divisible by 8 are not supported" -> ESP_ERR_NOT_SUPPORTED.
  // Trafialy na to logo stacji TuneIn (145x145), podczas gdy okladki albumow
  // (512, 640) przechodzily. Obraz jest jednak ZAKODOWANY w pelnych blokach MCU,
  // czyli fizycznie jako 160x160 — koder dopelnia ostatni blok. Dane sa w
  // strumieniu, klamie tylko naglowek. Podnosimy wiec w naglowku SOF wymiary do
  // wielokrotnosci bloku, a przy skalowaniu PPA i tak wycinamy obszar o
  // rzeczywistych wymiarach (in.block_w/h), wiec dopelnienie nie trafia na ekran.
  if ((info.width % 8) != 0 || (info.height % 8) != 0) {
    size_t k = 2;
    bool zmieniono = false;
    while (k + 9 < dlugosc) {
      if (this->jpeg_buf_[k] != 0xFF) { k++; continue; }
      const uint8_t zn = this->jpeg_buf_[k + 1];
      if (zn == 0xC0 || zn == 0xC1 || zn == 0xC2) {
        this->jpeg_buf_[k + 5] = (uint8_t) (wys_wyr >> 8);
        this->jpeg_buf_[k + 6] = (uint8_t) (wys_wyr & 0xFF);
        this->jpeg_buf_[k + 7] = (uint8_t) (szer_wyr >> 8);
        this->jpeg_buf_[k + 8] = (uint8_t) (szer_wyr & 0xFF);
        zmieniono = true;
        break;
      }
      if (zn == 0xDA || zn == 0xD9) break;             // dalej sa juz dane
      if (zn == 0x01 || (zn >= 0xD0 && zn <= 0xD7)) { k += 2; continue; }
      k += 2 + ((this->jpeg_buf_[k + 2] << 8) | this->jpeg_buf_[k + 3]);
    }
    if (!zmieniono) {
      ESP_LOGW(TAG, "Nie znaleziono naglowka SOF do korekty rozmiaru %ux%u",
               (unsigned) info.width, (unsigned) info.height);
      this->bledow_.fetch_add(1);
      return false;
    }
    ESP_LOGI(TAG, "Rozmiar %ux%u niepodzielny przez 8 — dekoduje jako %ux%u, wycinam do oryginalu",
             (unsigned) info.width, (unsigned) info.height, (unsigned) szer_wyr, (unsigned) wys_wyr);
  }

  jpeg_decode_cfg_t cfg = {};
  cfg.output_format = JPEG_DECODE_OUT_FORMAT_RGB565;
  cfg.rgb_order = JPEG_DEC_RGB_ELEMENT_ORDER_BGR;   // LVGL pracuje na BGR565
  uint32_t wynik = 0;
  esp_err_t err = jpeg_decoder_process(this->dekoder_, &cfg, this->jpeg_buf_, dlugosc,
                                       this->dekod_buf_, this->dekod_rozmiar_, &wynik);
  if (err != ESP_OK) {
    ESP_LOGW(TAG, "Dekodowanie nieudane %ux%u (wyrownane %ux%u, MCU %ux%u): %s",
             (unsigned) info.width, (unsigned) info.height,
             (unsigned) szer_wyr, (unsigned) wys_wyr,
             (unsigned) mcu_w, (unsigned) mcu_h, esp_err_to_name(err));
    this->bledow_.fetch_add(1);
    return false;
  }

  return this->skaluj_i_odslon(info.width, info.height, szer_wyr, wys_wyr);
}

// Wspolne zakonczenie obu sciezek dekodowania: skalowanie sprzetowe do
// rozmiaru docelowego i odsloniecie bufora. `szer`/`wys` to rzeczywisty obraz,
// `wiersz_px`/`wys_zrodla` to wymiary bufora zrodlowego (moga byc wieksze,
// bo dekoder JPEG wyrownuje do bloku MCU, a sciezka PNG do 16 pikseli).
bool MjpegLvgl::skaluj_i_odslon(uint32_t szer, uint32_t wys, uint32_t wiersz_px,
                                uint32_t wys_zrodla) {
  // Skalowanie sprzetowe do rozmiaru docelowego. LVGL dostaje obraz gotowy,
  // dzieki czemu nie uruchamia swojego programowego przeksztalcenia — to ono
  // kosztowalo 200-400 ms na kazde narysowanie okladki.
  ppa_srm_oper_config_t srm = {};
  srm.in.buffer = this->dekod_buf_;
  srm.in.pic_w = wiersz_px;
  srm.in.pic_h = wys_zrodla;
  srm.in.block_w = szer;
  srm.in.block_h = wys;
  srm.in.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  srm.out.buffer = this->rgb_[this->wypelniany_];
  srm.out.buffer_size = this->rgb_rozmiar_;
  srm.out.pic_w = this->width_;
  srm.out.pic_h = this->height_;
  srm.out.srm_cm = PPA_SRM_COLOR_MODE_RGB565;
  srm.rotation_angle = PPA_SRM_ROTATION_ANGLE_0;
  // Jedna skala dla obu osi. Osobne scale_x i scale_y rozciagaly obraz do
  // ksztaltu bufora — kamera 4:3 wcisnieta w kafel 3:4 wygladala jak odbicie
  // w krzywym lustrze. Bierzemy mniejsza z dwoch skal, wiec caly obraz sie
  // miesci i zachowuje proporcje.
  const float skala = std::min(static_cast<float>(this->width_) / szer,
                               static_cast<float>(this->height_) / wys);
  srm.scale_x = skala;
  srm.scale_y = skala;
  srm.mode = PPA_TRANS_MODE_BLOCKING;

  // PPA kwantuje skale do krokow po 1/16, wiec faktyczny rozmiar wyniku
  // rzadko rowna sie dokladnie rozmiarowi bufora. Zamiast dopychac obraz do
  // ksztaltu bufora i czyscic reszte, robimy obraz DOKLADNIE takiej wielkosci,
  // jaka wyszla ze skalowania — LVGL dostaje wtedy wlasciwe proporcje i sam
  // wysrodkowuje go w kaflu. Nie ma marginesu, wiec nie ma tez czego czyscic.
  //
  // Poprzednia wersja robila memset calego bufora (614 kB) plus zapis cache'u
  // przy KAZDEJ klatce wideo — dwanascie razy na sekunde. To wystarczylo,
  // zeby panel zaliczyl niebieski ekran i restart od watchdoga.
  const uint32_t sx_i = (uint32_t) srm.scale_x;
  const uint32_t sx_f = (uint32_t) (srm.scale_x * 16) & 15u;
  const uint32_t sy_i = (uint32_t) srm.scale_y;
  const uint32_t sy_f = (uint32_t) (srm.scale_y * 16) & 15u;
  const uint32_t wy_w = sx_i * szer + sx_f * szer / 16;
  const uint32_t wy_h = sy_i * wys + sy_f * wys / 16;
  if (wy_w == 0 || wy_h == 0) {
    ESP_LOGW(TAG, "Skalowanie dalo pusty obraz (%ux%u)", (unsigned) wy_w, (unsigned) wy_h);
    this->bledow_.fetch_add(1);
    return false;
  }
  // Szerokosc wiersza zaokraglona w gore do 4 pikseli — PPA pisze przez DMA
  // i lubi rowne wiersze. LVGL i tak czyta wy_w pikseli, reszta to zapas.
  const uint32_t stride_px = (wy_w + 3u) & ~3u;
  srm.out.pic_w = stride_px;
  srm.out.pic_h = wy_h;
  srm.out.block_offset_x = 0;
  srm.out.block_offset_y = 0;
  esp_err_t blad_ppa = ppa_do_scale_rotate_mirror(this->ppa_, &srm);
  if (blad_ppa != ESP_OK) {
    ESP_LOGW(TAG, "Skalowanie PPA nieudane, skala %.3f, %s", (double) srm.scale_x, esp_err_to_name(blad_ppa));
    ESP_LOGW(TAG, "  (%ux%u -> %ux%u)", (unsigned) szer,
             (unsigned) wys, (unsigned) wy_w, (unsigned) wy_h);
    this->bledow_.fetch_add(1);
    return false;
  }

  // Wymiary zapisujemy obok bufora, nie w opisie dla LVGL. Opis wypelni
  // petla glowna, gdy odbierze te klatke — wtedy nikt z niego nie rysuje.
  MjpegLvgl::Ksztalt &k = this->ksztalt_[this->wypelniany_];
  k.szer = wy_w;
  k.wys = wy_h;
  k.wiersz_b = stride_px * 2;
  k.rozmiar = stride_px * wy_h * 2;
  if (this->zdekodowanych_.load() == 0 || wiersz_px != this->ost_szer_) {
    ESP_LOGI(TAG, "Obraz %ux%u -> %ux%u", (unsigned) szer, (unsigned) wys,
             (unsigned) wy_w, (unsigned) wy_h);
    this->ost_szer_ = wiersz_px;
  }
  // Odslon wypelniony bufor i przelacz sie na drugi.
  this->gotowy_.store(this->wypelniany_);
  this->wypelniany_ = 1 - this->wypelniany_;
  this->zdekodowanych_.fetch_add(1);
  return true;
}

// Tryb pojedynczych obrazow: adres trafia do kolejki o glebokosci 1.
// Nowsze zlecenie nadpisuje starsze — przy szybkiej zmianie utworow liczy
// sie ostatnia okladka, nie wszystkie po drodze.
void MjpegLvgl::pobierz(const std::string &url) {
  if (this->kolejka_ == nullptr || url.empty())
    return;
  auto *kopia = new std::string(url);
  std::string *stary = nullptr;
  if (xQueueReceive(this->kolejka_, &stary, 0) == pdTRUE)
    delete stary;
  if (xQueueSend(this->kolejka_, &kopia, 0) != pdTRUE)
    delete kopia;
}

bool MjpegLvgl::pobierz_jeden(const std::string &url) {
  esp_http_client_config_t cfg = {};
  cfg.url = url.c_str();
  cfg.timeout_ms = 8000;
  cfg.buffer_size = 4096;
  esp_http_client_handle_t klient = esp_http_client_init(&cfg);
  if (klient == nullptr)
    return false;

  bool ok = false;
  if (esp_http_client_open(klient, 0) == ESP_OK) {
    esp_http_client_fetch_headers(klient);
    if (esp_http_client_get_status_code(klient) == 200) {
      uint32_t dl = 0;
      while (true) {
        int n = esp_http_client_read(klient, reinterpret_cast<char *>(this->jpeg_buf_ + dl),
                                     this->buffer_size_ - dl);
        if (n <= 0)
          break;
        dl += n;
        if (dl >= this->buffer_size_) {
          ESP_LOGW(TAG, "Obraz wiekszy niz bufor (%u B)", this->buffer_size_);
          dl = 0;
          break;
        }
      }
      if (dl > 4) {
        this->ramek_.fetch_add(1);
        this->ostatnia_dl_.store(dl);
        ok = this->dekoduj(dl);
        // Rozmiar w logu pozwala odroznic dwa przypadki nieaktualnej okladki:
        // nieudane pobranie (widac ostrzezenie) od poprawnego pobrania starego
        // obrazka z posrednika Home Assistanta (ten sam rozmiar co poprzednio).
        ESP_LOGI(TAG, "Okladka pobrana: %u B, dekodowanie %s", (unsigned) dl, ok ? "ok" : "NIEUDANE");
      }
    } else {
      ESP_LOGW(TAG, "Obraz: HTTP %d", esp_http_client_get_status_code(klient));
    }
  } else {
    ESP_LOGW(TAG, "Obraz: polaczenie nieudane (%s)", url.c_str());
  }
  esp_http_client_close(klient);
  esp_http_client_cleanup(klient);
  return ok;
}

bool MjpegLvgl::nowa_klatka() {
  const int i = this->gotowy_.exchange(-1);
  if (i < 0)
    return false;
  // Jestesmy w petli glownej i LVGL w tej chwili nie rysuje — dopiero teraz
  // przepisujemy wymiary do opisu, ktory oglada widget.
  const MjpegLvgl::Ksztalt &k = this->ksztalt_[i];
  this->opis_.data = this->rgb_[i];
  this->opis_.header.w = k.szer;
  this->opis_.header.h = k.wys;
  this->opis_.header.stride = k.wiersz_b;
  this->opis_.data_size = k.rozmiar;
  return true;
}

void MjpegLvgl::dump_config() {
  ESP_LOGCONFIG(TAG, "Strumien MJPEG:");
  ESP_LOGCONFIG(TAG, "  Adres: %s", this->url_.c_str());
  ESP_LOGCONFIG(TAG, "  Rozmiar: %ux%u", this->width_, this->height_);
  ESP_LOGCONFIG(TAG, "  Klatek na sekunde: %u", this->fps_);
}

void MjpegLvgl::start_stream() {
  // Zabezpieczenie przed wywolaniem SPRZED setup(). Przywracany stan
  // przelacznika w ESPHome wykonuje sie w trakcie konfiguracji komponentow,
  // wiec potrafi trafic tu, zanim powstana bufory i kolejka. Zadanie
  // wchodzilo wtedy w galaz pojedynczych obrazow z pustym uchwytem kolejki,
  // xQueueReceive walil assertem i panel wpadal w petle restartow z czarnym
  // ekranem — bez mozliwosci wgrania czegokolwiek po sieci.
  if (this->rgb_[0] == nullptr || (!this->tryb_strumienia_ && this->kolejka_ == nullptr)) {
    ESP_LOGW(TAG, "start_stream przed konfiguracja komponentu — pomijam");
    return;
  }
  if (this->biegnie_.load())
    return;
  // Poprzednie zadanie moze jeszcze konczyc odczyt (np. tuz po przelaczeniu
  // kamery). Drugie zadanie pisaloby wtedy do tych samych buforow i obraz
  // rozpadalby sie na kawalki. Zamiast tworzyc je teraz, zostawiamy zlecenie
  // dla loop(), ktory wznowi strumien, gdy uchwyt zniknie.
  if (this->task_handle_.load() != nullptr) {
    if (this->url_oczekujacy_.empty())
      this->url_oczekujacy_ = this->url_;
    return;
  }
  this->biegnie_.store(true);
  this->ramek_.store(0);
  this->poprzednio_ = 0;   // bez tego (n - poprzednio_) przekreca sie na uint32
  this->bledow_.store(0);
  // Wlasne zadanie: pobieranie nie moze blokowac glownej petli, bo to
  // wlasnie ono zawieszalo panel przy okladkach.
  TaskHandle_t uchwyt = nullptr;
  xTaskCreatePinnedToCore(MjpegLvgl::task_trampoline, "mjpeg", 12288, this,
                          tskIDLE_PRIORITY + 2,
                          &uchwyt, 1);
  this->task_handle_.store(uchwyt);
}

void MjpegLvgl::stop_stream() {
  this->biegnie_.store(false);
  this->wstrzymane_.store(false);
}

void MjpegLvgl::przelacz_strumien(const std::string &url) {
  if (url.empty() || url == this->url_)
    return;   // ta sama kamera — nie przerywamy leciacego obrazu
  ESP_LOGI(TAG, "Przelaczam strumien na: %s", url.c_str());
  this->url_oczekujacy_ = url;
  this->biegnie_.store(false);   // zadanie wyjdzie po zakonczeniu biezacego odczytu
}

void MjpegLvgl::task_trampoline(void *arg) {
  static_cast<MjpegLvgl *>(arg)->task_loop();
  vTaskDelete(nullptr);
}

void MjpegLvgl::task_loop() {
  if (!this->tryb_strumienia_) {
    // Tryb pojedynczych obrazow: zadanie zyje caly czas i czeka na zlecenia.
    while (true) {
      std::string *url = nullptr;
      if (xQueueReceive(this->kolejka_, &url, portMAX_DELAY) == pdTRUE && url != nullptr) {
        // Nieudane pobranie zostawialo na ekranie okladke z poprzedniego utworu,
        // bo tytul i wykonawca ida wlasnym torem i aktualizuja sie mimo to.
        // Wynik nie moze byc odrzucany — probujemy ponownie z narastajaca przerwa.
        static const uint16_t przerwy[] = {400, 1200, 3000};
        bool ok = false;
        this->blad_formatu_ = false;
        for (int proba = 0; proba < 4 && !ok && !this->blad_formatu_; proba++) {
          if (proba > 0) {
            // Nowsze zlecenie uniewaznia ponawianie — nie nadpisujmy go stara okladka.
            if (uxQueueMessagesWaiting(this->kolejka_) > 0)
              break;
            vTaskDelay(pdMS_TO_TICKS(przerwy[proba - 1]));
          }
          ok = this->pobierz_jeden(*url);
        }
        if (ok) {
          this->ostatni_ok_ = *url;
        } else if (this->blad_formatu_) {
          ESP_LOGW(TAG, "Okladka w formacie nie do odczytania — pomijam: %s", url->c_str());
        } else {
          ESP_LOGW(TAG, "Nie udalo sie pobrac okladki po 4 probach: %s", url->c_str());
        }
        delete url;
      }
    }
  }
  while (this->biegnie_.load()) {
    if (!this->czytaj_strumien())
      vTaskDelay(pdMS_TO_TICKS(1000));   // po bledzie odczekaj przed ponowieniem
  }
  this->task_handle_.store(nullptr);
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

  uint8_t *const kawalek = this->kawalek_;   // bufor skladowy, nie na stosie
  uint32_t dl = 0;
  bool w_ramce = false;
  uint8_t poprzedni = 0;       // ostatni bajt z poprzedniej porcji: znacznik
                               // FFD8 potrafi wypasc na styku dwoch odczytow
  uint32_t bajtow = 0;
  while (this->biegnie_.load()) {
    int n = esp_http_client_read(klient, reinterpret_cast<char *>(kawalek), 2048);
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
          // Ograniczenie tempa dekodowania. Kamera potrafi przyslac 17 klatek
          // na sekunde, a panel przy pelnoekranowym obrazie rysuje ich 3-5 —
          // reszta byla dekodowana po to, zeby ja natychmiast nadpisac.
          // Klatki ponad limit odrzucamy zaraz po zlozeniu, bez dekodowania.
          const uint32_t teraz_ms = millis();
          const uint32_t odstep = this->fps_ > 0 ? 1000u / this->fps_ : 0u;
          if (!this->wstrzymane_.load() &&
              (odstep == 0 || teraz_ms - this->ost_dekod_ms_ >= odstep)) {
            this->ost_dekod_ms_ = teraz_ms;
            const int64_t t0 = esp_timer_get_time();
            this->dekoduj(dl);
            this->us_dekod_.fetch_add((uint32_t) (esp_timer_get_time() - t0));
          }
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
  // Wznowienie po przelaczeniu kamery. Czekamy, az stare zadanie sie zakonczy
  // (task_loop zeruje uchwyt na wyjsciu), zeby nie mialy dwa zadania naraz
  // dostepu do dekodera i buforow.
  if (!this->url_oczekujacy_.empty() && this->task_handle_.load() == nullptr) {
    this->url_ = this->url_oczekujacy_;
    this->url_oczekujacy_.clear();
    this->start_stream();
  }

  // Raport co 5 s, na razie tylko licznik ramek — dowod, ze rozbior dziala.
  const uint32_t teraz = millis();
  if (this->biegnie_.load() && teraz - this->ostatni_raport_ > 5000) {
    this->ostatni_raport_ = teraz;
    const uint32_t n = this->ramek_.load();
    this->fps_ost_ = (n - this->poprzednio_) / 5.0f;
    this->obc_ost_ = this->us_dekod_.exchange(0) / 50000.0f;   // % z 5 s
    ESP_LOGI(TAG, "ramek: %u (%.1f/s), ostatnia %u B, odrzuconych: %u, obciazenie %.0f%%", n,
             this->fps_ost_, this->ostatnia_dl_.load(), this->bledow_.load(),
             (double) this->obc_ost_);
    ESP_LOGI(TAG, "zdekodowanych: %u", this->zdekodowanych_.load());
    // Zapas stosu zadania dekodujacego. Stos ma 12288 B; przy 6144 B i tablicy
    // lokalnej 2048 B lancuch HTTP + dekoder + PPA podchodzil pod wartownika,
    // a przepelnienie objawialo sie niebieskim ekranem i restartem. Ta liczba
    // to najmniejszy zaobserwowany wolny zapas w bajtach — ma zostac wysoko.
    if (this->task_handle_.load() != nullptr) {
      const UBaseType_t zapas =
          uxTaskGetStackHighWaterMark(static_cast<TaskHandle_t>(this->task_handle_.load()));
      ESP_LOGI(TAG, "zapas stosu zadania: %u B", (unsigned) (zapas * sizeof(StackType_t)));
    }
    this->poprzednio_ = n;
  }
}

}  // namespace mjpeg_lvgl
}  // namespace esphome

#endif  // USE_ESP32
