# Komponenty ESPHome — panel domofonu

Własne komponenty dla panelu Guition JC8012P4A1C (ESP32-P4) w Home Assistant.

## `mjpeg_lvgl`

Odbiór strumienia MJPEG we własnym zadaniu FreeRTOS, poza główną pętlą ESPHome.
Docelowo: sprzętowe dekodowanie JPEG (kodek ESP32-P4) i rysowanie w LVGL bez
blokowania interfejsu.

Etap 1 (obecny): pobieranie strumienia i rozbiór klatek, licznik w logu.

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/jokers1975/esphome-panel-domofon
      ref: main
    components: [mjpeg_lvgl]
    username: !secret github_user
    password: !secret github_token

mjpeg_lvgl:
  id: strumien_domofonu
  url: "http://192.168.3.2:1984/api/stream.mjpeg?src=domofon"
  width: 480
  height: 640
  fps: 10
```

Wymaga ESP32-P4 — komponent odmawia konfiguracji na innych układach.
