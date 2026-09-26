# Firmware perangkat IoTMaxhub (ESP32)

Sketch Arduino untuk perangkat ATM security: DHT22 (suhu/kelembapan),
ADXL345 (vibrasi), PZEM-004T v3.0 (listrik), layar ST7789 240x240 untuk QR
claim, dua relay, dan dua profil WiFi dengan AP recovery.

| Folder | Versi | Status |
|---|---|---|
| [`iotmaxhub_atm_apmode_v4.5/`](iotmaxhub_atm_apmode_v4.5/) | `5.1` | **Terpasang di perangkat sekarang.** HTTP penuh: telemetri POST tiap 5 detik, relay di-poll tiap 2 detik. |
| [`iotmaxhub-atm-mqtt/`](iotmaxhub-atm-mqtt/) | `5.2-mqtt` | **Belum di-flash.** Turunan 5.1 + MQTT-over-WSS. |

Nama folder harus sama dengan nama `.ino` di dalamnya — itu aturan Arduino IDE.

## Yang berubah di 5.2-mqtt

`mqtt.ino` menambahkan jalur realtime lewat `wss://mqtt.iotmaxhub.com/mqtt`
(esp-mqtt, sertifikat diverifikasi dengan `esp_crt_bundle_attach`):

| | 5.1 | 5.2-mqtt |
|---|---|---|
| Telemetri | HTTP tiap 5 detik | MQTT tiap 5 detik; HTTP tiap 15 menit sebagai cadangan sekaligus pemeriksa API key |
| Relay | poll HTTP tiap 2 detik | seketika lewat MQTT; poll HTTP tiap 30 detik sebagai cadangan |
| Alert | HTTP seketika | HTTP seketika (tidak berubah) |
| Offline | - | Last Will `iot/<MAC>/status` = `offline` |

`MQTT_URL` dikosongkan = MQTT mati, perangkat kembali berperilaku seperti 5.1.
Detail topik, login, dan ACL ada di
[`../backend/docs/MQTT.md`](../backend/docs/MQTT.md).

5.2-mqtt memakai 98% partisi bawaan (default 4 MB with spiffs). Masih muat dan
OTA tetap jalan, tapi ruangnya tinggal sedikit — kalau nanti perlu tambahan
besar, pindah ke skema partisi Minimal SPIFFS.

## Sebelum compile: isi secrets.h

Kunci bootstrap dan sandi AP **tidak ikut git**. Di tiap folder sketch:

```bash
cp secrets.h.contoh secrets.h
```

lalu isi ketiga nilainya. `secrets.h` sudah masuk `.gitignore` — jangan pernah
menaruh nilai aslinya kembali ke dalam `.ino`.

## Compile

Arduino IDE 2.x, board **ESP32 Dev Module** (paket `esp32` by Espressif).
Library yang dibutuhkan: `Adafruit ADXL345`, `Adafruit GFX`, `Adafruit ST7789`,
`ArduinoJson`, `DHT sensor library`, `PZEM004Tv30`.

**Setiap kali mengubah firmware, naikkan `FIRMWARE_VER`** dan isi versi yang
sama di halaman admin `/firmware`. Kalau berbeda, perangkat akan mengunduh dan
mem-flash ulang terus-menerus karena menganggap versinya selalu tidak cocok.

`build/` dan berkas `.bin`/`.elf` tidak ikut git; binari OTA diunggah lewat
halaman admin Firmware, bukan lewat repo ini.
