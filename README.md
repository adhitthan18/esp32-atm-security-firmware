# esp32-atm-security-firmware

ESP32 firmware (Arduino) for a site-monitoring unit installed at ATM booths. It measures vibration, mains power, temperature and humidity, raises alerts, and switches two relays from a web dashboard. To enroll it into the platform, you scan a QR code shown on its own display.

This firmware ran on devices connected to the IoTMaxhub platform (now Adhi Teknologi). The server and dashboard live in separate repositories.

## Hardware

| Part | Purpose | Interface |
|---|---|---|
| ESP32 DevKit V1 (30-pin) | MCU | — |
| ADXL345 | vibration / tamper detection, sampled at 50 Hz | I²C (GPIO 21/22) |
| PZEM-004T v3.0 + 100 A CT | mains voltage, current, power, energy | UART2 (GPIO 16/17) |
| DHT22 | temperature and humidity | GPIO 4 |
| ST7789 1.3" 240×240 IPS | status pages and the pairing QR code | SPI, no CS pin |
| 2× relay module | remote switching (active LOW) | GPIO 18, 19 |

The full wiring diagram, including the mains side of the PZEM, is in the header comment of the sketch.

## Features

- **QR enrollment, no typing on the device.**
  1. On first boot the unit registers itself with a bootstrap key.
  2. It fetches a short-lived pairing token and draws it as a QR code on the ST7789. The QR encoder is bundled (`qrcode.c`) and the code refreshes before the token expires.
  3. An admin scans the code in the dashboard.
  4. The device polls until it is claimed, then stores its per-device API key in NVS.
- **Telemetry and alerts.**
  - Sensor data is posted over HTTPS every 5 seconds.
  - Alerts go out immediately for `VIBRATION_WARNING`/`VIBRATION_CRITICAL`, `LOW_VOLTAGE`/`HIGH_VOLTAGE`, and `HIGH_CURRENT`.
  - A 30-second cooldown per alert type keeps one event from flooding the server.
- **Relay control** from the dashboard: the device polls for the desired state and acknowledges each change.
- **Resilient WiFi.**
  - Primary and backup profiles with rotation.
  - After one minute offline, the unit opens a recovery access point with a captive portal (network scan, save, and the Android and iOS captive-portal probes).
  - Saving through the portal requires a separate admin password.
- **Remote settings.** WiFi profiles can be changed from the dashboard. Each change carries a revision number. The device saves it, restarts, and acknowledges the revision only once it is back online, so the dashboard shows whether the change worked.
- **OTA updates over HTTPS.** The firmware checks at boot and every 6 hours against the version published in the admin panel.
- **No secrets in source.** The bootstrap key and AP passwords live in an untracked `secrets.h`.

## Versions

| Folder | Version | Notes |
|---|---|---|
| [`iotmaxhub_atm_apmode_v4.5/`](iotmaxhub_atm_apmode_v4.5/) | 5.1 | HTTP only: telemetry every 5 s, relay polled every 2 s. The version deployed on devices. |
| [`iotmaxhub-atm-mqtt/`](iotmaxhub-atm-mqtt/) | 5.2-mqtt | 5.1 plus an MQTT-over-WSS path (`mqtt.ino`, esp-mqtt, certificate verified with `esp_crt_bundle_attach`). Not yet flashed to devices. |

What changes with MQTT in 5.2:

| | 5.1 | 5.2-mqtt |
|---|---|---|
| Telemetry | HTTP every 5 s | MQTT every 5 s; HTTP every 15 min as a fallback and API-key check |
| Relay | HTTP poll every 2 s | pushed instantly over MQTT; HTTP poll every 30 s as a fallback |
| Offline detection | — | MQTT Last Will on `iot/<MAC>/status` |

If `MQTT_URL` is left empty, MQTT is disabled and the firmware behaves like 5.1. Version 5.2 uses 98% of the default 4 MB app partition. Anything much larger needs the *Minimal SPIFFS* partition scheme.

## Building

1. Use Arduino IDE 2.x with board **ESP32 Dev Module** (Espressif `esp32` core).
2. Install these libraries: `Adafruit ADXL345`, `Adafruit GFX`, `Adafruit ST7789`, `ArduinoJson`, `DHT sensor library`, `PZEM004Tv30`.
3. In the sketch folder, create the secrets file, then fill in its three values:

   ```bash
   cp secrets.h.contoh secrets.h
   ```

4. Before releasing a build, bump `FIRMWARE_VER` and enter the same version on the admin panel's firmware page. If the two differ, the OTA check keeps reflashing the device.

Compiled binaries (`build/`, `*.bin`, `*.elf`) are not committed. OTA images are uploaded through the admin panel.

## Known limitations

- Apart from the MQTT connection, HTTPS requests use `WiFiClientSecure::setInsecure()`, so the server certificate is not verified. The next step is moving them to the certificate bundle that `mqtt.ino` already uses.
- API URLs are compile-time constants and still point to the platform's former `iotmaxhub.com` domain.

Code comments and serial logs are in Indonesian.
