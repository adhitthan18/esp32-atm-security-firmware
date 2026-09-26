// ═══════════════════════════════════════════════════════════════
//  IoTMaxhub — ESP32 Firmware v5.2 ST7789 QR + DUAL WIFI + MQTT
//  Board   : ESP32 DevKit V1 (30 pin)
//  Sensors : ADXL345 (vibrasi I2C) · PZEM-004T v3.0 (listrik UART)
//            DHT22 (suhu & kelembaban)
//  Comms   : Primary/backup WiFi → HTTPS API + QR claim provisioning
//            MQTT-over-WSS 443 untuk telemetri realtime & relay seketika
//            (mqtt.ino). Turunan iotmaxhub_atm_apmode_v4.5.
//
//  WIRING LENGKAP:
//  ┌──────────────────────────────────────────────────────────┐
//  │  DHT22      VCC  → 3.3V                                  │
//  │             GND  → GND                                   │
//  │             DATA → GPIO 4                                │
//  │             (10kΩ pull-up dari DATA ke 3.3V)             │
//  │                                                          │
//  │  ADXL345    VCC  → 3.3V                                  │
//  │             GND  → GND                                   │
//  │             SDA  → GPIO 21                               │
//  │             SCL  → GPIO 22                               │
//  │             CS   → 3.3V   (paksa mode I2C)               │
//  │             SDO  → GND    (I2C address = 0x53)           │
//  │                                                          │
//  │  LCD ST7789 1.3" 240x240 (SPI, modul tanpa pin CS):      │
//  │             SCL→GPIO25 SDA→GPIO26 RES→27 DC→14          │
//  │             VCC/BLK→3.3V, GND→GND                       │
//  │                                                          │
//  │  PZEM-004T  5V   → VIN (5V pin ESP32)                    │
//  │             GND  → GND                                   │
//  │             TX   → GPIO 16 (RX2 ESP32)                   │
//  │             RX   → GPIO 17 (TX2 ESP32)                   │
//  │             (TX ke RX dan RX ke TX — disilang!)          │
//  │                                                          │
//  │  PZEM sisi AC (220V — BERBAHAYA, matikan dulu sebelum):  │
//  │             L-IN/N-IN   ← dari MCB/PLN                   │
//  │             L-OUT/N-OUT → ke beban ATM                   │
//  │             CT 100A     → jepit ke kabel FASA saja       │
//  │             (CT punya arah — ikuti tanda panah/K→L)      │
//  │                                                          │
//  │  RELAY 1    IN   → GPIO 18  (active LOW)                 │
//  │  RELAY 2    IN   → GPIO 19  (active LOW)                 │
//  │             VCC  → 5V                                    │
//  │             GND  → GND                                   │
//  └──────────────────────────────────────────────────────────┘
//
//  LIBRARY YANG DIBUTUHKAN (install via Library Manager):
//    - Adafruit ADXL345         (by Adafruit)
//    - Adafruit Unified Sensor  (by Adafruit, dependency)
//    - DHT sensor library       (by Adafruit)
//    - PZEM004Tv30              (by olehs)
//    - Adafruit ST7735 and ST7789 Library (by Adafruit)
//    - Adafruit GFX Library     (by Adafruit)
//    - ArduinoJson              (by Benoit Blanchon, versi 6.x)
//
//  LOGIKA DETEKSI VIBRASI ADXL345:
//    - Baca akselerasi X,Y,Z tiap 100ms
//    - Hitung magnitude resultante: sqrt(x²+y²+z²)
//    - Bandingkan dengan baseline (rata-rata saat diam)
//    - Delta > VIB_WARN_THRESHOLD → WARNING
//    - Delta > VIB_CRIT_THRESHOLD → CRITICAL
//    - Cooldown 30 detik antar alert
// ═══════════════════════════════════════════════════════════════

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <esp_mac.h>   // untuk esp_read_mac() — baca MAC langsung dari eFuse hardware
#include <HTTPUpdate.h> // OTA firmware update over HTTPS
#include <HTTPClient.h>
#include <Preferences.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <DHT.h>
#include <Adafruit_ADXL345_U.h>
#include <PZEM004Tv30.h>
#include <ArduinoJson.h>
#include "qrcode.h"
#include "mbedtls/sha256.h"
// ─────────────────────────────────────────
//  PIN DEFINITIONS
// ─────────────────────────────────────────
#define DHTPIN          4
#define DHTTYPE         DHT22

#define I2C_SDA         21
#define I2C_SCL         22

#define PZEM_RX         16      // ESP32 RX2 ← PZEM TX
#define PZEM_TX         17      // ESP32 TX2 → PZEM RX
#define PZEM_SERIAL     Serial2

#define RELAY1_PIN      18      // active LOW
#define RELAY2_PIN      19      // active LOW
#define RELAY_COUNT     2

#define TFT_WIDTH       240
#define TFT_HEIGHT      240
#define TFT_SCLK        25
#define TFT_MOSI        26
#define TFT_RST         27
#define TFT_DC          14
#define TFT_CS          -1      // modul 7 pin: CS diikat LOW pada PCB
#define ADXL_ADDR       0x53    // SDO → GND

// ─────────────────────────────────────────
//  SERVER
// ─────────────────────────────────────────
#define SERVER_URL   "https://api.iotmaxhub.com/api/device/data"
#define RELAY_URL "https://api.iotmaxhub.com/relay_api"
#define OTA_URL   "https://api.iotmaxhub.com/ota_check"
#define REGISTER_URL  "https://api.iotmaxhub.com/api/device/register"
#define CLAIM_URL     "https://api.iotmaxhub.com/api/device/claim-status"
#define PAIRING_URL   "https://api.iotmaxhub.com/api/device/pairing-token"
// BOOTSTRAP_KEY, AP_PASS, dan CONFIG_ADMIN_PASS ada di secrets.h, yang
// sengaja tidak ikut git. Salin secrets.h.contoh menjadi secrets.h lalu isi
// nilainya sebelum compile.
#include "secrets.h"
#define DEVICE_TYPE_HINT "atm_security"   // hint tipe untuk halaman claim admin
#define PROV_POLL_MS  (30UL * 1000UL)     // poll claim tiap 30 detik
#define PAIRING_RETRY_MS   (30UL * 1000UL)
#define PAIRING_REFRESH_MS (4UL * 60UL * 1000UL)
// QR Version 3 / ECC_LOW: 567/8 - 15 = 55 codeword -> 53 karakter mode BYTE.
// Versi 2 hanya muat 32 karakter, lebih kecil dari buffer payload di bawah.
#define PAIRING_QR_VERSION 3
// Latar QR sengaja abu-abu terang, bukan putih penuh. Panel IPS 240x240 ini
// sangat terang; putih penuh membuat auto-exposure kamera jenuh dan cahaya
// meluber ke modul hitam di sebelahnya sehingga tepinya kabur. Menurunkan
// level putih memangkas silau tanpa mengurangi kontras secara berarti.
#define PAIRING_QR_BG   0xC618   // RGB565 ~ #C0C0C0

char gApiKey[64] = "";                    // diisi dari NVS saat boot / provisioning
uint8_t keyFailCount = 0;

// PENTING (OTA): tiap compile firmware baru, UBAH versi ini
// dan samakan dengan versi yang diisi di halaman /firmware.
// Kalau tidak sama, device akan download+flash berulang terus
// karena merasa versinya selalu "beda" dari server.
#define FIRMWARE_VER "5.2-mqtt"

// MQTT-over-WebSocket di port 443 (mqtt.ino). Kosong = dimatikan, perangkat
// kembali ke HTTP penuh seperti 5.1.
#define MQTT_URL "wss://mqtt.iotmaxhub.com:443/mqtt"
#define MQTT_TELEMETRY_MS  5000UL                 // telemetri realtime lewat MQTT
#define HTTP_INTERVAL_MQTT (15UL * 60UL * 1000UL) // HTTP selama MQTT tersambung
#define RELAY_POLL_MS      2000UL                 // poll relay tanpa MQTT
#define RELAY_POLL_MQTT_MS (30UL * 1000UL)        // poll relay selama MQTT tersambung

// ─────────────────────────────────────────
//  WIFI + AP MODE
// ─────────────────────────────────────────
#define WIFI_RETRY_MS      (30UL * 1000UL)     // rotasi/retry profil tiap 30 detik
#define WIFI_CONNECT_TIMEOUT_MS (12UL * 1000UL) // batas tunggu per profil saat boot
#define WIFI_RECOVERY_MS   (60UL * 1000UL)     // 1 menit putus -> AP recovery

// Interval cek OTA: tiap 6 jam + sekali 1 menit setelah boot
#define OTA_CHECK_INTERVAL (6UL * 3600UL * 1000UL)
#define OTA_BOOT_DELAY     (60UL * 1000UL)

// ─────────────────────────────────────────
//  THRESHOLD VIBRASI ADXL345
//  Satuan: m/s² (meter per detik kuadrat)
//  1g ≈ 9.81 m/s² = nilai saat sensor diam rata
//  Delta = perubahan magnitude dari baseline
// ─────────────────────────────────────────
#define VIB_WARN_THRESHOLD   2.0f   // m/s² → WARNING
#define VIB_CRIT_THRESHOLD   5.0f   // m/s² → CRITICAL
#define VIB_COOLDOWN_MS      30000UL

// ─────────────────────────────────────────
//  THRESHOLD LISTRIK (PZEM)
// ─────────────────────────────────────────
#define VOLT_MIN         200.0f   // V AC
#define VOLT_MAX         240.0f   // V AC
#define CURR_MAX          20.0f   // A AC

// ─────────────────────────────────────────
//  INTERVAL
// ─────────────────────────────────────────
#define HTTP_INTERVAL     5000UL    // kirim data ke server tiap 5 detik
#define SENSOR_INTERVAL   20UL      // ADXL345 dibaca 50x/detik
#define DISPLAY_INTERVAL  1000UL
#define DISPLAY_PAGE_MS   3000UL

// ─────────────────────────────────────────
//  OBJEK SENSOR
// ─────────────────────────────────────────
DHT                  dht(DHTPIN, DHTTYPE);
SPIClass             displaySPI(HSPI);
Adafruit_ST7789      display(&displaySPI, TFT_CS, TFT_DC, TFT_RST);
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);
PZEM004Tv30          pzem(PZEM_SERIAL, PZEM_RX, PZEM_TX);
Preferences          prefs;
WebServer            server(80);
DNSServer            dnsServer;
const byte           DNS_PORT = 53;
// ─────────────────────────────────────────
//  STATE GLOBAL
// ─────────────────────────────────────────
// Sensor readings
float gTemp     = 0.0f;
float gHum      = 0.0f;
float gAccX     = 0.0f;  // m/s²
float gAccY     = 0.0f;
float gAccZ     = 9.81f; // Z ≈ 9.81 saat diam (gravity)
float gVibDelta = 0.0f;  // delta magnitude dari baseline
float gVoltage  = 0.0f;
float gCurrent  = 0.0f;
float gPower    = 0.0f;
float gEnergy   = 0.0f;
float gFreq     = 0.0f;
float gPF       = 0.0f;

bool  displayOK    = false;
bool  adxlOK    = false;
bool  pzemOK    = false;
uint8_t adxlAddress = 0;

// Relay
bool relayState[RELAY_COUNT + 1] = {false, false, false};

// ADXL345 baseline per sumbu. Sensor harus diam saat kalibrasi awal.
float vibBaselineMag = 9.81f;
float vibBaseX       = 0.0f;
float vibBaseY       = 0.0f;
float vibBaseZ       = 9.81f;
float vibSumX        = 0.0f;
float vibSumY        = 0.0f;
float vibSumZ        = 0.0f;
int   vibBaseSamples = 0;
const int BASELINE_N = 100;     // 100 sampel @ 50Hz = sekitar 2 detik

// Alert management
struct PendingAlert {
  String type, message, severity;
  int value;
};
PendingAlert pendingAlerts[4];
int pendingCount = 0;

unsigned long lastVibWarnAlert = 0;
unsigned long lastVibCritAlert = 0;
unsigned long lastVoltAlert    = 0;
unsigned long lastCurrAlert    = 0;
int totalAlertsSent = 0;

// WiFi. Key NVS lama "ssid"/"pass" tetap menjadi profil primary.
String gSSID, gPass, gSSIDBackup, gPassBackup, gMacFull;
uint8_t activeWiFiProfile = 0;    // 0=none, 1=primary, 2=backup
uint8_t attemptedWiFiProfile = 0;
bool   wifiConfigMode = false;
bool   waitSSID = false, waitPASS = false;
bool   apModeActive = false;
bool   recoveryAP = false;
bool   webRoutesReady = false;
bool   webServerStarted = false;

// QR provisioning berjalan non-blocking agar sensor, relay, AP, dan WiFi
// failover tetap hidup selama menunggu pengguna melakukan claim dari app.
enum ProvisioningState : uint8_t {
  PROV_NOT_STARTED,
  PROV_REGISTERING,
  PROV_WAITING_QR,
  PROV_ACTIVE,
  PROV_NEEDS_RESET
};
ProvisioningState gProvState = PROV_NOT_STARTED;
char gProvSecret[64] = "";
char gPairingPayload[54] = "";   // 53 karakter + NUL, sesuai kapasitas QR v3
uint16_t provPollCount = 0;
unsigned long tProvPoll = 0;
unsigned long tPairingRequest = 0;

// HTTP stats
int httpOkCount   = 0;
int httpFailCount = 0;
int lastHttpCode  = 0;

// Timers
unsigned long tHTTP    = 0;
unsigned long tSensor  = 0;
unsigned long tDHT     = 0;
unsigned long tPZEM    = 0;
unsigned long tADXLLog = 0;
unsigned long tDisplay    = 0;
unsigned long tDisplayPage= 0;
unsigned long tRelay   = 0;
unsigned long tMqttTele = 0;
unsigned long tOTA     = 0;
bool otaBootChecked    = false;
unsigned long wifiLostSince = 0;   // sejak kapan WiFi putus (0 = tidak putus)
unsigned long tWifiRetry    = 0;
uint8_t displayPage = 0;

// ─────────────────────────────────────────
//  FORWARD DECLARATIONS
// ─────────────────────────────────────────
void connectWiFi();
bool beginWiFiProfile(uint8_t profile, bool waitForConnection = false);
void tryNextWiFiProfile();
bool hasWiFiCredentials();
const char* activeWiFiProfileName();
void startAPMode(bool recoveryMode = false);
void stopAPMode();
void setupWebRoutes();
void handlePortalRoot();
void handlePortalData();
void handlePortalScan();
void handlePortalSave();
bool isPortalMode();
String htmlEscape(String value);
void readSensors();
bool i2cDevicePresent(uint8_t address);
void resetVibrationBaseline();
void sendTelemetry();
void sendAlert(const String&, const String&, const String&, int);
int httpPostWithRetry(const char* url, const char* payload, int timeoutMs, int maxRetries);
void pollRelayCommands();
void sendRelayAck(int, int);
void checkOTA();
void tftVibration();
void tftSensors();
void tftNetwork();
void tftPairingQR();
void tftBoot(const char*);
// ═══════════════════════════════════════════════
//  SETUP
// ═══════════════════════════════════════════════
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);  // disable brownout

  Serial.begin(115200);
  delay(500);
  Serial.printf("\n=== IoTMaxhub v%s ST7789 QR + DUAL WIFI BOOT ===\n", FIRMWARE_VER);

  // ── Relay — OFF dulu ──
  pinMode(RELAY1_PIN, OUTPUT); digitalWrite(RELAY1_PIN, HIGH);
  pinMode(RELAY2_PIN, OUTPUT); digitalWrite(RELAY2_PIN, HIGH);
  Serial.println(F("[RELAY] init OFF (active LOW)"));

  // ── DHT22 ──
  dht.begin();
  Serial.println(F("[DHT22] init OK"));

  // ── I2C (ADXL345) ──
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(100000);
  delay(100);

  Serial.printf("[I2C] SDA=%d SCL=%d\n", I2C_SDA, I2C_SCL);
  Serial.printf("[I2C] ADXL 0x53: %s\n",
                i2cDevicePresent(0x53) ? "TERDETEKSI" : "TIDAK ADA");
  Serial.printf("[I2C] ADXL 0x1D: %s\n",
                i2cDevicePresent(0x1D) ? "TERDETEKSI" : "TIDAK ADA");

  // ── LCD IPS ST7789 240x240 (SPI tanpa CS) ──
  displaySPI.begin(TFT_SCLK, -1, TFT_MOSI, TFT_CS);
  // Modul ST7789 tanpa pin CS mensyaratkan clock diam HIGH (SPI_MODE3).
  // Tanpa CS, keadaan diam clock adalah satu-satunya penanda awal transaksi;
  // dengan MODE0 bawaan seluruh perintah terbaca bergeser dan panel tidak
  // pernah keluar dari sleep — backlight menyala, layar tetap hitam.
  display.init(TFT_WIDTH, TFT_HEIGHT, SPI_MODE3);
  display.setSPISpeed(24000000);
  display.setRotation(0);
  display.setTextWrap(false);
  // Uji mandiri: kalau SPI bekerja, layar berkedip merah-hijau-biru sekali
  // saat boot. Kalau tetap hitam, sebabnya di SPI atau wiring — bukan di
  // fungsi gambar mana pun. Hapus blok ini setelah display terbukti hidup.
  display.fillScreen(ST77XX_RED);   delay(300);
  display.fillScreen(ST77XX_GREEN); delay(300);
  display.fillScreen(ST77XX_BLUE);  delay(300);

  display.fillScreen(ST77XX_BLACK);
  displayOK = true;
  Serial.printf("[TFT] ST7789 init OK SCK=%d MOSI=%d RST=%d DC=%d\n",
                TFT_SCLK, TFT_MOSI, TFT_RST, TFT_DC);
  tftBoot("Booting...");
  // ADXL345: coba alamat 0x53 (SDO=GND), lalu 0x1D (SDO=3.3V)
  if (i2cDevicePresent(0x53) && adxl.begin(0x53)) {
    adxlAddress = 0x53;
  } else if (i2cDevicePresent(0x1D) && adxl.begin(0x1D)) {
    adxlAddress = 0x1D;
  }

  if (adxlAddress != 0) {
    adxlOK = true;
    adxl.setRange(ADXL345_RANGE_2_G);
    adxl.setDataRate(ADXL345_DATARATE_100_HZ);
    resetVibrationBaseline();
    Serial.printf("[ADXL345] init OK address=0x%02X range=+-2g rate=100Hz\n",
                  adxlAddress);
    Serial.println(F("[ADXL345] Diamkan sensor sekitar 2 detik untuk kalibrasi"));
    tftBoot("ADXL345 OK");
  } else {
    adxlOK = false;
    Serial.println(F("[ADXL345] TIDAK DITEMUKAN"));
    Serial.println(F("[ADXL345] I2C: SDA=21 SCL=22 CS=3.3V SDO=GND/3.3V"));
    tftBoot("ADXL345 ERROR!");
  }

  // ── PZEM-004T ──
  // PZEM_SERIAL (Serial2) otomatis diinisialisasi oleh library
  Serial.printf("[PZEM] UART2 RX=%d TX=%d\n", PZEM_RX, PZEM_TX);
  tftBoot("PZEM init...");

  // ── WiFi ──
  prefs.begin("wifi", true);
  gSSID = prefs.getString("ssid", "");
  gPass = prefs.getString("pass", "");
  gSSIDBackup = prefs.getString("ssid2", "");
  gPassBackup = prefs.getString("pass2", "");
  prefs.end();

  WiFi.mode(WIFI_STA);
  
  uint8_t macBytes[6];
  char macBuf[18];
  int macRetry = 0;
  do {
    esp_read_mac(macBytes, ESP_MAC_WIFI_STA);
    snprintf(macBuf, sizeof(macBuf), "%02X:%02X:%02X:%02X:%02X:%02X",
             macBytes[0], macBytes[1], macBytes[2], macBytes[3], macBytes[4], macBytes[5]);
    gMacFull = String(macBuf);

    // Validasi: MAC semua-nol berarti baca gagal, coba lagi
    bool allZero = (macBytes[0]==0 && macBytes[1]==0 && macBytes[2]==0 &&
                     macBytes[3]==0 && macBytes[4]==0 && macBytes[5]==0);
    if (!allZero) break;

    Serial.printf("[WiFi] MAC baca gagal (semua nol), retry %d/5...\n", macRetry + 1);
    delay(200);
    macRetry++;
  } while (macRetry < 5);

  Serial.printf("[WiFi] MAC: %s%s\n", gMacFull.c_str(),
                macRetry >= 5 ? " (WARNING: gagal setelah 5x percobaan!)" : "");

  tftBoot("Connecting WiFi...");
  if (!hasWiFiCredentials()) {
    Serial.println(F("[WiFi] Belum ada konfigurasi — membuka AP Mode"));
    startAPMode(false);
  } else {
    connectWiFi();
    if (!WiFi.isConnected()) {
      startAPMode(true);
    }
  }
  if (WiFi.isConnected()) provEnsureApiKey();
  Serial.println(F("[BOOT] Selesai"));
}

// ═══════════════════════════════════════════════
//  LOOP
// ═══════════════════════════════════════════════
void loop() {
  // Layani captive portal AP Mode
  if (apModeActive) {
    dnsServer.processNextRequest();
  }
  if (webServerStarted) {
    server.handleClient();
  }

  // Selama esp-mqtt resolve/TLS/handshake, semua HTTP di loop menunggu
  // (DNS bersamaan bisa membuat lwIP panic, lihat mqtt.ino).
  const bool netFree = !mqttNetworkBusy();
  // Juga saat key kosong: key yang dicabut harus ikut menghentikan sesi MQTT.
  if (WiFi.isConnected()) mqttTick();

  // Provisioning non-blocking: satu langkah per interval, loop tidak berhenti.
  if (netFree && WiFi.isConnected() && gApiKey[0] == '\0') provEnsureApiKey();
  // ── Serial command ──
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd == "CONFIG") {
      wifiConfigMode = true; waitSSID = true;
      Serial.println(F("Enter SSID:"));
    } else if (wifiConfigMode && waitSSID) {
      gSSID = cmd; waitSSID = false; waitPASS = true;
      Serial.println(F("Enter Password:"));
    } else if (wifiConfigMode && waitPASS) {
      gPass = cmd; waitPASS = false; wifiConfigMode = false;
      prefs.begin("wifi", false);
      prefs.putString("ssid", gSSID);
      prefs.putString("pass", gPass);
      prefs.end();
      connectWiFi();
    } else if (cmd == "STATUS") {
      Serial.printf("[STATUS] T:%.1f H:%.1f Vib:%.3f\n", gTemp, gHum, gVibDelta);
      Serial.printf("         V:%.1f I:%.3f P:%.1f E:%.3f PF:%.2f\n",
                    gVoltage, gCurrent, gPower, gEnergy, gPF);
      Serial.printf("         ADXL:%s addr:0x%02X cal:%d/%d PZEM:%s WiFi:%s AP:%s\n",
                    adxlOK?"OK":"ERR", adxlAddress, vibBaseSamples, BASELINE_N,
                    pzemOK?"OK":"ERR", WiFi.isConnected()?"OK":"NO",
                    apModeActive?"ON":"OFF");
      Serial.printf("         WiFi profile:%s primary:%s backup:%s\n",
                    activeWiFiProfileName(), gSSID.length() > 0 ? "SET" : "EMPTY",
                    gSSIDBackup.length() > 0 ? "SET" : "EMPTY");
      Serial.printf("         Provisioning:%s\n", provisioningStateName());
      if (apModeActive) {
        Serial.printf("         AP SSID:%s IP:%s\n",
                      WiFi.softAPSSID().c_str(), WiFi.softAPIP().toString().c_str());
      }
    } else if (cmd == "APMODE") {
      startAPMode(false);
    } else if (cmd == "RESTART") {
      ESP.restart();
    } else if (cmd == "RESETVIB") {
      resetVibrationBaseline();
      Serial.println(F("[ADXL345] Baseline di-reset. Diamkan sensor sekitar 2 detik."));
    } else if (cmd == "CLEARWIFI") {
      prefs.begin("wifi", false);
      prefs.clear();
      prefs.end();
      Serial.println(F("[WiFi] Config WiFi dihapus, restart..."));
      delay(500);
      ESP.restart();
    }
  }

  unsigned long now = millis();

  // ── Baca sensor (tiap 100ms untuk ADXL345 real-time) ──
  if (now - tSensor >= SENSOR_INTERVAL) {
    tSensor = now;
    readSensors();
  }

  // ── Relay polling ──
  // Dengan MQTT, perintah datang seketika; poll dijarangkan dan tinggal
  // menjemput jadwal, OTA, dan perintah yang terkirim saat MQTT terputus.
  unsigned long relayEvery = mqttConnected() ? RELAY_POLL_MQTT_MS : RELAY_POLL_MS;
  if (mqttTakePollRequest()) tRelay = now - relayEvery;
  if (netFree && WiFi.isConnected() && gApiKey[0] != '\0' && now - tRelay >= relayEvery) {
    tRelay = now;
    pollRelayCommands();
  }

  // ── OTA check: hanya setelah device sudah memiliki credential aktif ──
  if (netFree && WiFi.isConnected() && gApiKey[0] != '\0') {
    if (!otaBootChecked && now >= OTA_BOOT_DELAY) {
      otaBootChecked = true;
      tOTA = now;
      checkOTA();
    } else if (otaBootChecked && now - tOTA >= OTA_CHECK_INTERVAL) {
      tOTA = now;
      checkOTA();
    }
  }

  // ── PEMELIHARAAN WIFI + AP RECOVERY ──
  wl_status_t wifiStatus = WiFi.status();

  if (wifiStatus == WL_CONNECTED) {
    wifiLostSince = 0;
    if (attemptedWiFiProfile != 0 && activeWiFiProfile != attemptedWiFiProfile) {
      activeWiFiProfile = attemptedWiFiProfile;
      Serial.printf("[WiFi] Failover berhasil: profil %s aktif, SSID=%s\n",
                    activeWiFiProfileName(), WiFi.SSID().c_str());
    }

    // AP recovery otomatis ditutup setelah WiFi berhasil pulih.
    // AP konfigurasi manual tetap aktif sampai konfigurasi disimpan/restart.
    if (apModeActive && recoveryAP) {
      stopAPMode();
    }
  } else if (hasWiFiCredentials()) {
    if (wifiLostSince == 0) {
      wifiLostSince = now;
      tWifiRetry = now;
      Serial.printf("[WiFi] Putus dari profil %s\n", activeWiFiProfileName());
    }

    // Setelah putus cukup lama, buka AP tanpa menghentikan usaha koneksi STA.
    if (!apModeActive && now - wifiLostSince >= WIFI_RECOVERY_MS) {
      startAPMode(true);
    }

    if (now - tWifiRetry >= WIFI_RETRY_MS) {
      tWifiRetry = now;
      tryNextWiFiProfile();
    }
  } else if (!apModeActive) {
    // Tidak ada kredensial WiFi tersimpan.
    startAPMode(false);
  }

  // ── Kirim data ──
  // Alert selalu lewat HTTP seketika (tersimpan utuh di server). Telemetri
  // tiap MQTT_TELEMETRY_MS lewat MQTT; HTTP tetap berjalan tiap 15 menit
  // sebagai cadangan dan pemeriksa key. Tanpa MQTT, HTTP kembali tiap 5 detik.
  if (netFree && WiFi.isConnected() && gApiKey[0] != '\0') {
    for (int i = 0; i < pendingCount; i++) {
      sendAlert(pendingAlerts[i].type, pendingAlerts[i].message,
                pendingAlerts[i].severity, pendingAlerts[i].value);
      delay(200);
    }
    pendingCount = 0;

    unsigned long httpEvery = mqttConnected() ? HTTP_INTERVAL_MQTT : HTTP_INTERVAL;
    if (now - tHTTP >= httpEvery) {
      tHTTP = now;
      sendTelemetry();
    }
  }
  if (mqttConnected() && now - tMqttTele >= MQTT_TELEMETRY_MS) {
    tMqttTele = now;
    char payload[768];
    if (buildTelemetry(payload, sizeof(payload))) mqttPublishTelemetry(payload);
  }

  // ── Update OLED ──
  if (now - tDisplay >= DISPLAY_INTERVAL) {
    tDisplay = now;
    if (gProvState == PROV_WAITING_QR && gPairingPayload[0] != '\0') {
      // Pertahankan QR di layar agar teknisi punya waktu memindainya.
      tftPairingQR();
    } else {
      if (now - tDisplayPage >= DISPLAY_PAGE_MS) {
        tDisplayPage = now;
        displayPage = (displayPage + 1) % 3;
      }
      switch (displayPage) {
        case 0: tftVibration(); break;
        case 1: tftSensors();   break;
        case 2: tftNetwork();   break;
      }
    }
  }
}

// ═══════════════════════════════════════════════
//  HELPER I2C DAN KALIBRASI ADXL345
// ═══════════════════════════════════════════════
bool i2cDevicePresent(uint8_t address) {
  Wire.beginTransmission(address);
  return Wire.endTransmission() == 0;
}

void resetVibrationBaseline() {
  vibBaseSamples = 0;
  vibSumX = 0.0f;
  vibSumY = 0.0f;
  vibSumZ = 0.0f;
  vibBaseX = 0.0f;
  vibBaseY = 0.0f;
  vibBaseZ = 9.81f;
  vibBaselineMag = 9.81f;
  gVibDelta = 0.0f;
}

// ═══════════════════════════════════════════════
//  BACA SENSOR
//  ADXL345 : setiap 20 ms (50 Hz)
//  PZEM    : setiap 1 detik
//  DHT22   : setiap 2 detik
// ═══════════════════════════════════════════════
void readSensors() {
  const unsigned long nowSensor = millis();

  // ── ADXL345 — Deteksi vibrasi berdasarkan perubahan tiap sumbu ──
  if (adxlOK) {
    sensors_event_t event;
    adxl.getEvent(&event);

    gAccX = event.acceleration.x;
    gAccY = event.acceleration.y;
    gAccZ = event.acceleration.z;

    // Nilai tidak masuk akal menandakan pembacaan I2C bermasalah.
    const bool validAcc = isfinite(gAccX) && isfinite(gAccY) && isfinite(gAccZ) &&
                          fabsf(gAccX) < 40.0f && fabsf(gAccY) < 40.0f &&
                          fabsf(gAccZ) < 40.0f;

    if (!validAcc) {
      Serial.println(F("[ADXL345] Data tidak valid, cek kabel/power"));
    } else if (vibBaseSamples < BASELINE_N) {
      vibSumX += gAccX;
      vibSumY += gAccY;
      vibSumZ += gAccZ;
      vibBaseSamples++;
      gVibDelta = 0.0f;

      if (vibBaseSamples == BASELINE_N) {
        vibBaseX = vibSumX / BASELINE_N;
        vibBaseY = vibSumY / BASELINE_N;
        vibBaseZ = vibSumZ / BASELINE_N;
        vibBaselineMag = sqrtf(vibBaseX * vibBaseX +
                               vibBaseY * vibBaseY +
                               vibBaseZ * vibBaseZ);

        Serial.printf("[ADXL345] Baseline OK X:%.3f Y:%.3f Z:%.3f Mag:%.3f m/s2\n",
                      vibBaseX, vibBaseY, vibBaseZ, vibBaselineMag);
      }
    } else {
      const float deltaX = gAccX - vibBaseX;
      const float deltaY = gAccY - vibBaseY;
      const float deltaZ = gAccZ - vibBaseZ;

      // Resultan perubahan terhadap posisi awal sensor.
      const float rawDelta = sqrtf(deltaX * deltaX +
                                   deltaY * deltaY +
                                   deltaZ * deltaZ);

      // Filter ringan agar noise tidak mudah memicu alarm.
      gVibDelta = (0.70f * gVibDelta) + (0.30f * rawDelta);

      if (nowSensor - tADXLLog >= 500UL) {
        tADXLLog = nowSensor;
        Serial.printf("[ADXL345] X:%7.3f Y:%7.3f Z:%7.3f Vib:%7.3f\n",
                      gAccX, gAccY, gAccZ, gVibDelta);
      }

      // Alert vibrasi
      if (gVibDelta >= VIB_CRIT_THRESHOLD &&
          (lastVibCritAlert == 0 || nowSensor - lastVibCritAlert >= VIB_COOLDOWN_MS)) {
        lastVibCritAlert = nowSensor;
        String msg = "Guncangan keras terdeteksi: delta " +
                     String(gVibDelta, 2) + " m/s²";
        if (pendingCount < 4) {
          pendingAlerts[pendingCount++] = {
            "VIBRATION_CRITICAL", msg, "critical", (int)(gVibDelta * 100)
          };
        }
      } else if (gVibDelta >= VIB_WARN_THRESHOLD &&
                 (lastVibWarnAlert == 0 || nowSensor - lastVibWarnAlert >= VIB_COOLDOWN_MS)) {
        lastVibWarnAlert = nowSensor;
        String msg = "Getaran terdeteksi: delta " +
                     String(gVibDelta, 2) + " m/s²";
        if (pendingCount < 4) {
          pendingAlerts[pendingCount++] = {
            "VIBRATION_WARNING", msg, "warning", (int)(gVibDelta * 100)
          };
        }
      }
    }
  }

  // ── DHT22 — jangan dibaca lebih cepat dari 2 detik ──
  if (nowSensor - tDHT >= 2000UL) {
    tDHT = nowSensor;

    const float t = dht.readTemperature();
    const float h = dht.readHumidity();

    if (!isnan(t) && t > -40.0f && t < 80.0f) {
      gTemp = t;
    }
    if (!isnan(h) && h >= 0.0f && h <= 100.0f) {
      gHum = h;
    }
  }

  // ── PZEM-004T — cukup dibaca setiap 1 detik ──
  if (nowSensor - tPZEM >= 1000UL) {
    tPZEM = nowSensor;

    const float v  = pzem.voltage();
    const float i  = pzem.current();
    const float p  = pzem.power();
    const float e  = pzem.energy();
    const float f  = pzem.frequency();
    const float pf = pzem.pf();

    if (!isnan(v) && !isnan(i)) {
      pzemOK   = true;
      gVoltage = v;
      gCurrent = i;
      gPower   = isnan(p)  ? v * i : p;
      gEnergy  = isnan(e)  ? gEnergy : e;
      gFreq    = isnan(f)  ? 0.0f : f;
      gPF      = isnan(pf) ? 0.0f : pf;

      // Alert tegangan
      if (lastVoltAlert == 0 || nowSensor - lastVoltAlert >= VIB_COOLDOWN_MS) {
        if (gVoltage < VOLT_MIN) {
          lastVoltAlert = nowSensor;
          String msg = "Tegangan AC rendah: " + String(gVoltage, 1) + "V";
          if (pendingCount < 4) {
            pendingAlerts[pendingCount++] = {
              "LOW_VOLTAGE", msg, "warning", (int)gVoltage
            };
          }
        } else if (gVoltage > VOLT_MAX) {
          lastVoltAlert = nowSensor;
          String msg = "Tegangan AC tinggi: " + String(gVoltage, 1) + "V";
          if (pendingCount < 4) {
            pendingAlerts[pendingCount++] = {
              "HIGH_VOLTAGE", msg, "warning", (int)gVoltage
            };
          }
        }
      }

      // Alert arus
      if (gCurrent > CURR_MAX &&
          (lastCurrAlert == 0 || nowSensor - lastCurrAlert >= VIB_COOLDOWN_MS)) {
        lastCurrAlert = nowSensor;
        String msg = "Arus berlebih: " + String(gCurrent, 2) + "A";
        if (pendingCount < 4) {
          pendingAlerts[pendingCount++] = {
            "HIGH_CURRENT", msg, "critical", (int)gCurrent
          };
        }
      }
    } else {
      pzemOK = false;
    }
  }
}

// ═══════════════════════════════════════════════
//  HTTP POST DENGAN RETRY — tahan gangguan WiFi/TLS sesaat
//
//  Sebelumnya: http.begin(SERVER_URL) pakai URL string biasa,
//  ESP32 core otomatis buat WiFiClientSecure internal tanpa kita
//  kendalikan. Sekarang eksplisit pakai WiFiClientSecure supaya:
//    - setInsecure() jelas dipanggil (skip validasi sertifikat,
//      lebih cepat & lebih toleran di WiFi lemah)
//    - Connect timeout & response timeout dipisah, lebih presisi
//    - Kalau gagal (network/TLS blip sesaat), retry otomatis
//      2x dengan jeda singkat sebelum benar-benar dianggap FAIL
//
//  Return: HTTP status code (200 dst), atau kode negatif
//  HTTPClient kalau semua percobaan gagal.
// ═══════════════════════════════════════════════
int httpPostWithRetry(const char* url, const char* payload, int timeoutMs, int maxRetries) {
  int code = -1;

  for (int attempt = 0; attempt <= maxRetries; attempt++) {
    WiFiClientSecure client;
    client.setInsecure();          // skip validasi sertifikat (cocok untuk ESP32, lebih ringan)
    client.setTimeout(timeoutMs / 1000); // detik, dipakai saat baca stream

    HTTPClient http;
    http.setConnectTimeout(5000);  // maks 5 detik untuk fase connect/TLS handshake
    http.setTimeout(timeoutMs);    // maks timeoutMs untuk fase baca response

    if (!http.begin(client, url)) {
      Serial.printf("[HTTP] begin() gagal (percobaan %d/%d)\n", attempt + 1, maxRetries + 1);
      code = -1;
    } else {
      http.addHeader("Content-Type", "application/json");
      http.addHeader("X-API-KEY", gApiKey);
      code = http.POST((uint8_t*)payload, strlen(payload));
      http.end();
    }

    if (code == 200) return code; // sukses, tidak perlu retry

    // Gagal — log detail termasuk RSSI untuk diagnosis jaringan
    Serial.printf("[HTTP] FAIL %d (percobaan %d/%d) RSSI:%ddBm Heap:%d\n",
                  code, attempt + 1, maxRetries + 1, WiFi.RSSI(), (int)ESP.getFreeHeap());

    if (attempt < maxRetries) {
      delay(500 + attempt * 500); // backoff singkat: 500ms, lalu 1000ms
    }
  }

  return code; // semua percobaan gagal, kembalikan kode terakhir
}

void provSHA256Hex(const char* in, char* outHex) {
  unsigned char hash[32];
  mbedtls_sha256((const unsigned char*)in, strlen(in), hash, 0);
  for (int i = 0; i < 32; i++) sprintf(outHex + i * 2, "%02x", hash[i]);
  outHex[64] = '\0';
}

// Pastikan device punya secret (sekali seumur hidup, tersimpan NVS)
void provEnsureSecret(char* secretOut, size_t cap) {
  prefs.begin("iotmax", false);
  String s = prefs.getString("prov_secret", "");
  if (s.length() == 0) {
    // generate 24 byte acak → 48 hex char
    char buf[49];
    for (int i = 0; i < 24; i++) sprintf(buf + i * 2, "%02x", (uint8_t)(esp_random() & 0xFF));
    buf[48] = '\0';
    s = String(buf);
    prefs.putString("prov_secret", s);
    Serial.println("[PROV] Secret baru dibuat & disimpan NVS");
  }
  prefs.end();
  s.toCharArray(secretOut, cap);
}

// POST JSON dengan header X-BOOTSTRAP-KEY. Return body (atau "" kalau gagal).
String provPost(const char* url, const String& payload) {
  WiFiClientSecure client;
  client.setInsecure();  // konsisten dgn pola HTTPS existing firmware
  HTTPClient http;
  http.setConnectTimeout(5000);
  http.setTimeout(10000);
  String body = "";
  if (http.begin(client, url)) {
    http.addHeader("Content-Type", "application/json");
    http.addHeader("X-BOOTSTRAP-KEY", BOOTSTRAP_KEY);
    int code = http.POST(payload);
    if (code > 0) body = http.getString();
    Serial.printf("[PROV] POST %s → %d\n", url, code);
    http.end();
  }
  return body;
}

// Register ke server (masuk antrean pending di admin)
void provRegister(const char* secret) {
  char secretHash[65];
  provSHA256Hex(secret, secretHash);

  StaticJsonDocument<384> doc;
  doc["mac_address"] = gMacFull;
  doc["secret_hash"] = secretHash;
  doc["device_type"] = DEVICE_TYPE_HINT;
  doc["firmware"]    = FIRMWARE_VER;
  doc["rssi"]        = WiFi.RSSI();
  String payload;
  serializeJson(doc, payload);
  provPost(REGISTER_URL, payload);
}

bool provRequestPairingToken(const char* secret) {
  tPairingRequest = millis();
  StaticJsonDocument<256> doc;
  doc["mac_address"] = gMacFull;
  doc["secret"] = secret;
  String payload;
  serializeJson(doc, payload);

  String body = provPost(PAIRING_URL, payload);
  if (body.length() == 0) return false;
  StaticJsonDocument<512> resp;
  if (deserializeJson(resp, body) != DeserializationError::Ok) return false;
  const char* qr = resp["qr_payload"] | "";
  if (strncmp(qr, "IMXPAIR1.", 9) != 0) {
    Serial.printf("[PROV] qr_payload prefix tidak dikenal: \"%.20s\"\n", qr);
    return false;
  }
  if (strlen(qr) >= sizeof(gPairingPayload)) {
    Serial.printf("[PROV] qr_payload %u byte, buffer %u - ditolak\n",
                  (unsigned)strlen(qr), (unsigned)sizeof(gPairingPayload));
    return false;
  }
  strlcpy(gPairingPayload, qr, sizeof(gPairingPayload));
  Serial.printf("[PROV] Pairing QR diterima (%u byte), berlaku 5 menit\n", strlen(qr));
  // Isi mentah payload. Ini kredensial pairing berumur 5 menit, bukan secret
  // permanen perangkat — aman dicetak ke Serial saat debugging.
  Serial.printf("[PROV] Isi QR: %s\n", gPairingPayload);
  return true;
}

// Poll claim-status. Return true kalau key diterima (tersimpan ke NVS+gApiKey).
bool provPollClaim(const char* secret) {
  StaticJsonDocument<256> doc;
  doc["mac_address"] = gMacFull;
  doc["secret"]      = secret;
  String payload;
  serializeJson(doc, payload);

  String body = provPost(CLAIM_URL, payload);
  if (body.length() == 0) return false;

  StaticJsonDocument<384> resp;
  if (deserializeJson(resp, body) != DeserializationError::Ok) return false;

  const char* status = resp["status"] | "";
  Serial.printf("[PROV] status: %s\n", status);

  if (strcmp(status, "claimed") == 0) {
    const char* key = resp["api_key"] | "";
    if (strlen(key) > 10) {
      prefs.begin("iotmax", false);
      prefs.putString("api_key", key);
      prefs.end();
      strlcpy(gApiKey, key, sizeof(gApiKey));
      Serial.println("[PROV] API key diterima & tersimpan NVS!");
      return true;
    }
  }
  if (strcmp(status, "active") == 0 && gApiKey[0] == '\0') {
    gProvState = PROV_NEEDS_RESET;
    Serial.println("[PROV] ERROR: key sudah pernah dikirim tetapi hilang dari NVS.");
    Serial.println("[PROV] Release/reset claim device dari admin sebelum mencoba lagi.");
  }
  return false;
}

const char* provisioningStateName() {
  switch (gProvState) {
    case PROV_REGISTERING: return "registering";
    case PROV_WAITING_QR: return "waiting_qr_claim";
    case PROV_ACTIVE: return "active";
    case PROV_NEEDS_RESET: return "needs_admin_reset";
    default: return "not_started";
  }
}

// Satu pemanggilan hanya melakukan maksimal satu request. Tidak ada while/delay
// panjang, sehingga QR dapat di-scan sebelum atau sesudah device menyala tanpa
// membekukan portal AP, sensor, relay, OTA, atau failover WiFi.
void provEnsureApiKey() {
  // Self-healing: credential yang hilang/dicabut tidak boleh meninggalkan
  // state ACTIVE, karena state itu akan menghalangi register dan QR baru.
  if (gApiKey[0] == '\0' && gProvState == PROV_ACTIVE) {
    gProvState = PROV_NOT_STARTED;
    gPairingPayload[0] = '\0';
    provPollCount = 0;
    tProvPoll = 0;
    tPairingRequest = 0;
  }

  if (gProvState == PROV_NOT_STARTED) {
    prefs.begin("iotmax", true);
    String saved = prefs.getString("api_key", "");
    prefs.end();

    if (saved.length() > 10) {
      strlcpy(gApiKey, saved.c_str(), sizeof(gApiKey));
      gProvState = PROV_ACTIVE;
      Serial.println("[PROV] API key dari NVS — siap kirim data");
      return;
    }

    Serial.println("[PROV] Belum punya key → register untuk QR claim...");
    provEnsureSecret(gProvSecret, sizeof(gProvSecret));
    gProvState = PROV_REGISTERING;
    provRegister(gProvSecret);
    if (provPollClaim(gProvSecret)) {
      gProvState = PROV_ACTIVE;
      return;
    }
    if (gProvState == PROV_NEEDS_RESET) return;
    gProvState = PROV_WAITING_QR;
    provPollCount = 0;
    tProvPoll = millis();
    provRequestPairingToken(gProvSecret);
    Serial.println("[PROV] Scan QR OLED di app, pilih project, lalu Claim device");
    return;
  }

  if (gProvState == PROV_ACTIVE || gProvState == PROV_NEEDS_RESET) return;
  unsigned long now = millis();
  unsigned long pairingInterval = gPairingPayload[0] == '\0' ? PAIRING_RETRY_MS : PAIRING_REFRESH_MS;
  if (now - tPairingRequest >= pairingInterval) {
    provRequestPairingToken(gProvSecret);
  }
  if (now - tProvPoll < PROV_POLL_MS) return;
  tProvPoll = now;
  provPollCount++;

  if (provPollClaim(gProvSecret)) {
    gProvState = PROV_ACTIVE;
    gPairingPayload[0] = '\0';
    return;
  }
  if (gProvState == PROV_NEEDS_RESET) {
    gPairingPayload[0] = '\0';
    return;
  }

  // Refresh presence setiap ±5 menit. Ini juga mengonsumsi authorization
  // scan-first yang dibuat app bila device baru menyala setelah QR di-scan.
  if (provPollCount % 10 == 0) provRegister(gProvSecret);
  Serial.printf("[PROV] Menunggu QR claim... (poll ke-%u)\n", provPollCount);
}
// ═══════════════════════════════════════════════
//  KIRIM TELEMETRY KE SERVER
// ═══════════════════════════════════════════════
// Menyusun JSON telemetri (dipakai HTTP dan MQTT). false = MAC belum valid.
bool buildTelemetry(char* out, size_t cap) {
  if (gMacFull == "00:00:00:00:00:00" || gMacFull.length() < 17) return false;

  StaticJsonDocument<768> doc;

  doc["mac_address"] = gMacFull;
  doc["firmware"]    = FIRMWARE_VER;

  doc["temperature"] = round(gTemp * 10.0f) / 10.0f;
  doc["humidity"]    = round(gHum  * 10.0f) / 10.0f;
  doc["vibration"]   = round(gVibDelta * 1000.0f) / 1000.0f;

  doc["acc_x"] = round(gAccX * 1000.0f) / 1000.0f;
  doc["acc_y"] = round(gAccY * 1000.0f) / 1000.0f;
  doc["acc_z"] = round(gAccZ * 1000.0f) / 1000.0f;

  doc["voltage"]       = round(gVoltage * 10.0f)   / 10.0f;
  doc["current_a"]     = round(gCurrent * 1000.0f) / 1000.0f;
  doc["power_w"]       = round(gPower   * 10.0f)   / 10.0f;
  doc["energy_kwh"]    = round(gEnergy * 1000.0f)  / 1000.0f;
  doc["frequency"]     = round(gFreq   * 10.0f)    / 10.0f;
  doc["power_factor"]  = round(gPF     * 100.0f)   / 100.0f;

  doc["rssi"]   = WiFi.RSSI();
  doc["heap"]   = (int)ESP.getFreeHeap();
  doc["uptime"] = (int)(millis() / 1000);

  serializeJson(doc, out, cap);
  return true;
}

void sendTelemetry() {
  if (!WiFi.isConnected()) return;

  char payload[768];
  if (!buildTelemetry(payload, sizeof(payload))) {
    Serial.println(F("[HTTP] Dibatalkan — MAC address tidak valid"));
    return;
  }

  Serial.print("[HTTP] Payload: ");
  Serial.println(payload);

  int code = httpPostWithRetry(SERVER_URL, payload, 12000, 2);
  lastHttpCode = code;

  if (code == 200) {
    httpOkCount++;
    keyFailCount = 0;   // key sehat — reset counter
    Serial.printf("[HTTP] OK Vib:%.3f V:%.1f I:%.3f P:%.1f T:%.1f\n",
                  gVibDelta, gVoltage, gCurrent, gPower, gTemp);
  } else {
    httpFailCount++;
    if (code == 401 || code == 403) {          // key ditolak server (bukan gangguan jaringan)
      keyFailCount++;
      Serial.printf("[PROV] Key ditolak server (%d/5)\n", keyFailCount);
      if (keyFailCount >= 5) {
        Serial.println(F("[PROV] Key dicabut → wipe & kembali ke provisioning"));
        prefs.begin("iotmax", false);
        prefs.remove("api_key");               // prov_secret DIBIARKAN — identitas tetap
        prefs.end();
        gApiKey[0] = '\0';                     // lazy provisioning (loop) ambil alih
        gProvState = PROV_NOT_STARTED;
        gPairingPayload[0] = '\0';
        provPollCount = 0;
        tProvPoll = 0;
        tPairingRequest = 0;
        Serial.println(F("[PROV] State di-reset; QR pairing akan diminta ulang"));
        keyFailCount = 0;
      }
    }
  }
}
// ═══════════════════════════════════════════════
//  KIRIM ALERT LANGSUNG
// ═══════════════════════════════════════════════
void sendAlert(const String& type, const String& msg,
               const String& sev, int val) {
  if (!WiFi.isConnected()) return;
  if (gMacFull == "00:00:00:00:00:00" || gMacFull.length() < 17) return;

  StaticJsonDocument<768> doc;

  doc["mac_address"] = gMacFull;
  doc["firmware"]    = FIRMWARE_VER;

  doc["temperature"] = round(gTemp * 10.0f) / 10.0f;
  doc["humidity"]    = round(gHum  * 10.0f) / 10.0f;
  doc["vibration"]   = round(gVibDelta * 1000.0f) / 1000.0f;

  doc["acc_x"] = round(gAccX * 1000.0f) / 1000.0f;
  doc["acc_y"] = round(gAccY * 1000.0f) / 1000.0f;
  doc["acc_z"] = round(gAccZ * 1000.0f) / 1000.0f;

  doc["voltage"]       = round(gVoltage * 10.0f)   / 10.0f;
  doc["current_a"]     = round(gCurrent * 1000.0f) / 1000.0f;
  doc["power_w"]       = round(gPower   * 10.0f)   / 10.0f;
  doc["energy_kwh"]    = round(gEnergy * 1000.0f)  / 1000.0f;
  doc["frequency"]     = round(gFreq   * 10.0f)    / 10.0f;
  doc["power_factor"]  = round(gPF     * 100.0f)   / 100.0f;

  doc["rssi"]   = WiFi.RSSI();
  doc["heap"]   = (int)ESP.getFreeHeap();
  doc["uptime"] = (int)(millis() / 1000);

  doc["alert_type"] = type;
  doc["alert_msg"]  = msg;
  doc["severity"]   = sev;

  char payload[768];
  serializeJson(doc, payload);

  Serial.print("[ALERT] Payload: ");
  Serial.println(payload);

  int code = httpPostWithRetry(SERVER_URL, payload, 10000, 1);

  if (code == 200) {
    totalAlertsSent++;
    Serial.printf("[ALERT] Sent: %s\n", type.c_str());
  } else {
    Serial.printf("[ALERT] FAIL %d: %s\n", code, type.c_str());
  }
}
// ═══════════════════════════════════════════════
//  RELAY CONTROL
// ═══════════════════════════════════════════════
void pollRelayCommands() {
  HTTPClient http;
  String url = String(RELAY_URL) + "?action=poll&mac=" + gMacFull
               + "&api_key=" + String(gApiKey);
  http.begin(url);
  http.setTimeout(3000);
  int code = http.GET();
  if (code != 200) { http.end(); return; }

  String body = http.getString();
  http.end();

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, body) != DeserializationError::Ok) return;
  if (!doc["ok"]) return;

  JsonArray cmds = doc["commands"];
  for (JsonObject cmd : cmds) {
    int relayId = cmd["relay_id"] | 0;
    int state   = cmd["state"]    | 0;
    if (relayId < 1 || relayId > RELAY_COUNT) continue;
    int pin = (relayId == 1) ? RELAY1_PIN : RELAY2_PIN;
    digitalWrite(pin, state ? LOW : HIGH);
    relayState[relayId] = (bool)state;
    Serial.printf("[RELAY] %d → %s\n", relayId, state ? "ON" : "OFF");
    sendRelayAck(relayId, state);
  }

  // ── Perintah PAKSA CEK OTA dari dashboard (v4.2) ──
  // Server menyisipkan "ota_check":true di respons poll saat
  // superadmin menekan tombol "Paksa Cek Sekarang" — device tidak
  // perlu menunggu jadwal 6 jam.
  if (doc["ota_check"] | false) {
    Serial.println(F("[OTA] Cek dipaksa dari dashboard"));
    tOTA = millis();   // reset jadwal 6 jam
    otaBootChecked = true;
    checkOTA();
  }
}

void sendRelayAck(int relayId, int state) {
  HTTPClient http;
  // PENTING: buffer HARUS cukup untuk menampung api_key (imx_live_ + 48 hex =
  // 57 char) yang DISALIN ke pool JSON. Dengan <128>, pool habis sebelum
  // api_key di-assign → api_key jadi null di body → server balas 401 →
  // pending relay tak pernah 0 → status stuck "Menunggu" & relay berulang.
  StaticJsonDocument<256> doc;
  doc["action"]   = "ack";
  doc["mac"]      = gMacFull;
  doc["relay_id"] = relayId;
  doc["state"]    = state;
  doc["api_key"]  = gApiKey;
  char buf[256]; serializeJson(doc, buf);
  http.begin(RELAY_URL);
  http.addHeader("Content-Type", "application/json");
  http.addHeader("X-API-KEY", gApiKey);   // konsisten dgn upload data & jaga-jaga
  http.setTimeout(3000);
  http.POST(buf);
  http.end();
}

// ═══════════════════════════════════════════════
//  OTA FIRMWARE UPDATE
//  Cek versi ke server; kalau beda → download .bin,
//  verifikasi MD5 (otomatis oleh HTTPUpdate via header
//  x-MD5), flash ke partisi OTA, reboot sendiri.
//  Gagal di tengah (koneksi putus/MD5 salah) → firmware
//  lama tetap jalan, tidak ada risiko brick.
// ═══════════════════════════════════════════════
void checkOTA() {
  Serial.println(F("[OTA] Cek update..."));

  // 1. Tanya server: ada versi baru?
  WiFiClientSecure client;
  client.setInsecure();
  HTTPClient http;
  String url = String(OTA_URL) + "?mac=" + gMacFull
             + "&fw=" + FIRMWARE_VER + "&api_key=" + String(gApiKey);
  if (!http.begin(client, url)) return;
  http.setTimeout(8000);
  int code = http.GET();
  if (code != 200) {
    Serial.printf("[OTA] Cek gagal HTTP %d\n", code);
    http.end();
    return;
  }
  String body = http.getString();
  http.end();

  StaticJsonDocument<512> doc;
  if (deserializeJson(doc, body) != DeserializationError::Ok) return;
  if (!doc["update"]) {
    Serial.println(F("[OTA] Sudah versi terbaru"));
    return;
  }

  const char* newVer = doc["version"] | "?";
  const char* fwUrl  = doc["url"]     | "";
  if (strlen(fwUrl) == 0) return;

  Serial.printf("[OTA] Update tersedia: v%s -> v%s\n", FIRMWARE_VER, newVer);
  Serial.println(F("[OTA] Download & flash dimulai — JANGAN matikan power!"));
  tftBoot("OTA update...");

  // 2. Progress ke OLED supaya kelihatan di lapangan
  httpUpdate.onProgress([](int cur, int total) {
    static int lastPct = -1;
    int pct = total > 0 ? (cur * 100 / total) : 0;
    if (pct != lastPct && pct % 10 == 0) {
      lastPct = pct;
      if (!displayOK) return;
      Serial.printf("[OTA] %d%%\n", pct);
      display.fillScreen(ST77XX_BLACK);
      display.setTextWrap(false);
      display.setTextColor(ST77XX_CYAN);
      display.setTextSize(2);
      display.setCursor(10, 12);
      display.println("OTA FIRMWARE");
      display.drawFastHLine(10, 40, 220, ST77XX_CYAN);
      display.setTextColor(ST77XX_WHITE);
      display.setCursor(10, 66);
      display.print("Progress ");
      display.print(pct);
      display.println("%");
      display.drawRect(9, 111, 222, 28, ST77XX_WHITE);
      display.fillRect(10, 112, (220 * pct) / 100, 26, ST77XX_GREEN);
      display.setTextColor(ST77XX_YELLOW);
      display.setCursor(10, 170);
      display.println("DO NOT POWER OFF");
    }
  });

  // 3. Eksekusi: download → verifikasi MD5 → flash → reboot otomatis
  WiFiClientSecure otaClient;
  otaClient.setInsecure();
  otaClient.setTimeout(30);
  httpUpdate.rebootOnUpdate(true);

  t_httpUpdate_return ret = httpUpdate.update(otaClient, String(fwUrl));

  // Baris di bawah hanya tercapai kalau update GAGAL
  // (sukses = device langsung reboot dengan firmware baru)
  switch (ret) {
    case HTTP_UPDATE_FAILED:
      Serial.printf("[OTA] GAGAL (%d): %s\n",
                    httpUpdate.getLastError(),
                    httpUpdate.getLastErrorString().c_str());
      tftBoot("OTA gagal, lanjut");
      break;
    case HTTP_UPDATE_NO_UPDATES:
      Serial.println(F("[OTA] Server bilang tidak ada update"));
      break;
    default:
      break;
  }
}

// ═══════════════════════════════════════════════
//  TFT ST7789 DISPLAY (240x240)
// ═══════════════════════════════════════════════
void tftBoot(const char* msg) {
  if (!displayOK) return;

  display.fillScreen(ST77XX_BLACK);
  display.setTextWrap(false);
  display.setTextColor(ST77XX_CYAN);
  display.setTextSize(2);
  display.setCursor(12, 12);
  display.print("IoTMaxhub v");
  display.println(FIRMWARE_VER);
  display.drawFastHLine(12, 42, 216, ST77XX_CYAN);

  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(2);
  display.setTextWrap(true);
  display.setCursor(12, 68);
  display.println(msg);
  display.setTextWrap(false);

  display.setTextColor(ST77XX_BLUE);
  display.setTextSize(1);
  display.setCursor(12, 218);
  display.println("ST7789 240x240");
}

void tftVibration() {
  if (!displayOK) return;

  float vib = gVibDelta;
  int barW = min(212, (int)(vib * 35));
  const char* label = (vib >= VIB_CRIT_THRESHOLD) ? "CRITICAL" :
                      (vib >= VIB_WARN_THRESHOLD) ? "WARNING" :
                      adxlOK ? "NORMAL" : "ADXL ERROR";
  uint16_t statusColor = (vib >= VIB_CRIT_THRESHOLD) ? ST77XX_RED :
                         (vib >= VIB_WARN_THRESHOLD) ? ST77XX_YELLOW :
                         adxlOK ? ST77XX_GREEN : ST77XX_RED;

  display.fillScreen(ST77XX_BLACK);
  display.setTextWrap(false);
  display.setTextColor(ST77XX_CYAN);
  display.setTextSize(2);
  display.setCursor(12, 10);
  display.print("VIBRATION");
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(1);
  display.setCursor(210, 14);
  display.print("1/3");

  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(3);
  display.setCursor(12, 52);
  display.print(vib, 2);
  display.setTextSize(1);
  display.print(" m/s2");

  display.setTextSize(2);
  display.setCursor(12, 99);
  display.print("X ");
  display.print(gAccX, 1);
  display.setCursor(12, 124);
  display.print("Y ");
  display.print(gAccY, 1);
  display.setCursor(122, 124);
  display.print("Z ");
  display.print(gAccZ, 1);

  display.drawRect(12, 157, 216, 22, ST77XX_WHITE);
  display.fillRect(14, 159, barW, 18, statusColor);
  display.setTextColor(statusColor);
  display.setTextSize(3);
  display.setCursor(12, 196);
  display.print(label);
}

void tftSensors() {
  if (!displayOK) return;

  display.fillScreen(ST77XX_BLACK);
  display.setTextWrap(false);
  display.setTextColor(ST77XX_CYAN);
  display.setTextSize(2);
  display.setCursor(12, 10);
  display.print("SENSORS");
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(1);
  display.setCursor(210, 14);
  display.print("2/3");

  display.setTextSize(2);
  display.setCursor(12, 52);
  display.setTextColor(ST77XX_YELLOW);
  display.print("TEMP ");
  display.setTextColor(ST77XX_WHITE);
  display.print(gTemp, 1);
  display.print(" C  ");
  display.print(gHum, 0);
  display.print("%");

  display.setCursor(12, 87);
  display.setTextColor(ST77XX_GREEN);
  display.print("VOLT ");
  display.setTextColor(ST77XX_WHITE);
  display.print(gVoltage, 1);
  display.print(" V");

  display.setCursor(12, 122);
  display.setTextColor(ST77XX_GREEN);
  display.print("CURR ");
  display.setTextColor(ST77XX_WHITE);
  display.print(gCurrent, 2);
  display.print(" A");

  display.setCursor(12, 157);
  display.setTextColor(ST77XX_GREEN);
  display.print("POWER ");
  display.setTextColor(ST77XX_WHITE);
  display.print(gPower, 0);
  display.print(" W");

  display.setCursor(12, 201);
  display.setTextSize(1);
  display.print("PZEM ");
  display.setTextColor(pzemOK ? ST77XX_GREEN : ST77XX_RED);
  display.print(pzemOK ? "OK" : "ERROR");
  display.setTextColor(ST77XX_WHITE);
  display.print("   API ");
  display.setTextColor(lastHttpCode == 200 ? ST77XX_GREEN : ST77XX_RED);
  display.print(lastHttpCode == 200 ? "OK" : String(lastHttpCode));
}

void tftPairingQR() {
  if (!displayOK || gPairingPayload[0] == '\0') return;

  // qrcode.c tidak memeriksa kapasitas sama sekali: bb_appendBits menulis
  // melewati batas data dan qrcode_initText tetap mengembalikan 0. Jadi
  // panjangnya harus diperiksa di sini, sebelum dikodekan.
  const size_t maxLen = 53;   // versi 3, ECC_LOW, mode BYTE
  const size_t len = strlen(gPairingPayload);
  if (len > maxLen) {
    Serial.printf("[QR] Payload %u byte, kapasitas %u - tidak dikodekan\n",
                  (unsigned)len, (unsigned)maxLen);
    tftBoot("QR payload too long");
    return;
  }

  QRCode qr;
  uint8_t modules[qrcode_getBufferSize(PAIRING_QR_VERSION)];
  if (qrcode_initText(&qr, modules, PAIRING_QR_VERSION, ECC_LOW, gPairingPayload) != 0) {
    tftBoot("QR encode error");
    return;
  }

  // Quiet zone ditekan dari 4 modul (anjuran spek) ke 2, supaya tiap modul
  // dapat piksel sebanyak mungkin — kamera ponsel butuh beberapa piksel per
  // modul untuk bisa memisahkannya. Sisa pembulatan di tepi menambah margin
  // terang lagi, dan bezel gelap panel ikut berfungsi sebagai pembatas.
  const uint8_t quietModules = 2;
  const int16_t scale = min(TFT_WIDTH, TFT_HEIGHT) / (qr.size + quietModules * 2);
  const int16_t qrPixels = qr.size * scale;
  const int16_t qrX = (TFT_WIDTH  - qrPixels) / 2;
  const int16_t qrY = (TFT_HEIGHT - qrPixels) / 2;

  // v3: 29 modul x 7 px = 203 px, margin terang 18 px (2,6 modul) tiap sisi.
  // Sebelumnya 29 x 6 = 174 px — modul sekarang 17% lebih besar.
  display.fillScreen(PAIRING_QR_BG);
  display.startWrite();
  for (uint8_t y = 0; y < qr.size; y++) {
    for (uint8_t x = 0; x < qr.size; x++) {
      if (qrcode_getModule(&qr, x, y)) {
        display.writeFillRect(qrX + x * scale, qrY + y * scale,
                              scale, scale, ST77XX_BLACK);
      }
    }
  }
  display.endWrite();
}

void tftNetwork() {
  if (!displayOK) return;

  bool sta = WiFi.isConnected();
  display.fillScreen(ST77XX_BLACK);
  display.setTextWrap(false);
  display.setTextColor(ST77XX_CYAN);
  display.setTextSize(2);
  display.setCursor(12, 10);
  display.print(sta ? "NETWORK" : apModeActive ? "AP CONFIG" : "NO WIFI");
  display.setTextColor(ST77XX_WHITE);
  display.setTextSize(1);
  display.setCursor(210, 14);
  display.print("3/3");

  display.setTextSize(2);
  if (sta) {
    display.setCursor(12, 55);
    display.print("IP");
    display.setTextSize(1);
    display.setCursor(12, 82);
    display.print(WiFi.localIP().toString());
    display.setTextSize(2);
    display.setCursor(12, 112);
    display.print("RSSI ");
    display.print(WiFi.RSSI());
    display.print(" dBm");
  } else if (apModeActive) {
    display.setCursor(12, 55);
    display.print(WiFi.softAPSSID());
    display.setCursor(12, 90);
    display.print(WiFi.softAPIP().toString());
    display.setCursor(12, 125);
    display.print("PASS");
    display.setTextSize(1);
    display.setCursor(12, 153);
    display.print(AP_PASS);
  } else {
    display.setCursor(12, 60);
    display.print("MAC");
    display.setTextSize(1);
    display.setCursor(12, 90);
    display.print(gMacFull);
  }

  display.setTextSize(1);
  display.setTextColor(ST77XX_GREEN);
  display.setCursor(12, 203);
  display.print("OK ");
  display.print(httpOkCount);
  display.setTextColor(ST77XX_RED);
  display.print("  FAIL ");
  display.print(httpFailCount);
  display.setTextColor(ST77XX_WHITE);
  display.setCursor(150, 203);
  display.print("UP ");
  display.print((int)(millis() / 1000));
  display.print("s");
}
// ═══════════════════════════════════════════════
//  WIFI + AP MODE / CAPTIVE PORTAL
// ═══════════════════════════════════════════════
String htmlEscape(String value) {
  value.replace("&", "&amp;");
  value.replace("<", "&lt;");
  value.replace(">", "&gt;");
  value.replace("\"", "&quot;");
  value.replace("'", "&#39;");
  return value;
}

bool isPortalMode() {
  return apModeActive;
}

void setupWebRoutes() {
  if (webRoutesReady) return;

  server.on("/", HTTP_GET, handlePortalRoot);
  server.on("/scan", HTTP_GET, handlePortalScan);
  server.on("/data", HTTP_GET, handlePortalData);
  server.on("/save", HTTP_POST, handlePortalSave);

  // URL umum yang biasa dipanggil Android/iOS/Windows untuk deteksi captive portal.
  server.on("/generate_204", HTTP_GET, handlePortalRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handlePortalRoot);
  server.on("/connecttest.txt", HTTP_GET, handlePortalRoot);
  server.on("/ncsi.txt", HTTP_GET, handlePortalRoot);
  server.on("/fwlink", HTTP_GET, handlePortalRoot);

  server.onNotFound([]() {
    if (isPortalMode()) {
      server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
      server.send(302, "text/plain", "");
    } else {
      server.send(404, "text/plain", "Not found");
    }
  });

  webRoutesReady = true;
}

void startAPMode(bool recoveryMode) {
  if (apModeActive) {
    recoveryAP = recoveryAP || recoveryMode;
    return;
  }

  // AP_STA memungkinkan portal tetap aktif sambil ESP32 mencoba reconnect ke router.
  WiFi.mode(WIFI_AP_STA);
  delay(100);

  String suffix = gMacFull;
  suffix.replace(":", "");
  if (suffix.length() > 4) suffix = suffix.substring(suffix.length() - 4);
  String apName = "IoTMaxhub_ATM_" + suffix;

  bool apOK = WiFi.softAP(apName.c_str(), AP_PASS);
  IPAddress apIP = WiFi.softAPIP();

  dnsServer.stop();
  dnsServer.start(DNS_PORT, "*", apIP);

  setupWebRoutes();
  if (!webServerStarted) {
    server.begin();
    webServerStarted = true;
  }

  apModeActive = apOK;
  recoveryAP = recoveryMode;

  Serial.println();
  Serial.println(F("[AP] ======================================="));
  Serial.printf("[AP] Mode       : %s\n", recoveryMode ? "RECOVERY" : "CONFIG");
  Serial.printf("[AP] Status     : %s\n", apOK ? "AKTIF" : "GAGAL");
  Serial.printf("[AP] SSID       : %s\n", apName.c_str());
  Serial.printf("[AP] Password   : %s\n", AP_PASS);
  Serial.printf("[AP] Portal     : http://%s/\n", apIP.toString().c_str());
  Serial.println(F("[AP] ======================================="));

  tftBoot(apOK ? "AP Mode: 192.168.4.1" : "AP Mode gagal");
}

void stopAPMode() {
  if (!apModeActive) return;

  dnsServer.stop();
  if (webServerStarted) {
    server.stop();
    webServerStarted = false;
  }
  WiFi.softAPdisconnect(true);
  delay(100);

  apModeActive = false;
  recoveryAP = false;

  // Pertahankan STA yang sudah terhubung.
  WiFi.mode(WIFI_STA);
  Serial.println(F("[AP] Recovery ditutup karena WiFi sudah terhubung"));
}

void handlePortalData() {
  StaticJsonDocument<768> doc;
  doc["mac_address"] = gMacFull;
  doc["ap_active"] = apModeActive;
  doc["ap_ssid"] = apModeActive ? WiFi.softAPSSID() : "";
  doc["ap_ip"] = apModeActive ? WiFi.softAPIP().toString() : "";
  doc["wifi_connected"] = WiFi.isConnected();
  doc["wifi_ssid"] = WiFi.isConnected() ? WiFi.SSID() : (gSSID.length() > 0 ? gSSID : gSSIDBackup);
  doc["wifi_profile"] = activeWiFiProfileName();
  doc["wifi_primary_configured"] = gSSID.length() > 0;
  doc["wifi_backup_configured"] = gSSIDBackup.length() > 0;
  doc["wifi_ip"] = WiFi.isConnected() ? WiFi.localIP().toString() : "";
  doc["rssi"] = WiFi.isConnected() ? WiFi.RSSI() : 0;
  doc["provisioning_status"] = provisioningStateName();
  doc["temperature"] = gTemp;
  doc["humidity"] = gHum;
  doc["vibration"] = gVibDelta;
  doc["voltage"] = gVoltage;
  doc["current"] = gCurrent;
  doc["power"] = gPower;
  doc["adxl_ok"] = adxlOK;
  doc["pzem_ok"] = pzemOK;
  doc["uptime"] = (uint32_t)(millis() / 1000UL);

  String json;
  serializeJson(doc, json);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", json);
}

void handlePortalRoot() {
  String html;
  html.reserve(7800);

  String apSSID = apModeActive ? WiFi.softAPSSID() : "Tidak aktif";
  String staStatus = WiFi.isConnected() ? "Terhubung" : "Belum terhubung";
  String staIP = WiFi.isConnected() ? WiFi.localIP().toString() : "-";
  String wifiProfile = activeWiFiProfileName();

  html += F("<!DOCTYPE html><html lang='id'><head><meta charset='UTF-8'>");
  html += F("<meta name='viewport' content='width=device-width,initial-scale=1'>");
  html += F("<title>IoTMaxhub ATM Setup</title><style>");
  html += F("*{box-sizing:border-box}body{margin:0;background:#080c18;color:#eef2ff;font-family:Arial,sans-serif;padding:18px}");
  html += F(".wrap{max-width:520px;margin:auto}.card{background:#101828;border:1px solid rgba(0,212,255,.22);border-radius:14px;padding:18px;margin-bottom:14px}");
  html += F("h1{font-size:1.3rem;color:#00d4ff;margin:0 0 5px}.sub{color:#8f9bb3;font-size:.85rem;margin-bottom:16px}");
  html += F(".grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}.item{background:#0d1220;border-radius:8px;padding:10px}");
  html += F(".lbl{color:#8f9bb3;font-size:.68rem;text-transform:uppercase}.val{font-weight:700;margin-top:4px;word-break:break-all}");
  html += F("label{display:block;color:#a8b2c8;font-size:.78rem;margin:12px 0 5px}");
  html += F("select,input{width:100%;padding:12px;background:#0d1220;color:#eef2ff;border:1px solid rgba(0,212,255,.32);border-radius:8px}");
  html += F("button,.btn{display:block;width:100%;padding:12px;border:0;border-radius:8px;background:#00d4ff;color:#06101b;font-weight:700;margin-top:14px;text-decoration:none;text-align:center}");
  html += F(".ok{color:#00ff88}.bad{color:#ff4466}.note{color:#ffbf47;font-size:.78rem;line-height:1.5}.foot{color:#758097;font-size:.72rem;text-align:center}");
  html += F("</style></head><body><div class='wrap'>");

  html += F("<div class='card'><h1>IoTMaxhub ATM</h1><div class='sub'>WiFi Configuration Portal</div><div class='grid'>");
  html += "<div class='item'><div class='lbl'>AP SSID</div><div class='val'>" + htmlEscape(apSSID) + "</div></div>";
  html += "<div class='item'><div class='lbl'>AP IP</div><div class='val'>" + WiFi.softAPIP().toString() + "</div></div>";
  html += "<div class='item'><div class='lbl'>WiFi</div><div class='val " + String(WiFi.isConnected() ? "ok" : "bad") + "'>" + staStatus + "</div></div>";
  html += "<div class='item'><div class='lbl'>STA IP</div><div class='val'>" + staIP + "</div></div>";
  html += "<div class='item'><div class='lbl'>Profil Aktif</div><div class='val'>" + wifiProfile + "</div></div>";
  html += "<div class='item'><div class='lbl'>QR Claim</div><div class='val'>" + String(provisioningStateName()) + "</div></div>";
  html += F("</div></div>");

  html += F("<div class='card'><form action='/save' method='POST'>");
  html += F("<label>WiFi Primary / SSID</label><input id='ssid' name='ssid' type='text' placeholder='WiFi utama 2.4 GHz' required>");
  html += F("<label>Password WiFi Primary</label><input name='pass' type='password' placeholder='Kosongkan bila jaringan terbuka'>");
  html += F("<label>WiFi Backup / SSID (opsional)</label><input id='ssid2' name='ssid2' type='text' placeholder='WiFi cadangan 2.4 GHz'>");
  html += F("<label>Password WiFi Backup</label><input name='pass2' type='password' placeholder='Kosongkan bila tidak digunakan/terbuka'>");
  html += F("<label>Password Admin Device</label><input name='adminpass' type='password' placeholder='Password admin' required>");
  html += F("<button type='submit'>Simpan dan Sambungkan</button></form>");
  html += F("<a class='btn' href='/scan'>Cari Jaringan WiFi</a>");
  html += F("<p class='note'>Gunakan jaringan 2.4 GHz. Setelah konfigurasi disimpan, ESP32 akan restart otomatis.</p></div>");

  html += "<div class='foot'>MAC: " + htmlEscape(gMacFull) + " &bull; Firmware " + String(FIRMWARE_VER) + "</div>";
  html += F("</div></body></html>");

  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", html);
}

void handlePortalScan() {
  // Tetap AP_STA agar hotspot tidak mati saat proses scan.
  WiFi.mode(WIFI_AP_STA);
  delay(100);
  int count = WiFi.scanNetworks(false, true);

  String html;
  html.reserve(9500);
  html += F("<!DOCTYPE html><html lang='id'><head><meta charset='UTF-8'>");
  html += F("<meta name='viewport' content='width=device-width,initial-scale=1'><title>Scan WiFi</title>");
  html += F("<style>*{box-sizing:border-box}body{margin:0;background:#080c18;color:#eef2ff;font-family:Arial;padding:18px}.card{max-width:520px;margin:auto;background:#101828;border:1px solid rgba(0,212,255,.22);border-radius:14px;padding:18px}h2{color:#00d4ff}label{display:block;color:#a8b2c8;font-size:.78rem;margin:12px 0 5px}select,input{width:100%;padding:12px;background:#0d1220;color:#eef2ff;border:1px solid rgba(0,212,255,.32);border-radius:8px}button,a{display:block;width:100%;padding:12px;border:0;border-radius:8px;background:#00d4ff;color:#06101b;font-weight:700;margin-top:14px;text-decoration:none;text-align:center}.muted{color:#8f9bb3;font-size:.8rem}</style></head><body><div class='card'>");
  html += "<h2>Jaringan WiFi 2.4 GHz</h2><p class='muted'>Ditemukan " + String(count > 0 ? count : 0) + " jaringan.</p>";
  String networkOptions = "<option value=''>-- Pilih jaringan --</option>";
  for (int i = 0; i < count; i++) {
    String ssid = WiFi.SSID(i);
    if (ssid.length() == 0) continue;
    String safeSSID = htmlEscape(ssid);
    networkOptions += "<option value='" + safeSSID + "'>" + safeSSID + " (" + String(WiFi.RSSI(i)) + " dBm";
    networkOptions += (WiFi.encryptionType(i) == WIFI_AUTH_OPEN) ? ", terbuka)</option>" : ", terkunci)</option>";
  }

  html += F("<form action='/save' method='POST'><label>Pilih WiFi Primary</label><select onchange=\"document.getElementById('ssid').value=this.value\">");
  html += networkOptions;
  html += F("</select><label>SSID Primary</label><input id='ssid' name='ssid' type='text' required>");
  html += F("<label>Password Primary</label><input name='pass' type='password'>");
  html += F("<label>Pilih WiFi Backup (opsional)</label><select onchange=\"document.getElementById('ssid2').value=this.value\">");
  html += networkOptions;
  html += F("</select><label>SSID Backup</label><input id='ssid2' name='ssid2' type='text'>");
  html += F("<label>Password Backup</label><input name='pass2' type='password'>");
  html += F("<label>Password Admin Device</label><input name='adminpass' type='password' required>");
  html += F("<button type='submit'>Simpan dan Sambungkan</button></form><a href='/'>Kembali</a></div></body></html>");

  WiFi.scanDelete();
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html", html);
}

void handlePortalSave() {
  if (!server.hasArg("ssid") || !server.hasArg("pass") || !server.hasArg("adminpass")) {
    server.send(400, "text/plain", "Parameter tidak lengkap");
    return;
  }

  if (server.arg("adminpass") != CONFIG_ADMIN_PASS) {
    server.send(403, "text/html",
      "<html><body style='background:#080c18;color:#eef2ff;font-family:Arial;padding:30px'>"
      "<h2 style='color:#ff4466'>Password admin salah</h2><a style='color:#00d4ff' href='/'>Kembali</a></body></html>");
    return;
  }

  String newSSID = server.arg("ssid");
  String newPass = server.arg("pass");
  String newSSIDBackup = server.hasArg("ssid2") ? server.arg("ssid2") : "";
  String newPassBackup = server.hasArg("pass2") ? server.arg("pass2") : "";
  newSSID.trim();
  newSSIDBackup.trim();

  bool primaryInvalid = newSSID.length() == 0 || newSSID.length() > 32 || newPass.length() > 63;
  bool backupInvalid = newSSIDBackup.length() > 32 || newPassBackup.length() > 63 ||
                       (newSSIDBackup.length() == 0 && newPassBackup.length() > 0) ||
                       (newSSIDBackup.length() > 0 && newSSIDBackup == newSSID);
  if (primaryInvalid || backupInvalid) {
    server.send(400, "text/plain", "Konfigurasi WiFi primary/backup tidak valid atau SSID sama");
    return;
  }

  prefs.begin("wifi", false);
  prefs.putString("ssid", newSSID);
  prefs.putString("pass", newPass);
  prefs.putString("ssid2", newSSIDBackup);
  prefs.putString("pass2", newPassBackup);
  prefs.end();

  gSSID = newSSID;
  gPass = newPass;
  gSSIDBackup = newSSIDBackup;
  gPassBackup = newPassBackup;

  server.send(200, "text/html",
    "<html><body style='background:#080c18;color:#eef2ff;font-family:Arial;padding:30px'>"
    "<h2 style='color:#00ff88'>Konfigurasi tersimpan</h2>"
    "<p>ESP32 akan restart, mencoba WiFi primary, lalu otomatis beralih ke backup bila diperlukan.</p></body></html>");

  Serial.printf("[AP] WiFi disimpan: primary=%s backup=%s, restart...\n",
                gSSID.c_str(), gSSIDBackup.length() > 0 ? gSSIDBackup.c_str() : "-");
  delay(1500);
  ESP.restart();
}

bool hasWiFiCredentials() {
  return gSSID.length() > 0 || gSSIDBackup.length() > 0;
}

const char* activeWiFiProfileName() {
  if (activeWiFiProfile == 1 || (activeWiFiProfile == 0 && attemptedWiFiProfile == 1)) return "PRIMARY";
  if (activeWiFiProfile == 2 || (activeWiFiProfile == 0 && attemptedWiFiProfile == 2)) return "BACKUP";
  return "NONE";
}

bool beginWiFiProfile(uint8_t profile, bool waitForConnection) {
  const String* ssid = profile == 2 ? &gSSIDBackup : &gSSID;
  const String* pass = profile == 2 ? &gPassBackup : &gPass;
  if (ssid->length() == 0) return false;

  WiFi.mode(apModeActive ? WIFI_AP_STA : WIFI_STA);
  WiFi.setHostname("iotmaxhub-node");
  WiFi.setAutoReconnect(true);
  WiFi.persistent(false);
  WiFi.disconnect(false, false);
  delay(100);

  attemptedWiFiProfile = profile;
  Serial.printf("[WiFi] Mencoba profil %s, SSID=%s%s",
                profile == 2 ? "BACKUP" : "PRIMARY", ssid->c_str(),
                waitForConnection ? "" : " (non-blocking)\n");
  WiFi.begin(ssid->c_str(), pass->c_str());
  if (!waitForConnection) return true;

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < WIFI_CONNECT_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println(F(" gagal"));
    return false;
  }

  activeWiFiProfile = profile;
  Serial.printf("\n[WiFi] %s OK IP:%s RSSI:%d dBm\n",
                activeWiFiProfileName(), WiFi.localIP().toString().c_str(), WiFi.RSSI());
  return true;
}

void tryNextWiFiProfile() {
  uint8_t current = activeWiFiProfile != 0 ? activeWiFiProfile : attemptedWiFiProfile;
  uint8_t next = 1;
  if (current == 1 && gSSIDBackup.length() > 0) next = 2;
  else if (current == 2 && gSSID.length() > 0) next = 1;
  else if (gSSID.length() == 0 && gSSIDBackup.length() > 0) next = 2;

  Serial.printf("[WiFi] Rotasi failover dari %s ke %s\n",
                activeWiFiProfileName(), next == 2 ? "BACKUP" : "PRIMARY");
  activeWiFiProfile = 0;
  beginWiFiProfile(next, false);
}

void connectWiFi() {
  activeWiFiProfile = 0;
  attemptedWiFiProfile = 0;

  bool connected = false;
  if (gSSID.length() > 0) connected = beginWiFiProfile(1, true);
  if (!connected && gSSIDBackup.length() > 0) connected = beginWiFiProfile(2, true);

  if (connected) {
    if (apModeActive && recoveryAP) stopAPMode();
  } else {
    Serial.println(F("[WiFi] Primary dan backup gagal — AP Mode akan tersedia"));
  }
}
