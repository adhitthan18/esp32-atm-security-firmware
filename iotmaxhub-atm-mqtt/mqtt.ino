// ═══════════════════════════════════════════════════════════════
//  MQTT — telemetri realtime + perintah relay seketika
//  lewat MQTT-over-WebSocket di port 443 (lihat MQTT_URL).
//
//  Topik (MAC huruf besar dengan titik dua):
//    iot/<MAC>/telemetry  -> JSON sama persis dengan POST /api/device/data, QoS 0
//    iot/<MAC>/ack        -> {"relay_id","state","command_id"}, QoS 1
//    iot/<MAC>/status     -> "online" saat tersambung, "offline" lewat Last Will
//    iot/<MAC>/cmd        <- {"relay_id","state","command_id"} atau {"reconnect":1}
//
//  Login: username = MAC, password = API key hasil klaim QR (sama dengan
//  X-API-KEY di HTTP). Client ID wajib memuat MAC - server menolak yang tidak.
//
//  Jalur HTTP TIDAK dihapus: selama MQTT tersambung, telemetri HTTP dijarangkan
//  menjadi tiap 15 menit (sekaligus memeriksa key masih berlaku), alert tetap
//  lewat HTTP seketika, dan poll relay tiap 30 detik (jadwal, OTA, perintah yang
//  terlewat). Begitu MQTT putus, semuanya kembali ke interval semula.
//
//  DNS HANYA DARI loop(). Resolver lwIP di core 3.x bisa panic kalau task
//  esp-mqtt dan HTTP di loop() menanyakan nama bersamaan. Karena itu reconnect
//  otomatis esp-mqtt dimatikan, nama host di-resolve loop() lebih dulu, dan
//  selama esp-mqtt menyambung HTTP menunggu (mqttNetworkBusy).
// ═══════════════════════════════════════════════════════════════

#include "mqtt_client.h"
#include "esp_crt_bundle.h"

struct MqttCmd {
  int relayId;
  int state;
  char commandId[48];
};

static esp_mqtt_client_handle_t gMqtt = nullptr;
static volatile bool gMqttConnected = false;
static volatile bool gMqttPollRequest = false;
static volatile bool gMqttReconnectRequest = false;
static QueueHandle_t gMqttCmdQueue = nullptr;
static char gMqttTopicCmd[48];
static char gMqttTopicAck[48];
static char gMqttTopicStatus[48];
static char gMqttTopicTele[56];
static char gMqttClientId[40];
static char gMqttUser[24];
static char gMqttKeyInUse[64];

enum MqttPhase { MQTT_IDLE, MQTT_CONNECTING, MQTT_UP };
static MqttPhase gMqttPhase = MQTT_IDLE;
static volatile bool gMqttFailed = false;
static unsigned long gMqttNextTry = 0;
static unsigned long gMqttBackoff = 5000UL;
static unsigned long gMqttConnectStart = 0;
#define MQTT_BACKOFF_MAX_MS (60UL * 1000UL)
#define MQTT_CONNECT_MAX_MS (20UL * 1000UL)

bool mqttEnabled() { return strlen(MQTT_URL) > 0; }
bool mqttConnected() { return gMqttConnected; }

// Benar selama esp-mqtt sedang resolve/TLS/handshake. HTTP harus menunggu.
bool mqttNetworkBusy() { return gMqttPhase == MQTT_CONNECTING && !gMqttConnected; }

// "wss://host:443/mqtt" -> "host"
static String mqttHost() {
  String u = MQTT_URL;
  int a = u.indexOf("://");
  a = a < 0 ? 0 : a + 3;
  int b = a;
  while (b < (int)u.length() && u[b] != ':' && u[b] != '/') b++;
  return u.substring(a, b);
}

// Benar satu kali setiap kali MQTT (ulang) tersambung: perintah yang dikirim
// selama terputus tidak diulang broker, jadi satu poll HTTP menjemputnya.
bool mqttTakePollRequest() {
  if (!gMqttPollRequest) return false;
  gMqttPollRequest = false;
  return true;
}

static void mqttEvent(void *, esp_event_base_t, int32_t eventId, void *eventData) {
  esp_mqtt_event_handle_t e = (esp_mqtt_event_handle_t)eventData;
  switch ((esp_mqtt_event_id_t)eventId) {
    case MQTT_EVENT_CONNECTED:
      gMqttConnected = true;
      gMqttPollRequest = true;
      esp_mqtt_client_subscribe(gMqtt, gMqttTopicCmd, 1);
      esp_mqtt_client_publish(gMqtt, gMqttTopicStatus, "online", 0, 1, 1);
      break;

    case MQTT_EVENT_DISCONNECTED:
    case MQTT_EVENT_ERROR:
      // Klien tidak disentuh dari task esp-mqtt; loop() yang membereskan.
      gMqttConnected = false;
      gMqttFailed = true;
      break;

    case MQTT_EVENT_DATA: {
      if (e->topic_len != (int)strlen(gMqttTopicCmd) || strncmp(e->topic, gMqttTopicCmd, e->topic_len) != 0) break;
      if (e->total_data_len != e->data_len) break;  // pesan terpotong bukan milik kita

      StaticJsonDocument<256> doc;
      if (deserializeJson(doc, e->data, e->data_len)) break;
      if (doc.containsKey("reconnect")) {  // key dicabut/sesi basi: login ulang
        gMqttReconnectRequest = true;
        break;
      }
      MqttCmd cmd{doc["relay_id"] | 0, doc["state"] | 0, ""};
      strlcpy(cmd.commandId, doc["command_id"] | "", sizeof(cmd.commandId));
      if (gMqttCmdQueue) xQueueSend(gMqttCmdQueue, &cmd, 0);
      break;
    }

    default:
      break;
  }
}

static void mqttStart() {
  if (!mqttEnabled() || gMqtt || gApiKey[0] == '\0' || gMacFull.length() < 17) return;

  snprintf(gMqttTopicCmd, sizeof(gMqttTopicCmd), "iot/%s/cmd", gMacFull.c_str());
  snprintf(gMqttTopicAck, sizeof(gMqttTopicAck), "iot/%s/ack", gMacFull.c_str());
  snprintf(gMqttTopicStatus, sizeof(gMqttTopicStatus), "iot/%s/status", gMacFull.c_str());
  snprintf(gMqttTopicTele, sizeof(gMqttTopicTele), "iot/%s/telemetry", gMacFull.c_str());
  snprintf(gMqttClientId, sizeof(gMqttClientId), "atm-%s", gMacFull.c_str());
  strlcpy(gMqttUser, gMacFull.c_str(), sizeof(gMqttUser));
  strlcpy(gMqttKeyInUse, gApiKey, sizeof(gMqttKeyInUse));
  if (!gMqttCmdQueue) gMqttCmdQueue = xQueueCreate(8, sizeof(MqttCmd));

  esp_mqtt_client_config_t cfg = {};
  cfg.broker.address.uri = MQTT_URL;
  // Sertifikat server diperiksa dengan bundel CA bawaan core, jadi server palsu
  // tidak bisa menyamar di jalur MQTT.
  cfg.broker.verification.crt_bundle_attach = esp_crt_bundle_attach;
  cfg.credentials.client_id = gMqttClientId;
  cfg.credentials.username = gMqttUser;
  cfg.credentials.authentication.password = gMqttKeyInUse;
  cfg.session.keepalive = 30;  // di bawah batas 100 detik Cloudflare
  cfg.session.last_will.topic = gMqttTopicStatus;
  cfg.session.last_will.msg = "offline";
  cfg.session.last_will.qos = 1;
  cfg.session.last_will.retain = 1;
  cfg.network.disable_auto_reconnect = true;  // reconnect diatur loop()
  cfg.network.timeout_ms = 10000;

  gMqtt = esp_mqtt_client_init(&cfg);
  if (!gMqtt) {
    Serial.println(F("[MQTT] gagal membuat klien"));
    return;
  }
  gMqttFailed = false;
  esp_mqtt_client_register_event(gMqtt, MQTT_EVENT_ANY, mqttEvent, nullptr);
  if (esp_mqtt_client_start(gMqtt) != ESP_OK) {
    esp_mqtt_client_destroy(gMqtt);
    gMqtt = nullptr;
    Serial.println(F("[MQTT] gagal memulai klien"));
    return;
  }
  gMqttPhase = MQTT_CONNECTING;
  gMqttConnectStart = millis();
  Serial.printf("[MQTT] menyambung ke %s\n", MQTT_URL);
}

void mqttStop() {
  if (!gMqtt) return;
  esp_mqtt_client_stop(gMqtt);
  esp_mqtt_client_destroy(gMqtt);
  gMqtt = nullptr;
  gMqttConnected = false;
  gMqttPhase = MQTT_IDLE;
  Serial.println(F("[MQTT] dihentikan"));
}

static void mqttRetryLater(const char *alasan) {
  mqttStop();
  gMqttFailed = false;
  gMqttNextTry = millis() + gMqttBackoff;
  Serial.printf("[MQTT] %s - coba lagi %lu dtk\n", alasan, gMqttBackoff / 1000UL);
  gMqttBackoff = min(gMqttBackoff * 2, MQTT_BACKOFF_MAX_MS);
}

// Relay 1/2 aktif LOW, sama dengan pollRelayCommands().
// Argumen dipisah (bukan MqttCmd): prototipe buatan Arduino diletakkan di atas
// definisi struct, jadi tipe itu belum dikenal di sana.
static void mqttApplyRelay(int relayId, int state, const char *commandId) {
  if (relayId < 1 || relayId > RELAY_COUNT) return;
  const int pin = (relayId == 1) ? RELAY1_PIN : RELAY2_PIN;
  digitalWrite(pin, state ? LOW : HIGH);
  relayState[relayId] = state != 0;
  Serial.printf("[MQTT] relay %d -> %s\n", relayId, state ? "ON" : "OFF");

  StaticJsonDocument<160> doc;
  doc["relay_id"] = relayId;
  doc["state"] = state ? 1 : 0;
  if (commandId[0]) doc["command_id"] = commandId;
  char ack[160];
  serializeJson(doc, ack, sizeof(ack));
  const int msgId = (gMqtt && gMqttConnected) ? esp_mqtt_client_publish(gMqtt, gMqttTopicAck, ack, 0, 1, 0) : -1;
  // Gagal masuk antrean kirim -> ack lewat HTTP seperti jalur poll. Selama
  // esp-mqtt menyambung HTTP dilarang (DNS); perintahnya tetap pending di
  // server dan poll HTTP berikutnya mengulang + meng-ack-nya.
  if (msgId < 0 && !mqttNetworkBusy()) sendRelayAck(relayId, state ? 1 : 0);
}

// Telemetri realtime. false = tidak terkirim (pemanggil boleh pakai HTTP).
bool mqttPublishTelemetry(const char *payload) {
  if (!gMqtt || !gMqttConnected) return false;
  return esp_mqtt_client_publish(gMqtt, gMqttTopicTele, payload, 0, 0, 0) >= 0;
}

// Dipanggil tiap loop() selama WiFi tersambung dan perangkat punya API key.
void mqttTick() {
  if (!mqttEnabled()) return;

  // Kunci berganti (klaim ulang) atau server meminta login ulang.
  if (gMqtt && strcmp(gMqttKeyInUse, gApiKey) != 0) mqttStop();
  if (gMqttReconnectRequest) {
    gMqttReconnectRequest = false;
    mqttStop();
    gMqttNextTry = millis() + 2000UL;
    Serial.println(F("[MQTT] server meminta login ulang"));
  }

  if (gMqtt) {
    if (gMqttConnected && gMqttPhase != MQTT_UP) {
      gMqttPhase = MQTT_UP;
      gMqttBackoff = 5000UL;
      Serial.println(F("[MQTT] tersambung"));
    }
    if (gMqttFailed) {
      mqttRetryLater(gMqttPhase == MQTT_UP ? "putus" : "gagal tersambung");
    } else if (gMqttPhase == MQTT_CONNECTING && millis() - gMqttConnectStart > MQTT_CONNECT_MAX_MS) {
      mqttRetryLater("terlalu lama menyambung");
    }
  }

  if (!gMqtt && gApiKey[0] != '\0' && (long)(millis() - gMqttNextTry) >= 0 && WiFi.isConnected()) {
    IPAddress ip;
    const String host = mqttHost();
    if (!WiFi.hostByName(host.c_str(), ip) || ip == IPAddress((uint32_t)0)) {
      gMqttNextTry = millis() + gMqttBackoff;
      Serial.printf("[MQTT] DNS %s gagal - coba lagi %lu dtk\n", host.c_str(), gMqttBackoff / 1000UL);
      gMqttBackoff = min(gMqttBackoff * 2, MQTT_BACKOFF_MAX_MS);
    } else {
      mqttStart();
    }
  }

  MqttCmd cmd;
  while (gMqttCmdQueue && xQueueReceive(gMqttCmdQueue, &cmd, 0) == pdTRUE) {
    mqttApplyRelay(cmd.relayId, cmd.state, cmd.commandId);
  }
}
