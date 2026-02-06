#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Preferences.h>
#include <ArduinoOTA.h>
#include "DHT.h"

#define DHTPIN 19
#define DHTTYPE DHT11

// Set to your master Wi-Fi channel for best reliability.
// Leave 0 to skip forcing channel.
#define WIFI_CHANNEL 0

// Defaults (can be overridden by master)
#define DEFAULT_SEND_INTERVAL_MS 5000
#define DEFAULT_DHT_RETRY 2
#define DEFAULT_TX_POWER 78

// OTA Wi-Fi (only used when OTA mode is enabled)
#define OTA_SSID ""
#define OTA_PASS ""

DHT dht(DHTPIN, DHTTYPE);

Preferences prefs;

uint32_t sendIntervalMs = DEFAULT_SEND_INTERVAL_MS;
uint8_t dhtRetry = DEFAULT_DHT_RETRY;
int8_t txPower = DEFAULT_TX_POWER;
bool otaMode = false;
bool pendingRestart = false;
String otaSsid = "";
String otaPass = "";

uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

typedef struct {
  uint8_t type;
  uint8_t cmd;
  uint16_t reserved;
  uint32_t value;
} SensorCmdPacket;

typedef struct {
  uint8_t type;
  uint8_t cmd;
  char value[64];
} SensorCmdStrPacket;

typedef struct {
  uint8_t type;
  uint8_t cmd;
  uint8_t status;
  uint8_t reserved;
  uint32_t value;
} SensorAckPacket;

typedef struct {
  uint8_t type;
  uint8_t ota;
  uint8_t retry;
  uint8_t tx;
  uint32_t interval;
  uint32_t ip;
} SensorStatusPacket;

enum {
  CMD_SET_INTERVAL = 1,
  CMD_SET_RETRY = 2,
  CMD_SET_TXPOWER = 3,
  CMD_SET_OTA = 4,
  CMD_SET_WIFI_SSID = 5,
  CMD_SET_WIFI_PASS = 6,
  MSG_TYPE_CMD = 2,
  MSG_TYPE_ACK = 3,
  MSG_TYPE_STATUS = 4
};

struct SensorData {
  uint8_t mac[6];
  float temp;
  float hum;
};

SensorData data;
unsigned long lastSend = 0;
unsigned long lastStatus = 0;

float lastTemp = NAN;
float lastHum = NAN;

void saveConfig(){
  prefs.begin("sensor", false);
  prefs.putULong("int", sendIntervalMs);
  prefs.putUChar("ret", dhtRetry);
  prefs.putChar("txp", txPower);
  prefs.putBool("ota", otaMode);
  prefs.putString("ssid", otaSsid);
  prefs.putString("pass", otaPass);
  prefs.end();
}

void loadConfig(){
  prefs.begin("sensor", false);
  sendIntervalMs = prefs.getULong("int", DEFAULT_SEND_INTERVAL_MS);
  dhtRetry = prefs.getUChar("ret", DEFAULT_DHT_RETRY);
  txPower = prefs.getChar("txp", DEFAULT_TX_POWER);
  otaMode = prefs.getBool("ota", false);
  otaSsid = prefs.getString("ssid", "");
  otaPass = prefs.getString("pass", "");
  prefs.end();
}

void sendAck(const uint8_t *mac, uint8_t cmd, uint8_t status, uint32_t value){
  SensorAckPacket ack = { MSG_TYPE_ACK, cmd, status, 0, value };
  esp_now_send(mac, (uint8_t*)&ack, sizeof(ack));
}

void sendStatus() {
  SensorStatusPacket st = {};
  st.type = MSG_TYPE_STATUS;
  st.ota = otaMode ? 1 : 0;
  st.retry = dhtRetry;
  st.tx = (uint8_t)txPower;
  st.interval = sendIntervalMs;
  if (WiFi.status() == WL_CONNECTED) {
    st.ip = (uint32_t) WiFi.localIP();
  } else {
    st.ip = 0;
  }
  esp_now_send(broadcastAddress, (uint8_t*)&st, sizeof(st));
}

void onReceive(const uint8_t *mac, const uint8_t *data, int len) {
  if (len == sizeof(SensorCmdStrPacket)) {
    SensorCmdStrPacket sc;
    memcpy(&sc, data, sizeof(sc));
    if (sc.type != MSG_TYPE_CMD) return;
    if (sc.cmd == CMD_SET_WIFI_SSID) {
      otaSsid = String(sc.value);
      saveConfig();
      sendAck(mac, sc.cmd, 0, otaSsid.length());
      return;
    }
    if (sc.cmd == CMD_SET_WIFI_PASS) {
      otaPass = String(sc.value);
      saveConfig();
      sendAck(mac, sc.cmd, 0, otaPass.length());
      return;
    }
  }

  if (len != sizeof(SensorCmdPacket)) return;
  SensorCmdPacket cmd;
  memcpy(&cmd, data, sizeof(cmd));
  if (cmd.type != MSG_TYPE_CMD) return;

  switch (cmd.cmd) {
    case CMD_SET_INTERVAL:
      sendIntervalMs = cmd.value < 1000 ? 1000 : cmd.value;
      saveConfig();
      sendAck(mac, cmd.cmd, 0, sendIntervalMs);
      sendStatus();
      break;
    case CMD_SET_RETRY:
      dhtRetry = (cmd.value > 5) ? 5 : (uint8_t)cmd.value;
      saveConfig();
      sendAck(mac, cmd.cmd, 0, dhtRetry);
      sendStatus();
      break;
    case CMD_SET_TXPOWER:
      txPower = (cmd.value < 8) ? 8 : (cmd.value > 78 ? 78 : (int8_t)cmd.value);
      esp_wifi_set_max_tx_power(txPower);
      saveConfig();
      sendAck(mac, cmd.cmd, 0, txPower);
      sendStatus();
      break;
    case CMD_SET_OTA:
      otaMode = cmd.value ? true : false;
      saveConfig();
      sendAck(mac, cmd.cmd, 0, otaMode ? 1 : 0);
      sendStatus();
      pendingRestart = true;
      break;
    default:
      sendAck(mac, cmd.cmd, 1, 0);
      break;
  }
}

void onSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("ESP-NOW send: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "OK" : "FAIL");
}

bool isValid(float t, float h) {
  if (isnan(t) || isnan(h)) return false;
  if (t < -20.0f || t > 80.0f) return false;
  if (h < 0.0f || h > 100.0f) return false;
  return true;
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);

  loadConfig();
  esp_wifi_set_max_tx_power(txPower);

  if (WIFI_CHANNEL > 0) {
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  }

  WiFi.macAddress(data.mac);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW error");
    return;
  }

  esp_now_register_send_cb(onSent);
  esp_now_register_recv_cb(onReceive);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = (WIFI_CHANNEL > 0) ? WIFI_CHANNEL : 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  dht.begin();
  delay(1500); // let DHT settle on first boot

  if (otaMode && otaSsid.length() > 0) {
    WiFi.begin(otaSsid.c_str(), otaPass.c_str());
    unsigned long start = millis();
    while (WiFi.status() != WL_CONNECTED && millis() - start < 15000) {
      delay(200);
    }
    if (WiFi.status() == WL_CONNECTED) {
      ArduinoOTA.setHostname("BarnSensor");
      ArduinoOTA.begin();
    }
  }
}

void loop() {
  if (otaMode && otaSsid.length() > 0 && WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
  }

  unsigned long now = millis();
  if (now - lastStatus > 30000) {
    lastStatus = now;
    sendStatus();
  }
  if (now - lastSend < sendIntervalMs) return;
  lastSend = now;

  float t = NAN;
  float h = NAN;
  for (int i = 0; i <= dhtRetry; i++) {
    t = dht.readTemperature();
    h = dht.readHumidity();
    if (isValid(t, h)) break;
    delay(200);
  }

  if (isValid(t, h)) {
    lastTemp = t;
    lastHum = h;
  }

  // Always send a packet (heartbeat). Use last valid if current is invalid.
  data.temp = isValid(t, h) ? t : lastTemp;
  data.hum  = isValid(t, h) ? h : lastHum;

  esp_now_send(broadcastAddress, (uint8_t*)&data, sizeof(data));

  if (pendingRestart) {
    delay(200);
    ESP.restart();
  }
}
