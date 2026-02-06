#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "DHT.h"

#define DHTPIN 19
#define DHTTYPE DHT11

// Set to your master Wi-Fi channel for best reliability.
// Leave 0 to skip forcing channel.
#define WIFI_CHANNEL 0

// Send interval (ms)
#define SEND_INTERVAL_MS 5000

DHT dht(DHTPIN, DHTTYPE);

uint8_t broadcastAddress[] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

struct SensorData {
  uint8_t mac[6];
  float temp;
  float hum;
};

SensorData data;
unsigned long lastSend = 0;

float lastTemp = NAN;
float lastHum = NAN;

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

  if (WIFI_CHANNEL > 0) {
    esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
  }

  WiFi.macAddress(data.mac);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW error");
    return;
  }

  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = (WIFI_CHANNEL > 0) ? WIFI_CHANNEL : 0;
  peerInfo.encrypt = false;
  esp_now_add_peer(&peerInfo);

  dht.begin();
  delay(1500); // let DHT settle on first boot
}

void loop() {
  unsigned long now = millis();
  if (now - lastSend < SEND_INTERVAL_MS) return;
  lastSend = now;

  float t = dht.readTemperature();
  float h = dht.readHumidity();

  if (isValid(t, h)) {
    lastTemp = t;
    lastHum = h;
  }

  // Always send a packet (heartbeat). Use last valid if current is invalid.
  data.temp = isValid(t, h) ? t : lastTemp;
  data.hum  = isValid(t, h) ? h : lastHum;

  esp_now_send(broadcastAddress, (uint8_t*)&data, sizeof(data));
}
