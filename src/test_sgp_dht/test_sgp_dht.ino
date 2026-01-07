#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

SoftwareSerial stm(D6, D5);   // RX=D6(GPIO12), TX=D5(GPIO14)

const char* ssid = "embA";
const char* pass = "embA1234";

const char* mqtt_host = "10.10.14.109";
const int   mqtt_port = 1883;

WiFiClient espClient;
PubSubClient client(espClient);

// ===== SGP 임계값 =====
const int TVOC_WARN = 600;
const int TVOC_CRIT = 1000;

// ===== alert 스팸 방지(선택) =====
#define USE_COOLDOWN 1
#if USE_COOLDOWN
unsigned long lastPubMs = 0;
String lastZone = "";
String lastLevel = "";
const unsigned long COOLDOWN_MS = 10000;
#endif

// ===== env 전송 주기 제한(추천) =====
unsigned long lastEnvMs = 0;
const unsigned long ENV_INTERVAL_MS = 10000; // 10초에 1번만 ess/env publish

void wifi_connect() {
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi OK");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
}

void mqtt_connect() {
  while (!client.connected()) {
    Serial.print("MQTT connecting...");
    String cid = "D1mini_" + String(ESP.getChipId(), HEX);
    if (client.connect(cid.c_str())) {
      Serial.println("OK");
    } else {
      Serial.print("FAIL rc=");
      Serial.println(client.state());
      delay(1000);
    }
  }
}

String decideLevel(int tvoc, int eco2) {
  (void)eco2; // 지금은 TVOC 기준만 사용
  if (tvoc >= TVOC_CRIT) return "critical";
  if (tvoc >= TVOC_WARN) return "warning";
  return "";
}

#if USE_COOLDOWN
bool allowPublish(const String& zone, const String& level) {
  unsigned long now = millis();
  if (zone == lastZone && level == lastLevel && (now - lastPubMs) < COOLDOWN_MS) return false;
  lastZone = zone;
  lastLevel = level;
  lastPubMs = now;
  return true;
}
#endif

void publish_alert(const String& zone, int tvoc, int eco2) {
  String level = decideLevel(tvoc, eco2);
  if (level == "") {
    Serial.printf("OK  zone=%s tvoc=%d eco2=%d\n", zone.c_str(), tvoc, eco2);
    return;
  }

#if USE_COOLDOWN
  if (!allowPublish(zone, level)) return;
#endif

  StaticJsonDocument<256> out;
  out["event_type"] = "gas";
  out["level"] = level;
  out["value"] = tvoc; // value는 tvoc로 사용
  out["location"] = zone;
  out["message"] = (level == "critical") ? "Gas leak detected" : "Gas level elevated";

  char payload[256];
  serializeJson(out, payload, sizeof(payload));

  client.publish("ess/alert", payload);
  Serial.print("PUB ess/alert: ");
  Serial.println(payload);
}

void publish_env(float t, float h) {
  // 10초 rate-limit (DB 스팸 방지)
  unsigned long now = millis();
  if (now - lastEnvMs < ENV_INTERVAL_MS) return;
  lastEnvMs = now;

  // ✅ 규칙: env는 t/h만!
  char payload[64];
  snprintf(payload, sizeof(payload), "{\"t\":%.2f,\"h\":%.2f}", t, h);

  client.publish("ess/env", payload);
  Serial.print("PUB ess/env: ");
  Serial.println(payload);
}

void setup() {
  Serial.begin(115200);
  stm.begin(115200);  // STM32 USART1 115200과 동일
  delay(300);

  wifi_connect();
  client.setServer(mqtt_host, mqtt_port);
  mqtt_connect();

  Serial.println("READY");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) wifi_connect();
  if (!client.connected()) mqtt_connect();
  client.loop();

  if (!stm.available()) return;

  String line = stm.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.print("JSON parse fail: ");
    Serial.println(line);
    return;
  }

  // ✅ 0) ENV 우선 처리: type 없어도 t/h만 있으면 env로 처리
  if (doc.containsKey("t") && doc.containsKey("h") && !doc.containsKey("tvoc")) {
    float t = doc["t"] | 0.0;
    float h = doc["h"] | 0.0;
    Serial.printf("ENV t=%.2f h=%.2f\n", t, h);
    publish_env(t, h);
    return;
  }

  // ✅ 1) SGP 처리 (기존 방식)
  String type = doc["type"] | "";
  if (type == "sgp") {
    String zone = doc["zone"] | "zone_1";
    int tvoc = doc["tvoc"] | 0;
    int eco2 = doc["eco2"] | 0;
    publish_alert(zone, tvoc, eco2);
    return;
  }

}
