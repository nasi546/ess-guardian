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

// ===== ACCESS 토픽 (담당자 스펙 고정) =====
const char* TOPIC_ACCESS_REQ  = "ess/access/request";
const char* TOPIC_ACCESS_RESP = "ess/access/response";

// -------------------- WiFi/MQTT --------------------
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

void onMqttMessage(char* topic, byte* payload, unsigned int length) {
  // payload -> String
  String msg;
  msg.reserve(length + 1);
  for (unsigned int i = 0; i < length; i++) msg += (char)payload[i];

  Serial.print("[MQTT RX] ");
  Serial.print(topic);
  Serial.print(" : ");
  Serial.println(msg);

  // access response면 STM32로 그대로 전달 (한 줄 JSON)
  if (String(topic) == TOPIC_ACCESS_RESP) {
    // STM32가 readStringUntil('\n')로 읽기 쉽게 '\n' 붙여서 보냄
    stm.print(msg);
    stm.print("\n");
  }
}

void mqtt_connect() {
  while (!client.connected()) {
    Serial.print("MQTT connecting...");
    String cid = "D1mini_" + String(ESP.getChipId(), HEX);

    if (client.connect(cid.c_str())) {
      Serial.println("OK");

      // ✅ response 구독 (재연결 시마다 다시 구독해야 함)
      client.subscribe(TOPIC_ACCESS_RESP);
      Serial.print("SUB: "); Serial.println(TOPIC_ACCESS_RESP);

    } else {
      Serial.print("FAIL rc=");
      Serial.println(client.state());
      delay(1000);
    }
  }
}

// -------------------- 기존 alert/env --------------------
String decideLevel(int tvoc, int eco2) {
  (void)eco2;
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
  out["value"] = tvoc;
  out["location"] = zone;
  out["message"] = (level == "critical") ? "Gas leak detected" : "Gas level elevated";

  char payload[256];
  serializeJson(out, payload, sizeof(payload));

  client.publish("ess/alert", payload);
  Serial.print("PUB ess/alert: ");
  Serial.println(payload);
}

void publish_env(float t, float h) {
  unsigned long now = millis();
  if (now - lastEnvMs < ENV_INTERVAL_MS) return;
  lastEnvMs = now;

  char payload[64];
  snprintf(payload, sizeof(payload), "{\"t\":%.2f,\"h\":%.2f}", t, h);

  client.publish("ess/env", payload);
  Serial.print("PUB ess/env: ");
  Serial.println(payload);
}

// -------------- mapping uid to rfid
String mapUidArrToRfid6(int u0, int u1, int u2, int u3) {
  if (u0==209 && u1==101 && u2==199 && u3==5) return "RFID_123456";
  if (u0==139 && u1==114 && u2==206 && u3==5) return "RFID_234567";
  if (u0==156 && u1==220 && u2==206 && u3==5) return "RFID_234567";
  if (u0==227 && u1==94 && u2==180 && u3==5) return "RFID_234567";
  return "";
}


// -------------------- ✅ ACCESS request publish --------------------
void publish_access_request(const String& admin_id, const String& access_point) {
  // 담당자 스펙: {"admin_id":"...","access_point":"..."} 그대로
  StaticJsonDocument<128> out;
  out["admin_id"] = admin_id;
  out["access_point"] = access_point;

  char payload[128];
  serializeJson(out, payload, sizeof(payload));

  client.publish(TOPIC_ACCESS_REQ, payload);
  Serial.print("PUB ess/access/request: ");
  Serial.println(payload);
}

// -------------------- setup/loop --------------------
void setup() {
  Serial.begin(115200);
  stm.begin(115200);  // STM32 UART와 동일 속도
  delay(300);

  wifi_connect();

  client.setServer(mqtt_host, mqtt_port);
  client.setCallback(onMqttMessage);

  mqtt_connect();
  Serial.println("READY");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) wifi_connect();
  if (!client.connected()) mqtt_connect();
  client.loop();

  if (!stm.available()) return;

  // STM32가 '\n' 끝나는 JSON 한 줄을 보내야 함
  String line = stm.readStringUntil('\n');
  line.trim();
  if (line.length() == 0) return;

  // ✅ JSON만 뽑아내기 (UID/잡문 섞여도 OK)
  int s = line.indexOf('{');
  int e = line.lastIndexOf('}');
  if (s < 0 || e < 0 || e <= s) {
    Serial.print("No JSON frame: ");
    Serial.println(line);
    return;
  }
  String json = line.substring(s, e + 1);

  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, json);
  if (err) {
    Serial.print("JSON parse fail: ");
    Serial.println(json);
    return;
  }

  // ✅ 0) ENV 처리 (기존)
  if (doc.containsKey("t") && doc.containsKey("h") && !doc.containsKey("tvoc")) {
    float t = doc["t"] | 0.0;
    float h = doc["h"] | 0.0;
    Serial.printf("ENV t=%.2f h=%.2f\n", t, h);
    publish_env(t, h);
    return;
  }

  // ✅ 1) SGP 처리 (기존)
  String type = doc["type"] | "";
  if (type == "sgp") {
    String zone = doc["zone"] | "zone_1";
    int tvoc = doc["tvoc"] | 0;
    int eco2 = doc["eco2"] | 0;
    publish_alert(zone, tvoc, eco2);
    return;
  }

  // ✅ 2) ACCESS 처리 (추가)
  // STM32에서 {"admin_id":"...","access_point":"..."} 로 보내면 바로 처리
  if (doc.containsKey("access_point")) {
    String access_point = doc["access_point"] | "";
    access_point.trim();

    String rfid_admin_id = "";
    if (rfid_admin_id.length() == 0 && doc.containsKey("uid")) {
      JsonArray uidArr = doc["uid"].as<JsonArray>();
      if (!uidArr.isNull() && uidArr.size() >= 4) {
        int u0 = uidArr[0] | -1;
        int u1 = uidArr[1] | -1;
        int u2 = uidArr[2] | -1;
        int u3 = uidArr[3] | -1;
        rfid_admin_id = mapUidArrToRfid6(u0, u1, u2, u3);
      }
    }

    // 결과 처리
    if (rfid_admin_id.length() == 0) {
      Serial.print("[ACCESS] Unknown card. raw json=");
      Serial.println(json);
      return;
    }

    // ✅ 서버 스펙 그대로 publish: {"admin_id":"RFID_123456","access_point":"ew1"}
    publish_access_request(rfid_admin_id, access_point);
    return;
  }
}
