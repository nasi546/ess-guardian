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

// ===== 임계값(테스트용 기본값) =====
const int TVOC_WARN = 600;
const int TVOC_CRIT = 1000;
const int ECO2_WARN = 1200;
const int ECO2_CRIT = 2000;

// ===== 스팸 방지(선택) =====
// 1이면 켜짐, 0이면 완전 제외
#define USE_COOLDOWN 1
#if USE_COOLDOWN
unsigned long lastPubMs = 0;
String lastZone = "";
String lastLevel = "";
const unsigned long COOLDOWN_MS = 10000;
#endif

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
  if (tvoc >= TVOC_CRIT) return "critical";
  if (tvoc >= TVOC_WARN) return "warning";
  return ""; // 정상
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
    // 정상은 전송하지 않음(규칙)
    Serial.printf("OK  zone=%s tvoc=%d eco2=%d\n", zone.c_str(), tvoc, eco2);
    return;
  }

#if USE_COOLDOWN
  if (!allowPublish(zone, level)) return;
#endif

  // 서버/DB 규격에 맞는 ess/alert payload 생성
  StaticJsonDocument<256> out;
  out["event_type"] = "gas";
  out["level"] = level;

  // value는 한 개만 들어가므로 일단 tvoc를 value로 사용(권장)
  // eco2를 value로 쓰고 싶으면 아래 줄을 eco2로 바꿔
  out["value"] = tvoc;

  out["location"] = zone;
  out["message"] = (level == "critical") ? "Gas leak detected" : "Gas level elevated";

  char payload[256];
  serializeJson(out, payload, sizeof(payload));

  client.publish("ess/alert", payload);
  Serial.print("PUB ess/alert: ");
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

  // STM32에서 오는 raw 예:
  // {"type":"sgp","zone":"zone_1","tvoc":501,"eco2":1948}
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, line);
  if (err) {
    Serial.print("JSON parse fail: ");
    Serial.println(line);
    return;
  }

  const char* type = doc["type"] | "";
  if (String(type) != "sgp") {
    // sgp 외 타입은 일단 무시(나중에 env/access 추가 가능)
    return;
  }

  String zone = doc["zone"] | "zone_1";
  int tvoc = doc["tvoc"] | 0;
  int eco2 = doc["eco2"] | 0;

  // alert 규격으로 변환해서 발행
  publish_alert(zone, tvoc, eco2);
}
