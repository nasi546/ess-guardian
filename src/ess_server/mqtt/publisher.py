import paho.mqtt.client as mqtt
import json
import random
import time

BROKER = "10.10.14.109"
PORT = 1883

# ================================
#  MQTT Client Init
# ================================
client = mqtt.Client(protocol=mqtt.MQTTv311)

try:
    client.connect(BROKER, PORT)
except Exception as e:
    print("[ERROR] MQTT connection failed:", e)
    exit(1)


# ================================
#  Environment (t / h)
# ================================
def publish_environment():
    """
    온습도 센서 값 발행
    실제 센서 연동 시 STM은 {"t": ..., "h": ..., "fan": ..., "reason": ...} 형태로 보내도록 통일
    """

    temp = round(random.uniform(18.0, 32.0), 2)
    humid = round(random.uniform(30.0, 80.0), 2)

    fan_status = "OFF"
    reason = "-"

    if temp >= 28.0:
        fan_status = "ON"
        reason = "TEMP"
    elif humid >= 60.0:
        fan_status = "OFF"
        reason = "HUMI"

    else: fan_status = "OFF"

    data = {
        "t": temp,
        "h": humid,
        "fan": fan_status,
        "reason": reason
    }

    client.publish("ess/env", json.dumps(data))
    print("[Publish] ess/env:", data)


# ================================
#  Alert Events (gas, thermal)
# ================================

GAS_WARNING = 300
GAS_CRITICAL = 500

THERMAL_WARNING = 60.0
THERMAL_CRITICAL = 80.0

def generate_location(event_type: str) -> str:
    """
    프로젝트 규칙에 맞는 location 생성
    gas     -> zone_1 ~ zone_6
    thermal -> rack_1_1 ~ rack_3_3
    """
    if event_type == "gas":
        zone = random.randint(1, 6)
        return f"zone_{zone}"

    elif event_type == "thermal":
        rack = random.randint(1, 3)
        level = random.randint(1, 3)
        return f"rack_{rack}_{level}"

    return "unknown"

def publish_alert(event_type: str, level: str, value: float):
    """
    공통 Alert 발행 함수
    event_type: "gas", "thermal"
    level: "warning" 또는 "critical"
    """
    location = generate_location(event_type)

    alert = {
        "event_type": event_type,
        "level": level,
        "value": value,
        "location": location,
        "message": f"{event_type.upper()} {level.upper()} - value={value}"
    }


    client.publish("ess/alert", json.dumps(alert))
    print(f"[Publish] ess/alert ({event_type}/{level}):", alert)


def check_and_publish_gas():
    gas_value = random.randint(100, 600)

    if GAS_WARNING <= gas_value < GAS_CRITICAL:
        publish_alert("gas", "warning", gas_value)

    elif gas_value >= GAS_CRITICAL:
        publish_alert("gas", "critical", gas_value)


def check_and_publish_thermal():
    thermal_value = round(random.uniform(40.0, 90.0), 2)

    if THERMAL_WARNING <= thermal_value < THERMAL_CRITICAL:
        publish_alert("thermal", "warning", thermal_value)

    elif thermal_value >= THERMAL_CRITICAL:
        publish_alert("thermal", "critical", thermal_value)


# ================================
#  Access Request (출입 인증 요청)
# ================================
def publish_access_request(admin_id: str, access_point: str):
    """
    출입 인증 요청 발행
    admin_id : 등록된 관리자 RFID 코드
    access_point : 출입 지점(ex: ew2, main)
    """
    request = {
        "admin_id": admin_id,           # 오늘 기준 표준 키
        "access_point": access_point
    }

    client.publish("ess/access/request", json.dumps(request))
    print("[Publish] ess/access/request:", request)


# ================================
#  MAIN LOOP (테스트용)
# ================================
def main():
    print("Publisher started. Press Ctrl+C to stop.\n")

    try:
        while True:
            publish_environment()
            check_and_publish_gas()
            check_and_publish_thermal()

            # 수동 테스트 시 사용
            # publish_access_request("RFID_123456", "ew2")
            # publish_access_request("RFID_123456", "main")

            time.sleep(5)

    except KeyboardInterrupt:
        print("\nPublisher stopped.")
        client.disconnect()


if __name__ == "__main__":
    main()

