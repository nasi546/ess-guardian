import json
import time
import paho.mqtt.client as mqtt
from db_service import (
    save_environment,
    save_alert,
    get_admin_by_id,
    log_access_result
)

BROKER = "10.10.14.109"
PORT = 1883

# ================================
# MQTT Connect / Subscribe
# ================================
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("[MQTT] Connected successfully")
        client.subscribe("ess/env")
        client.subscribe("ess/alert")
        client.subscribe("ess/access/request")
        print("[MQTT] Subscribed: ess/env, ess/alert, ess/access/request")
    else:
        print(f"[MQTT ERROR] Connection failed with code {rc}")


# ================================
# MQTT Message Handler
# ================================
def on_message(client, userdata, msg):
    topic = msg.topic

    try:
        payload = msg.payload.decode("utf-8")
        data = json.loads(payload)
    except Exception as e:
        print(f"[ERROR] Invalid  Message: {e}")
        return

    print(f"[MQTT] Received ({topic}): {data}")

    if topic == "ess/env":
        handle_environment(data)
    elif topic == "ess/alert":
        handle_alert(data)
    elif topic == "ess/access/request":
        handle_access_request(client, data)


# ================================
# Environment / Alert Handlers
# ================================
def handle_environment(data):
    # STM32 발행 JSON: {"t": 23.1, "h": 55.3, "fan": "ON", "reason": "TEMP"}
    temp = data.get("t")
    humid = data.get("h")
    fan = data.get("fan", "OFF")		# 데이터가 없으면 기본값 "OFF"
    reason = data.get("reason", "-")	# 데이터가 없으면 기본값 "-"
    if temp is not None and humid is not None:
        save_environment({"temperature": temp, "humidity": humid, "fan": fan, "reason": reason})


def handle_alert(data):
    save_alert(data)


# ================================
# Access Request Handler
# ================================
def handle_access_request(client, data):
    admin_id = data.get("admin_id")
    access_point = data.get("access_point")

    if not admin_id or not access_point:
        print("[ACCESS ERROR] Missing admin_id or access_point")
        return

    # DB에서 admin 정보 가져오기
    admin = get_admin_by_id(admin_id)

    if admin:
        # admin은 {"id": 3, "access_points": "main,ew2"} 구조라고 가정
        access_points = admin.get("access_points", "")
        allowed_points = [x.strip() for x in access_points.split(",")]

        result = "success" if access_point in allowed_points else "fail"
    else:
        result = "fail"

    # DB 로그 기록
    log_access_result(admin_id=admin_id, access_point=access_point, result=result)

    # STM32로 성공/실패만 응답
    response = {"result": result}
    client.publish("ess/access/response", json.dumps(response))
    print(f"[ACCESS] ID:{admin_id} -> {access_point} [{result}]")


# ================================
# Subscriber 실행
# ================================
def run_subscriber():
    client = mqtt.Client(protocol=mqtt.MQTTv311, clean_session=True)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(BROKER, PORT, keepalive=60)
        print("[MQTT] Subscriber started. Waiting for messages...")
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n[MQTT] Stopped by user")
    except Exception as e:
        print(f"[MQTT CRITICAL ERROR] {e}")


# ================================
# Entry Point
# ================================
if __name__ == "__main__":
    run_subscriber()
