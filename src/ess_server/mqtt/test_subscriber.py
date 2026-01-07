import paho.mqtt.client as mqtt
import json
import mysql.connector

db = mysql.connector.connect(
        host="127.0.0.1",
        user="ess",
        password="ess1234",
        database="ess_db"
        )
cursor = db.cursor()

broker = "127.0.0.1"
port = 1883

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe("ess/env")
    client.subscribe("ess/alert")
    client.subscribe("ess/access/request")

def on_message(client, userdata, msg):
    topic = msg.topic
    data = json.loads(msg.payload)

    if topic == "ess/env":
        save_environment(data)

    elif topic == "ess/alert":
        save_alert(data)

    elif topic == "ess/access/request":
        check_access_request(data)

    print(f"[{topic}] Received:", data)

def save_environment(data):
    cursor.execute("""
        INSERT INTO environment_data (temperature, humidity)
        VALUES (%s, %s)
    """, (data["temperature"], data["humidity"]))
    db.commit()

def save_alert(data):
    cursor.execute("""
        INSERT INTO alert_events (event_type, level, value)
        VALUES (%s, %s, %s)
    """, (data["event_type"], data["level"], data["value"]))
    db.commit()

def check_access_request(data):
    print("Access request received:", data)
    pass

client = mqtt.Client(protocol=mqtt.MQTTv311)
client.on_connect = on_connect
client.on_message = on_message

client.connect(broker, port)
client.loop_forever()
    
