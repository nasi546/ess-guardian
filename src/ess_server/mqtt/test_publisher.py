import paho.mqtt.client as mqtt
import time
import json
import random
import time

broker = "127.0.0.1"
port = 1883

client = mqtt.Client(protocol=mqtt.MQTTv311)
client.connect(broker, port)

try:
    for i in range(10):
        data = {
             "temperature":round(random.uniform(18.0, 30.0), 2),
             "humidity": round(random.uniform(30.0, 70.0), 2)
         }

        client.publish("ess/env", json.dumps(data))
        print(f"[Publish] ess/env:", data)
        time.sleep(2)

except KeyboardInterrupt:
    print("Publisher stopped")

finally:
    client.disconnect()
