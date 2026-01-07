#!/usr/bin/env python3
import json
import time
import ast
import rclpy
from rclpy.node import Node
from paho.mqtt import client as mqtt
from std_msgs.msg import Int32

class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')
        
        # 1. 브로커 설정
        self.declare_parameter('broker_host', '10.10.14.109')
        self.host = self.get_parameter('broker_host').value
        
        # 2. 토픽 설정 (두 곳 다 구독해야 함)
        self.topic_cmd_sub = 'ess/alert/robot' 
        self.topic_alert_sub = 'ess/alert'     
        self.topic_pub = 'ess/alert'  # 서버로 보낼 때 사용

        self.current_zone_num = 0 

        # 3. ROS 2 통신 설정
        # [출동용] 이 토픽으로 int(구역번호)를 쏘면 ControlNode가 움직입니다.
        self.emergency_pub = self.create_publisher(Int32, '/ess/priority_zone', 10)
        
        # [내 위치 수신용] 로봇의 현재 구역 정보를 받습니다.
        self.zone_sub = self.create_subscription(
            Int32,
            '/ess/zone_status',
            self._on_robot_zone_update,
            10
        )

        # 4. MQTT Client 설정
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self._on_connect
        self.mqtt_client.on_message = self._on_message
        
        try:
            self.mqtt_client.connect(self.host, 1883)
            self.mqtt_client.loop_start()
            self.get_logger().info(f'MqttBridgeNode Started. Host: {self.host}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to MQTT Broker: {e}')

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self.get_logger().info('Connected to MQTT Broker successfully')
            # [수정] 두 개의 토픽 모두 구독 (리스트 형태)
            topic_list = [(self.topic_cmd_sub, 0), (self.topic_alert_sub, 0)]
            client.subscribe(topic_list)
            self.get_logger().info(f'Subscribed to: {self.topic_cmd_sub} AND {self.topic_alert_sub}')
        else:
            self.get_logger().error(f'Connection failed with code {rc}')

    def _on_robot_zone_update(self, msg):
        self.current_zone_num = msg.data

    def _on_message(self, client, userdata, msg):
        try:
            payload_str = msg.payload.decode('utf-8')
            
            # 1. 데이터 파싱
            try:
                data = json.loads(payload_str)
            except json.JSONDecodeError:
                try:
                    data = ast.literal_eval(payload_str)
                except:
                    self.get_logger().error(f'Cannot parse payload: {payload_str}')
                    return

            # 2. 데이터 전처리 (소문자 변환, 공백 제거)
            location_raw = str(data.get("location", "")).strip().lower()
            message_raw = str(data.get("message", "")).strip().lower()
            event_type = str(data.get("event_type", "")).strip().lower()

            # ==================================================================
            # [Logic 1] 내가 가스 감지 -> 서버로 전송
            # 조건: location == "robot_1" AND event_type == "gas"
            # ==================================================================
            if location_raw == "robot_1" and event_type == "gas":
                
                response_payload = data.copy()
                # 내 위치를 실어서 보냄
                response_payload["location"] = f"zone_{self.current_zone_num}"
                # 내가 처리했다는 표시 (message에 robot_1 넣기)
                response_payload["message"] = "robot_1"

                try:
                    json_str = json.dumps(response_payload)
                    self.mqtt_client.publish(self.topic_pub, json_str)
                    self.get_logger().info(f'>> [GAS DETECTED] Forwarding to Server: {json_str}')
                except Exception as e:
                    self.get_logger().error(f'Failed to publish response: {e}')

            # ==================================================================
            # [Logic 2] 가스 발생 알림 수신 -> 로봇 출동
            # 조건: event_type == "gas" AND location에 "zone" 포함 AND message에 "robot" 없음
            # ==================================================================
            elif event_type == "gas" and "zone" in location_raw and "robot" not in message_raw:
                
                try:
                    # location 문자열(예: "zone_3")에서 숫자만 추출
                    target_zone = int(location_raw.split('_')[-1])
                    
                    # ROS 2 토픽 발행 (ControlNode가 이걸 듣고 움직임)
                    ros_msg = Int32()
                    ros_msg.data = target_zone
                    self.emergency_pub.publish(ros_msg)
                    
                    self.get_logger().warn(f'>>> [EMERGENCY CALL] Gas at {location_raw}! Moving Robot to Zone {target_zone}')
                    
                except ValueError:
                    self.get_logger().error(f'Failed to parse zone number from: {location_raw}')
                except Exception as e:
                    self.get_logger().error(f'Error executing emergency logic: {e}')

            # (옵션) 내가 보낸 메시지가 되돌아온 경우 (Loop-back)
            elif "robot" in message_raw:
                # 이건 내가 보낸거니까 그냥 무시
                pass

        except Exception as e:
            self.get_logger().error(f'Critical Error inside _on_message: {e}')

def main():
    rclpy.init()
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.mqtt_client.loop_stop()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
