# communication/mqtt_handler.py
import paho.mqtt.client as mqtt
from config.settings import *

class MQTTHandler:
    def __init__(self, broker_address, broker_port):
        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.broker_address = broker_address
        self.broker_port = broker_port
        self.setup_callbacks()

    def setup_callbacks(self):
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    def _on_connect(self, client, userdata, flags, reason_code, properties):
        print(f"Connected with result code {reason_code}")
        client.subscribe("iot/RCcar/mode")
        client.subscribe("iot/RCcar/power")

    def _on_message(self, client, userdata, msg):
        print(f"{msg.topic} {str(msg.payload)}")
        # 메시지 처리 로직은 별도의 핸들러로 분리

    def connect(self):
        self.client.connect(self.broker_address, self.broker_port, 60)
        self.client.loop_start()

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()

    def publish(self, topic, message):
        self.client.publish(topic, message)