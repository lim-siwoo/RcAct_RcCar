import paho.mqtt.client as mqtt

class soonyong_mqtt:
    def __init__(self, address, port):
        # MQTT Broker setup
        self.broker_address = address
        self.broker_port = port
        self.mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.mqttc.on_connect = self.on_connect
        self.mqttc.on_message = self.on_message
        self.sub_topic = []

    # The callback for when the client receives a CONN-ACK response from the server.        
    def on_connect(client, userdata, flags, reason_code, properties):
        print(f"Connected with result code {reason_code}")
        
    # The callback for when a PUBLISH message is received from the server.
    def on_message(self, client, userdata, msg):
        if msg.topic in self.sub_topic:
            print(f"{msg.topic} {str(msg.payload)}")
        
    def connect(self):
        self.mqttc.connect(self.broker_address, self.broker_port, 60)
        self.mqttc.loop_start()
        
    def publish(self, topic, message):
        self.mqttc.publish(topic, message)
        
    def subscribe(self, topic):
        self.mqttc.subscribe(topic)
        self.sub_topic.append(topic)
    
    def unsub(self, topic):
        if topic in self.sub_topic:
            self.mqttc.unsubscribe(topic)
            self.sub_topic.remove(topic)
        else:
            print(f"topic {topic} is not subscribed.")
            
    def disconnect(self):
        self.mqttc.loop_stop()
        self.mqttc.disconnect()