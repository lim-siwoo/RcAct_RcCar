from mqtt import MQTT

mqtt_client = MQTT("127.0.0.1", 1883)
mqtt_client.connect()
mqtt_client.subscribe("soonyong")
mqtt_client.publish("soonyong", "Hello")

try:
    while True:
        pass
except KeyboardInterrupt:
    mqtt_client.disconnect()
    
print("end")