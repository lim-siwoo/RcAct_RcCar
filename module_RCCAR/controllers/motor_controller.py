# controllers/motor_controller.py
from Raspi_MotorHAT import Raspi_MotorHAT

class MotorController:
    def __init__(self, address, motor_num, default_speed):
        self.mh = Raspi_MotorHAT(addr=address)
        self.motor = self.mh.getMotor(motor_num)
        self.default_speed = default_speed
        self.mqtt_client = None

    def set_mqtt_client(self, mqtt_client):
        self.mqtt_client = mqtt_client

    def set_speed(self, speed, direction):
        self.motor.setSpeed(abs(speed))
        
        if direction == "FORWARD":
            self.motor.run(Raspi_MotorHAT.BACKWARD)
            self._publish_mqtt("FORWARD")
        elif direction == "BACKWARD":
            self.motor.run(Raspi_MotorHAT.FORWARD)
            self._publish_mqtt("BACKWARD")
        else:
            self.motor.run(Raspi_MotorHAT.RELEASE)
            self._publish_mqtt("STOP")

    def _publish_mqtt(self, message):
        if self.mqtt_client:
            self.mqtt_client.publish("iot/RCcar/command", message)

    def stop(self):
        self.set_speed(0, "STOP")