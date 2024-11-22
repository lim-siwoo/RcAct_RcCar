# main.py
import cv2
import threading
import time
from multiprocessing import Process
import sys
from config.settings import *
from controllers.motor_controller import MotorController
from controllers.servo_controller import ServoController
from controllers.pose_controller import PoseController
from hardware.camera import CameraManager
from hardware.sense_hat import SenseHatManager
from communication.mqtt_handler import MQTTHandler
from utils.system_monitor import SystemMonitor
from utils.pid import PIDController
from web.app import frame_queue, run_flask

class RCCarController:
    def __init__(self):
        # Initialize components
        self.mode = "AUTO"
        self.is_running = True
        self.camera = CameraManager(CAMERA_SIZE_X, CAMERA_SIZE_Y)
        self.sense_hat = SenseHatManager()
        self.motor = MotorController(MOTOR_ADDRESS, MOTOR_NUMBER, MOTOR_SPEED)
        self.servo = ServoController(MOTOR_ADDRESS, SERVO_MID, SERVO_LEFT, SERVO_RIGHT)
        self.pose_controller = PoseController()
        self.mqtt = MQTTHandler(BROKER_ADDRESS, BROKER_PORT)
        
        # Initialize PID controllers
        self.pid_x = PIDController(PID_KP_X, PID_KI_X, PID_TC, PID_INTEGRAL_MAX)
        self.pid_y = PIDController(PID_KP_Y, PID_KI_Y, PID_TC, PID_INTEGRAL_MAX)
        self.pid_z = PIDController(PID_KP_Z, PID_KI_Z, PID_TC, PID_INTEGRAL_MAX)
        
        # Set up MQTT callbacks
        self.setup_mqtt_callbacks()
        self.mqtt.connect()
        
        # Connect motor controller to MQTT
        self.motor.set_mqtt_client(self.mqtt.client)
        
        # Start system monitoring
        self.start_monitoring()

    def setup_mqtt_callbacks(self):
        def on_message(client, userdata, msg):
            if msg.topic == "iot/RCcar/power":
                if msg.payload in [b"STOP", b"off"]:
                    self.stop_car()
                    if msg.payload == b"off":
                        self.shutdown()
            
            elif msg.topic == "iot/RCcar/mode":
                if msg.payload == b"MANUAL":
                    self.mode = "MANUAL"
                elif msg.payload == b"AUTO":
                    self.mode = "AUTO"

        self.mqtt.client.on_message = on_message

    def start_monitoring(self):
        def send_status():
            while self.is_running:
                sensor_data = self.sense_hat.get_sensors_data()
                self.mqtt.publish(TOPIC_TEMP, sensor_data['temperature'])
                self.mqtt.publish(TOPIC_HUMID, sensor_data['humidity'])
                self.mqtt.publish(TOPIC_CPU_TEMP, SystemMonitor.get_cpu_temperature())
                self.mqtt.publish(TOPIC_CPU_USAGE, SystemMonitor.get_cpu_usage())
                time.sleep(5)

        status_thread = threading.Thread(target=send_status)
        status_thread.daemon = True
        status_thread.start()

    def process_frame(self):
        frame = self.camera.capture_frame()
        results = self.pose_controller.process_frame(frame)
        
        if results.pose_landmarks:
            landmarks = results.pose_landmarks.landmark
            nose = landmarks[self.pose_controller.mp_pose.PoseLandmark.NOSE.value]
            nose_pos = [
                nose.x * frame.shape[1],
                nose.y * frame.shape[0],
                nose.z * REFERENCE_Z * 2
            ]

            # Calculate errors
            error_x = -(REFERENCE_X - nose_pos[0])
            error_y = REFERENCE_Y - nose_pos[1]
            error_z = REFERENCE_Z + nose_pos[2]

            # Calculate control values
            u_x = self.pid_x.calculate(error_x)
            u_y = self.pid_y.calculate(error_y)
            u_z = self.pid_z.calculate(error_z)

            # Process pose and control car
            is_reversing = u_z < 20
            self.process_pose_and_control(landmarks, u_x, u_y, u_z, is_reversing)
            
            # Update visualization
            self.draw_tracking_visualization(frame, nose_pos, is_reversing)
        else:
            self.mqtt.publish(TOPIC_STATUS, "NO PERSON")
            self.stop_car()

        return frame

    def process_pose_and_control(self, landmarks, u_x, u_y, u_z, is_reversing):
        pose_action = self.pose_controller.detect_pose(landmarks)
        
        if is_reversing:
            self.sense_hat.set_led_color("SAD")
        else:
            self.sense_hat.set_led_color("HAPPY")

        if pose_action == "FORWARD":
            self.servo.set_position(SERVO_MID)
            self.motor.set_speed(MOTOR_SPEED, "FORWARD")
        elif pose_action == "BACKWARD":
            self.servo.set_position(SERVO_MID)
            self.motor.set_speed(MOTOR_SPEED, "BACKWARD")
        elif pose_action == "STOP":
            self.stop_car()
        elif pose_action == "SPECIAL":
            self.execute_special_action(is_reversing)
        else:  # FOLLOW mode
            self.follow_mode_control(u_x, u_z, is_reversing)

    def follow_mode_control(self, u_x, u_z, is_reversing):
        # Steering control
        # print(u_x)
        if abs(u_x) > 5:
            # print("servo123")
            if u_x < 0:
                self.servo.set_position(SERVO_RIGHT, is_reversing)
                self.mqtt.publish(TOPIC_COMMAND, "TURNLEFT")
            else:
                self.servo.set_position(SERVO_LEFT, is_reversing)
                self.mqtt.publish(TOPIC_COMMAND, "TURNRIGHT")
        else:
            self.servo.set_position(SERVO_MID)

        # Speed control
        print(u_z)
        if u_z > 20 and u_z < 50:
            self.motor.set_speed(0, "STOP")
            self.sense_hat.set_led_color("NORMAL")
        elif u_z > 50:
            self.motor.set_speed(MOTOR_SPEED, "FORWARD")
        elif u_z < 20:
            self.motor.set_speed(MOTOR_SPEED, "BACKWARD")

        self.mqtt.publish(TOPIC_STATUS, "FOLLOW")

    def execute_special_action(self, is_reversing):
        self.mqtt.publish(TOPIC_COMMAND, "SPECIAL")
        self.servo.set_position(SERVO_LEFT, is_reversing)
        self.motor.set_speed(MOTOR_SPEED, "FORWARD")
        time.sleep(6)
        self.stop_car()

    def draw_tracking_visualization(self, frame, nose_pos, is_reversing):
        # Draw reference point and nose position
        cv2.circle(frame, (int(REFERENCE_X), int(REFERENCE_Y-100)), 5, (0, 0, 255), -1)
        cv2.circle(frame, (int(nose_pos[0]), int(nose_pos[1])), 5, (0, 255, 0), -1)
        cv2.line(frame, (int(REFERENCE_X), int(REFERENCE_Y-20)), 
                (int(nose_pos[0]), int(nose_pos[1])), (0, 255, 255), 2)

        # Add direction indicator
        direction_text = "REVERSE" if is_reversing else "FORWARD"
        cv2.putText(frame, f"Direction: {direction_text}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    def stop_car(self):
        self.motor.stop()
        self.servo.set_position(SERVO_MID)
        self.sense_hat.clear()

    def shutdown(self):
        self.is_running = False
        self.stop_car()
        self.camera.close()
        self.mqtt.publish(TOPIC_STATUS, "END")
        self.mqtt.disconnect()
        cv2.destroyAllWindows()

    def run(self):
        try:
            # Start Flask in a separate process
            flask_process = Process(target=run_flask)
            flask_process.start()

            while self.is_running:
                frame = self.process_frame()
                
                # Update frame queue for web streaming
                if frame_queue.full():
                    frame_queue.get()
                frame_queue.put(frame)
                
                cv2.imshow('RC Car Pose Following', frame)
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break

        except KeyboardInterrupt:
            pass
        finally:
            self.shutdown()
            flask_process.terminate()
            flask_process.join()
            sys.exit(0)

if __name__ == "__main__":
    controller = RCCarController()
    controller.run()