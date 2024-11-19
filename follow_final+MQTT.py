from Raspi_MotorHAT import Raspi_MotorHAT
from sense_hat import SenseHat
from Raspi_PWM_Servo_Driver import PWM
import paho.mqtt.client as mqtt
import cv2
import mediapipe as mp
import numpy as np
from picamera2 import Picamera2
import threading
import time
from enum import Enum


# MQTT Broker setup
broker_address = "70.12.229.60"
broker_port = 1883
topic = "iot/RCcar/command"
topic2 = "iot/RCcar/sensor"

# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, reason_code, properties):
    print(f"Connected with result code {reason_code}")
    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    # client.subscribe("$SYS/#")

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    print(msg.topic+" "+str(msg.payload))


mqttc = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
mqttc.on_connect = on_connect
mqttc.on_message = on_message

mqttc.connect(broker_address, broker_port, 60)

mqttc.loop_start()



# Sense HAT setup
sense = SenseHat()
sense.clear()
# Motor setup
mh = Raspi_MotorHAT(addr=0x6f)
dc_motor = mh.getMotor(2)
dc_motor_speed = 100

# Servo setup
servo = PWM(0x6f)
servo.setPWMFreq(60)
servo_mid = 370
servo_left = 470
servo_right = 270

# MediaPipe Pose setup
mp_pose = mp.solutions.pose
pose = mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Camera setup
sizeX = 480
sizeY = 270
picam2 = Picamera2()
# 카메라 설정: 120도 광각 모드를 위한 카메라 구성 설정
camera_config = picam2.create_video_configuration(
    main={'size': (1920, 1080)},  # 120도 광각 모드 해상도 설정
    controls={
        'AfMode': 2,               # 자동 초점 모드 활성화 (2는 Continuous 모드)
        'AfRange': 1,              # 광각 범위에 적합하도록 설정
        'LensPosition': 0.0,       # 광각 설정을 위한 렌즈 포지션 (적절한 값으로 설정 필요)
        'FrameRate': 3
    },
    buffer_count=2
)
picam2.configure(camera_config)
picam2.start()

# Reference points
rifX = sizeX / 2
rifY = sizeY / 2
rifZ = 400

# PI constants
Kp_X = 0.1
Ki_X = 0.0
Kp_Y = 0.2
Ki_Y = 0.0
Kp_Z = 0.4
Ki_Z = 0.0

# Control variables
Tc = 0.05
integral_X = integral_Y = integral_Z = 0
previous_error_X = previous_error_Y = previous_error_Z = 0
integral_max = 1000  # To prevent integral windup
is_reversing = False

def send_meesage5sec():
    #sense-hat humid temp
    humidity = sense.get_humidity()
    temperature = sense.get_temperature()
    #mqtt message
    message = f"humidity: {humidity}, temperature: {temperature}"
    mqttc.publish(topic2, message)
    timer = threading.Timer(5, send_meesage5sec)
    timer.daemon = True
    timer.start()

send_meesage5sec()

def calculate_angle(a, b, c):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    
    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)
    
    if angle > 180.0:
        angle = 360 - angle
    
    return angle

def detect_pose(landmarks):
    leftShoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,
                    landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
    leftElbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,
                 landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
    leftWrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x,
                 landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
    
    rightShoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,
                     landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
    rightElbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,
                  landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
    rightWrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x,
                  landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]

    leftAngle = calculate_angle(leftShoulder, leftElbow, leftWrist)
    rightAngle = calculate_angle(rightShoulder, rightElbow, rightWrist)
    leftShoulderAngle = calculate_angle(rightShoulder, leftShoulder, leftElbow)
    rightShoulderAngle = calculate_angle(leftShoulder, rightShoulder, rightElbow)

    if (rightAngle > 70 and rightAngle < 110 and #오른팔 꺾기
        leftAngle > 150 and leftShoulderAngle > 150):
        print("ORDER : FORWARD")
        sense_led("YELLOW")
        return "FORWARD"
    elif (leftAngle > 70 and leftAngle < 110 and  #왼팔 꺾기
          rightAngle > 150 and rightShoulderAngle > 150):
        print("ORDER : BACKWARD")
        sense_led("YELLOW")
        return "BACKWARD"
    elif (rightAngle > 80 and rightAngle < 100 and 
          leftAngle > 80 and leftAngle < 100 and 
          leftShoulderAngle > 150 and rightShoulderAngle > 150):
        sense_led("YELLOW")
        return "STOP"
    elif (rightAngle > 150 and leftAngle > 150 and 
          leftShoulderAngle > 150 and rightShoulderAngle > 150):
        sense_led("YELLOW")
        return "SPECIAL"
    else:
        return "FOLLOW"

def set_servo_position(position, is_reversing=False):
    if is_reversing:
        position = servo_mid + (servo_mid - position)
    position = max(servo_right, min(position, servo_left))
    servo.setPWM(0, 0, position)

def set_dc_motor(speed, direction):
    dc_motor.setSpeed(abs(speed))
    if direction == "FORWARD":
        # MQTT message
        message = "FORWARD"
        mqttc.publish(topic, message)
        dc_motor.run(Raspi_MotorHAT.BACKWARD)  # Adjusted motor direction
    elif direction == "BACKWARD":
        # MQTT message
        message = "BACKWARD"
        mqttc.publish(topic, message)
        dc_motor.run(Raspi_MotorHAT.FORWARD)   # Adjusted motor direction
    else:
        # MQTT message
        message = "STOP"
        mqttc.publish(topic, message)
        dc_motor.run(Raspi_MotorHAT.RELEASE)

def pid_control(error, integral, Kp, Ki, Tc):
    integral += error * Tc
    integral = max(min(integral, integral_max), -integral_max)  # Prevent integral windup
    control = Kp * error + Ki * integral
    return control, integral

def stop_car():
    set_dc_motor(0, "STOP")
    set_servo_position(servo_mid)
    sense.clear()


def sense_led(color):
    sense.clear()
    if color == "GREEN":
        for y in range(8):
            for x in range(8):
                sense.set_pixel(x, y, 0, 100, 0)
    elif color == "RED":
        for y in range(8):
            for x in range(8):
                sense.set_pixel(x, y, 100, 0, 0)
    elif color == "YELLOW":
        for y in range(8):
            for x in range(8):
                sense.set_pixel(x, y, 100, 100, 0)

def main():
    global integral_X, integral_Y, integral_Z, previous_error_X, previous_error_Y, previous_error_Z, is_reversing

    try:
        while True:
            frame = picam2.capture_array()
            frame = cv2.resize(frame, (sizeX, sizeY))
            
            # 색상 변환 (BGR -> RGB)로 색상 왜곡 수정
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            frame = cv2.flip(frame, -1)  # Flip vertically and horizontally

            results = pose.process(frame)

            if results.pose_landmarks:
                landmarks = results.pose_landmarks.landmark
                nose = [
                    landmarks[mp_pose.PoseLandmark.NOSE.value].x * frame.shape[1],
                    landmarks[mp_pose.PoseLandmark.NOSE.value].y * frame.shape[0],
                    landmarks[mp_pose.PoseLandmark.NOSE.value].z * rifZ * 2
                ]
                # Draw reference point and nose position with connecting line
                cv2.circle(frame, (int(rifX), int(rifY-100)), 5, (0, 0, 255), -1)  # Reference point
                cv2.circle(frame, (int(nose[0]), int(nose[1])), 5, (0, 255, 0), -1)  # Nose position
                cv2.line(frame, (int(rifX), int(rifY-20)), (int(nose[0]), int(nose[1])), (0, 255, 255), 2)  # Connecting line

                # Calculate errors
                error_X = -(rifX - nose[0])
                error_Y = rifY - nose[1]
                error_Z = rifZ + nose[2]
                
            
                # Calculate control values
                uX, integral_X = pid_control(error_X, integral_X, Kp_X, Ki_X, Tc)
                uY, integral_Y = pid_control(error_Y, integral_Y, Kp_Y, Ki_Y, Tc)
                uZ, integral_Z = pid_control(error_Z, integral_Z, Kp_Z, Ki_Z, Tc)

                print(f"uX: {uX}, uY: {uY}, uZ: {uZ}")

                # Get pose action
                pose_action = detect_pose(landmarks)

                # Determine current movement direction (forward/reverse)
                is_reversing = uZ < 20

                # if is_reversing is True: Sense LED GREEN
                if is_reversing == False:
                    sense_led("GREEN")
                else:
                    sense_led("RED")


                # Execute actions based on pose
                if pose_action == "TURN_LEFT":
                    set_servo_position(servo_left, is_reversing)
                    set_dc_motor(dc_motor_speed, "FORWARD")
                    print("Turning Left")
                elif pose_action == "TURN_RIGHT":
                    set_servo_position(servo_right, is_reversing)
                    set_dc_motor(dc_motor_speed, "FORWARD")
                    print("Turning Right")
                elif pose_action == "STOP":
                    set_servo_position(servo_mid)
                    set_dc_motor(0, "STOP")
                    print("Stopping")
                elif pose_action == "SPECIAL":
                    print("Special Action")
                    # Set servo to turn position
                    set_servo_position(servo_left, is_reversing)
                    # Set DC motor to move forward
                    set_dc_motor(dc_motor_speed, "FORWARD")
                    # Wait for 5 seconds
                    time.sleep(5)
                    # Stop the car after 5 seconds
                    set_dc_motor(0, "STOP")
                    set_servo_position(servo_mid)
                elif pose_action == "FORWARD":
                    set_servo_position(servo_mid)
                    set_dc_motor(dc_motor_speed, "FORWARD")
                    print("Moving Forward")
                elif pose_action == "BACKWARD":
                    set_servo_position(servo_mid)
                    set_dc_motor(dc_motor_speed, "BACKWARD")
                    print("Moving Backward")
                else:  # FOLLOW mode
                    # Standard following behavior using PID control
                    if abs(error_X) > 50:  # Turning threshold
                        if error_X < 0:
                            set_servo_position(servo_right, is_reversing)
                        else:
                            set_servo_position(servo_left, is_reversing)
                    else:
                        set_servo_position(servo_mid)

                    if(uZ > 20 and uZ < 50):
                        set_dc_motor(dc_motor_speed, "STOP")
                        print("Maintaining Position")

                    if(uZ > 50):
                        set_dc_motor(dc_motor_speed, "FORWARD")
                        print("Moving Forward")
                    
                    if(uZ < 20):
                        set_dc_motor(dc_motor_speed, "BACKWARD")
                        print("Moving Backward")

                    # if abs(uZ) > 20:  # Forward/Backward threshold
                    #     if uZ > 0:
                    #         set_dc_motor(dc_motor_speed, "FORWARD")
                    #         print("Moving Forward")
                    #     else:
                    #         set_dc_motor(dc_motor_speed, "BACKWARD")
                    #         print("Moving Backward")
                    # else:
                    #     set_dc_motor(0, "STOP")
                    #     print("Maintaining Position")
            else:
                # Stop the car if no person is detected
                stop_car()

            # Draw landmarks
            mp_drawing.draw_landmarks(
                frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=mp_drawing_styles.get_default_pose_landmarks_style()
            )

            # Add direction indicator
            direction_text = "REVERSE" if is_reversing else "FORWARD"
            cv2.putText(frame, f"Direction: {direction_text}", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

            cv2.imshow('RC Car Pose Following', frame)
            
            # 여기서 streaming frame 전송
            

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        stop_car()
        picam2.close()
        # connect End
        mqttc.loop_stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
