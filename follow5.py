from Raspi_MotorHAT import Raspi_MotorHAT
from Raspi_PWM_Servo_Driver import PWM
import cv2
import mediapipe as mp
import numpy as np
from picamera2 import Picamera2
import time
from enum import Enum

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
sizeX = 960
sizeY = 720
# Pi Camera setup
picam2 = Picamera2()
camera_config = picam2.create_video_configuration(main={'size': (sizeX, sizeY)}, controls={'FrameRate' : 15})
picam2.configure(camera_config)
picam2.start()

# Reference points
rifX = sizeX / 2
rifY = sizeY / 2
rifZ = 600

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

    if (rightAngle > 70 and rightAngle < 110 and 
        leftAngle > 150 and leftShoulderAngle > 150):
        return "TURN_LEFT"
    elif (leftAngle > 70 and leftAngle < 110 and 
          rightAngle > 150 and rightShoulderAngle > 150):
        return "TURN_RIGHT"
    elif (rightAngle > 80 and rightAngle < 100 and 
          leftAngle > 80 and leftAngle < 100 and 
          leftShoulderAngle > 150 and rightShoulderAngle > 150):
        return "STOP"
    elif (rightAngle > 150 and leftAngle > 150 and 
          leftShoulderAngle > 150 and rightShoulderAngle > 150):
        return "SPECIAL"
    else:
        return "FOLLOW"

def set_servo_position(position, is_reversing=False):
    # 후진 시에는 조향 방향을 반대로 설정
    if is_reversing:
        position = servo_mid + (servo_mid - position)
    position = max(servo_right, min(position, servo_left))
    servo.setPWM(0, 0, position)

def set_dc_motor(speed, direction):
    dc_motor.setSpeed(abs(speed))
    if direction == "FORWARD":
        dc_motor.run(Raspi_MotorHAT.BACKWARD)  # 모터 방향 수정
    elif direction == "BACKWARD":
        dc_motor.run(Raspi_MotorHAT.FORWARD)   # 모터 방향 수정
    else:
        dc_motor.run(Raspi_MotorHAT.RELEASE)

def pid_control(error, integral, Kp, Ki, Tc):
    integral += error * Tc
    control = Kp * error + Ki * integral
    return control, integral

try:
    while True:
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.flip(frame, -1)

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
            cv2.line(frame, (int(rifX), int(rifY-100)), (int(nose[0]), int(nose[1])), (0, 255, 255), 2)  # Connecting line

            # Calculate errors
            error_X = -(rifX - nose[0])
            error_Y = rifY - nose[1]
            error_Z = rifZ + nose[2]

            # Calculate control values
            uX, integral_X = pid_control(error_X, integral_X, Kp_X, Ki_X, Tc)
            uY, integral_Y = pid_control(error_Y, integral_Y, Kp_Y, Ki_Y, Tc)
            uZ, integral_Z = pid_control(error_Z, integral_Z, Kp_Z, Ki_Z, Tc)

            # Get pose action
            pose_action = detect_pose(landmarks)

            # 현재 이동 방향 (전진/후진) 결정
            is_reversing = uZ < 0

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
            else:  # FOLLOW mode
                # Standard following behavior using PID control
                if abs(error_X) > 50:  # Turning threshold
                    if error_X < 0:
                        set_servo_position(servo_right, is_reversing)
                    else:
                        set_servo_position(servo_left, is_reversing)
                else:
                    set_servo_position(servo_mid)

                if abs(uZ) > 20:  # Forward/Backward threshold
                    if uZ > 0:
                        set_dc_motor(dc_motor_speed, "FORWARD")
                        print("Moving Forward")
                    else:
                        set_dc_motor(dc_motor_speed, "BACKWARD")
                        print("Moving Backward")
                else:
                    set_dc_motor(0, "STOP")
                    print("Maintaining Position")

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

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    set_dc_motor(0, "STOP")
    set_servo_position(servo_mid)
    picam2.close()
    cv2.destroyAllWindows()
