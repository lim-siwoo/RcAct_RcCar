from Raspi_MotorHAT import Raspi_MotorHAT
from Raspi_PWM_Servo_Driver import PWM
import cv2
import mediapipe as mp
import numpy as np
from time import sleep

# Motor and Servo setup
mh = Raspi_MotorHAT(addr=0x6f)
dc_motor = mh.getMotor(2)  # DC 모터는 전후 방향 제어에 사용
servo = PWM(0x6f)           # Servo 모터는 좌우 방향 제어에 사용
servo.setPWMFreq(60)        # Servo PWM 주파수 설정
servo_mid = 370             # Servo 중간 위치 값
servo_range_min = 270       # Servo 좌측 최대 회전 제한
servo_range_max = 470       # Servo 우측 최대 회전 제한

# DC motor 기본 속도 설정
dc_motor_speed = 100

# MediaPipe Pose setup
mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils

def set_dc_motor(speed, direction):
    """DC 모터를 특정 속도와 방향으로 설정합니다."""
    dc_motor.setSpeed(abs(speed))
    if direction == "FORWARD":
        dc_motor.run(Raspi_MotorHAT.FORWARD)
    elif direction == "BACKWARD":
        dc_motor.run(Raspi_MotorHAT.BACKWARD)
    else:
        dc_motor.run(Raspi_MotorHAT.RELEASE)

def set_servo_position(position):
    """서보 모터의 좌우 방향을 설정합니다."""
    position = max(servo_range_min, min(position, servo_range_max))  # 제한 범위 내에서 위치 설정
    servo.setPWM(0, 0, position)

def detect_pose(landmarks):
    """사용자의 포즈를 감지하고 방향을 결정합니다."""
    left_shoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,
                     landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
    left_elbow = [landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].x,
                  landmarks[mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
    left_wrist = [landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].x,
                  landmarks[mp_pose.PoseLandmark.LEFT_WRIST.value].y]
    right_shoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,
                      landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
    right_elbow = [landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,
                   landmarks[mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
    right_wrist = [landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].x,
                   landmarks[mp_pose.PoseLandmark.RIGHT_WRIST.value].y]

    # 왼팔 및 오른팔 각도 계산
    left_angle = calculate_angle(left_shoulder, left_elbow, left_wrist)
    right_angle = calculate_angle(right_shoulder, right_elbow, right_wrist)

    # 포즈에 따른 동작 결정
    if 70 < right_angle < 110 and left_angle > 150:
        return "LEFT"  # 좌회전
    elif 70 < left_angle < 110 and right_angle > 150:
        return "RIGHT"  # 우회전
    elif right_angle > 150 and left_angle > 150:
        return "FORWARD"  # 전진
    else:
        return "STOP"

def calculate_angle(a, b, c):
    """각도를 계산하는 함수"""
    a, b, c = np.array(a), np.array(b), np.array(c)
    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)
    return 360 - angle if angle > 180 else angle

# Video capture 및 MediaPipe Pose
cap = cv2.VideoCapture(0)
with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
    while cap.isOpened():
        success, frame = cap.read()
        if not success:
            print("카메라에서 영상을 읽을 수 없습니다.")
            break

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = pose.process(frame_rgb)

        if results.pose_landmarks:
            # 랜드마크에서 코 위치 가져오기
            landmarks = results.pose_landmarks.landmark

            # 포즈에 따라 모터 및 서보 모터 조정
            action = detect_pose(landmarks)
            if action == "LEFT":
                set_servo_position(servo_range_min)  # 좌회전
                set_dc_motor(dc_motor_speed, "FORWARD")
            elif action == "RIGHT":
                set_servo_position(servo_range_max)  # 우회전
                set_dc_motor(dc_motor_speed, "FORWARD")
            elif action == "FORWARD":
                set_servo_position(servo_mid)  # 정면
                set_dc_motor(dc_motor_speed, "FORWARD")
            else:
                set_dc_motor(0, "STOP")  # 정지

            # 카메라 영상에 랜드마크 그리기
            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        cv2.imshow('RcCar Pose Control', frame)
        if cv2.waitKey(5) & 0xFF == 27:  # 'Esc' 키로 종료
            break

cap.release()
cv2.destroyAllWindows()
set_dc_motor(0, "STOP")

