from Raspi_MotorHAT import Raspi_MotorHAT
from Raspi_PWM_Servo_Driver import PWM
import cv2
import mediapipe as mp
import numpy as np
from picamera2 import Picamera2
import time

# Motor setup
mh = Raspi_MotorHAT(addr=0x6f)
dc_motor = mh.getMotor(2)  # DC 모터: 전후 방향 제어
dc_motor_speed = 100  # 기본 속도 설정

# Servo setup
servo = PWM(0x6f)
servo.setPWMFreq(60)  # 서보 모터 PWM 주파수 설정
servo_mid = 370  # 서보 모터 중간 위치 (직진 방향)
servo_left = 470  # 서보 모터 왼쪽 최대 회전
servo_right = 270  # 서보 모터 오른쪽 최대 회전

# MediaPipe Pose setup
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils

# Pi Camera 설정
picam2 = Picamera2()
camera_config = picam2.create_still_configuration(main={'size': (960, 720)})  # 해상도 설정
picam2.configure(camera_config)
picam2.start()

# 중심 위치 설정 (화면 중심)
rifX = 960 / 2
rifY = 720 / 2
rifZ = 200

# PI 상수 설정
Kp_X = 0.1
Ki_X = 0.0
Kp_Y = 0.2
Ki_Y = 0.0
Kp_Z = 0.4
Ki_Z = 0.0

# 루프 시간
Tc = 0.05

# PI 변수 초기화
integral_X = 0
previous_error_X = 0
integral_Y = 0
previous_error_Y = 0
integral_Z = 0
previous_error_Z = 0

# 서보 모터 설정 함수
def set_servo_position(position):
    position = max(servo_left, min(position, servo_right))
    servo.setPWM(0, 0, position)

# DC 모터 설정 함수
def set_dc_motor(speed, direction):
    dc_motor.setSpeed(abs(speed))
    if direction == "FORWARD":
        dc_motor.run(Raspi_MotorHAT.FORWARD)
    elif direction == "BACKWARD":
        dc_motor.run(Raspi_MotorHAT.BACKWARD)
    else:
        dc_motor.run(Raspi_MotorHAT.RELEASE)

# PID 제어 함수
def pid_control(error, integral, Kp, Ki, Tc):
    integral += error * Tc
    control = Kp * error + Ki * integral
    return control, integral

# 무한 루프 시작
try:
    while True:
        # 카메라 프레임 읽기
        frame = picam2.capture_array()
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.flip(frame, -1)  # 카메라 반전

        # MediaPipe를 사용한 Pose Estimation
        results = pose.process(frame)

        if results.pose_landmarks:
            # 랜드마크에서 코 위치 가져오기
            landmarks = results.pose_landmarks.landmark
            nose = [
                landmarks[mp_pose.PoseLandmark.NOSE.value].x * frame.shape[1],
                landmarks[mp_pose.PoseLandmark.NOSE.value].y * frame.shape[0],
                landmarks[mp_pose.PoseLandmark.NOSE.value].z * rifZ * 2
            ]

            # 오류 계산 (X, Y, Z)
            error_X = -(rifX - nose[0])
            error_Y = rifY - nose[1]
            error_Z = rifZ + nose[2]

            # PID 제어값 계산
            uX, integral_X = pid_control(error_X, integral_X, Kp_X, Ki_X, Tc)
            uY, integral_Y = pid_control(error_Y, integral_Y, Kp_Y, Ki_Y, Tc)
            uZ, integral_Z = pid_control(error_Z, integral_Z, Kp_Z, Ki_Z, Tc)

            # 방향 제어
            if error_X < -50:
                print("Turning right")
                set_servo_position(servo_right)
            elif error_X > 50:
                print("Turning left")
                set_servo_position(servo_left)
            else:
                print("Center aligned")
                set_servo_position(servo_mid)

            print(uZ)

            # 전후 거리 제어
            if uZ > 0:
                print("Moving forward")
                set_dc_motor(dc_motor_speed, "FORWARD")
            elif uZ < 0:
                print("Moving backward")
                set_dc_motor(dc_motor_speed, "BACKWARD")
            else:
                print("Maintaining distance")
                set_dc_motor(0, "STOP")

            # 화면에 랜드마크 그리기
            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)

        # 화면에 이미지 표시
        cv2.imshow('Pose Following RcCar', frame)

        # 'q' 키를 눌러 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    set_dc_motor(0, "STOP")  # 프로그램 종료 시 모터 정지
    set_servo_position(servo_mid)  # 종료 시 서보를 중앙으로 설정
    picam2.close()
    cv2.destroyAllWindows()

