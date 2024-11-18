from Raspi_MotorHAT import Raspi_MotorHAT
from Raspi_PWM_Servo_Driver import PWM
import cv2
import mediapipe as mp
from picamera2 import Picamera2

# Motor setup
mh = Raspi_MotorHAT(addr=0x6f)
dc_motor = mh.getMotor(2)  # DC 모터: 전후 방향 제어
dc_motor_speed = 100  # 기본 속도 설정

# Servo setup
servo = PWM(0x6f)
servo.setPWMFreq(60)  # 서보 모터 PWM 주파수 설정
servo_mid = 370  # 서보 모터 중간 위치 (직진 방향)
servo_left = 270  # 서보 모터 왼쪽 최대 회전
servo_right = 470  # 서보 모터 오른쪽 최대 회전

# MediaPipe Pose setup
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils

# Pi Camera 설정
picam2 = Picamera2()
picam2.start()

# 화면 중심 좌표 설정 (기준점)
target_x = 480  # 화면 중심의 x좌표 (960x720 해상도 기준)
tolerance = 100  # 중앙 위치에서의 허용 오차
z_tolerance = 0.04
# 거리 기준 설정 (적정 거리 값)
target_distance = -0.5  # 사용자가 유지해야 할 적정 거리 (z 값)

def set_dc_motor(speed, direction):
    """DC 모터를 특정 속도와 방향으로 설정합니다."""
    dc_motor.setSpeed(abs(speed))
    if direction == "FORWARD":
        dc_motor.run(Raspi_MotorHAT.BACKWARD)
    elif direction == "BACKWARD":
        dc_motor.run(Raspi_MotorHAT.FORWARD)
    else:
        dc_motor.run(Raspi_MotorHAT.RELEASE)

def set_servo_position(position):
    """서보 모터의 좌우 방향을 설정합니다."""
    #servo.setPWM(0, 0, position)

def follow_person(nose_x, nose_z):
    """코의 x, z 위치를 기준으로 방향 및 거리 제어"""
    # 좌우 방향 제어
    if nose_x < target_x - tolerance:
        print("Turning left")
        set_servo_position(servo_left)  # 서보를 왼쪽으로 회전
    elif nose_x > target_x + tolerance:
        print("Turning right")
        set_servo_position(servo_right)  # 서보를 오른쪽으로 회전
    else:
        print("Center aligned")
        set_servo_position(servo_mid)  # 서보를 중앙으로 설정
    print(nose_z)
    # 전후 거리 제어
    if nose_z > target_distance + z_tolerance:
        print("Moving forward")
        set_dc_motor(dc_motor_speed, "FORWARD")  # 전진
    elif nose_z < target_distance - z_tolerance:
        print("Moving backward")
        set_dc_motor(dc_motor_speed, "BACKWARD")  # 후진
    else:
        print("Maintaining distance")
        set_dc_motor(0, "STOP")  # 거리 유지

# 무한 루프 시작
try:
    while True:
        # 카메라 프레임 읽기
        frame = picam2.capture_array()

        # OpenCV에서 BGR로 변환
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        frame = cv2.flip(frame, -1)
       
        # MediaPipe를 사용한 Pose Estimation
        results = pose.process(frame)

        # 추정 결과를 화면에 표시
        if results.pose_landmarks:
            # 랜드마크에서 코 위치 가져오기
            landmarks = results.pose_landmarks.landmark
            nose_x = int(landmarks[mp_pose.PoseLandmark.NOSE.value].x * frame.shape[1])
            nose_z = landmarks[mp_pose.PoseLandmark.NOSE.value].z
            
            # 코 위치를 기준으로 따라가도록 설정
            follow_person(nose_x, nose_z)

            # 화면에 랜드마크를 그려서 표시
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

