import cv2
import mediapipe as mp
from picamera2 import Picamera2

# MediaPipe Pose 설정
mp_pose = mp.solutions.pose
pose = mp_pose.Pose()
mp_drawing = mp.solutions.drawing_utils

# Pi Camera 설정
picam2 = Picamera2()
picam2.start()

# 무한 루프 시작
try:
    while True:
        # 카메라 프레임 읽기
        frame = picam2.capture_array()
        
        # OpenCV에서 BGR로 변환
        frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        
        # MediaPipe를 사용한 Pose Estimation
        results = pose.process(frame)
        
        # 추정 결과를 화면에 표시
        if results.pose_landmarks:
            mp_drawing.draw_landmarks(frame, results.pose_landmarks, mp_pose.POSE_CONNECTIONS)
        
        # 화면에 이미지 표시
        cv2.imshow('Pose Estimation', frame)
        
        # 'q' 키를 눌러 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    picam2.close()
    cv2.destroyAllWindows()

