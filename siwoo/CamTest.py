from picamera2 import Picamera2, Preview
import cv2
import numpy as np

# Picamera2 객체 초기화
picam2 = Picamera2()

# 카메라 설정: 120도 광각 모드를 위한 카메라 구성 설정
camera_config = picam2.create_video_configuration(
    main={'size': (1920, 1080)},  # 120도 광각 모드 해상도 설정
    controls={
        'AfMode': 2,               # 자동 초점 모드 활성화 (2는 Continuous 모드)
        'AfRange': 1,              # 광각 범위에 적합하도록 설정
        'LensPosition': 0.0,       # 광각 설정을 위한 렌즈 포지션 (적절한 값으로 설정 필요)
        'FrameRate': 5
    },
    buffer_count=2
)

# 카메라 설정 구성 적용
picam2.configure(camera_config)
picam2.start()

# OpenCV 창에서 실시간 카메라 출력
try:
    while True:
        # 카메라 이미지 캡처
        frame = picam2.capture_array()

        # 해상도를 낮추지만 동일한 넓은 시야각 유지 (960x540으로 리사이즈)
        resized_frame = cv2.resize(frame, (480, 270))
        
        # 색상 변환 (BGR -> RGB)로 색상 왜곡 수정
        resized_frame = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)
        
        # 이미지 출력을 위한 OpenCV 창
        cv2.imshow("Camera Module 3 Wide Angle - Reduced Resolution", resized_frame)
        
        # 'q' 키를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # 자원 해제
    picam2.stop()
    cv2.destroyAllWindows()

