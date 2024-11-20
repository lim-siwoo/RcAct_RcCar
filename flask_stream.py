from flask import Flask, render_template, Response
from picamera2 import Picamera2
import cv2

# Flask 앱 생성
app = Flask(__name__)

# Picamera2 초기화
picam2 = Picamera2()
config = picam2.create_video_configuration(main={'size': (1920, 1080)},  # 120도 광각 모드 해상도 설정
    controls={
        'AfMode': 2,               # 자동 초점 모드 활성화 (2는 Continuous 모드)
        'AfRange': 1,              # 광각 범위에 적합하도록 설정
        'LensPosition': 0.0,       # 광각 설정을 위한 렌즈 포지션 (적절한 값으로 설정 필요)
        'FrameRate': 3
    },
    buffer_count=5)

picam2.configure(config)
picam2.start()

# 프레임 생성 함수
def generate_frames():
    while True:
        # 카메라에서 프레임 가져오기
        frame = picam2.capture_array()

        # OpenCV로 영상 처리
        processed_frame = cv2.resize(frame, (480, 270))
        processed_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        # flip vertically
        processed_frame = cv2.flip(processed_frame, -1)

        # JPEG로 인코딩
        _, buffer = cv2.imencode('.jpg', processed_frame)
        frame = buffer.tobytes()

        # 프레임 송출
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')
        

# 라우트 설정
@app.route('/')
def index():
    return render_template('index.html')  # 스트리밍 페이지

@app.route('/video_feed')
def video_feed():
    return Response(generate_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

# 서버 실행
if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
