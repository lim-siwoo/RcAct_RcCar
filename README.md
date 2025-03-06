# RcAct_RcCar
SSAFY 12기 실시간 자세 인식 기반 자율주행 RC카 프로젝트

<div align="center">
  <img src="https://img.shields.io/badge/Python-3776AB?style=for-the-badge&logo=python&logoColor=white">
  <img src="https://img.shields.io/badge/Linux-FCC624?style=for-the-badge&logo=linux&logoColor=black">
  <img src="https://img.shields.io/badge/Flask-000000?style=for-the-badge&logo=flask&logoColor=white">
  <img src="https://img.shields.io/badge/MQTT-3C5280?style=for-the-badge&logo=eclipsemosquitto&logoColor=white">
  <img src="https://img.shields.io/badge/Raspberry%20Pi-A22846?style=for-the-badge&logo=raspberrypi&logoColor=white">
  <img src="https://img.shields.io/badge/Node--RED-8F0000?style=for-the-badge&logo=nodered&logoColor=white">
  <img src="https://img.shields.io/badge/AWS%20EC2-FF9900?style=for-the-badge&logo=amazonec2&logoColor=white">
</div>

## 📽️ 프로젝트 데모 영상

[![프로젝트 데모 영상](https://img.youtube.com/vi/Mb7SEpPNLkQ/0.jpg)](https://youtu.be/Mb7SEpPNLkQ?si=nFUtwLtL93wUg7lg)

## 📋 프로젝트 개요

### 기간
2024.11 ~ 2024.12 (삼성청년소프트웨어아카데미)

### 주요 기능
- 인체 자세 인식을 통한 RC카 실시간 제어
- 웹 인터페이스를 통한 실시간 영상 스트리밍
- MQTT 기반 IoT 기기 간 통신
- Node-RED 대시보드를 통한 시스템 모니터링 및 원격 제어
- ChatGPT API 연동 시스템 로그 자동 분석
- ChatGPT를 이용한 음성 명령 인식 및 실행 기능

## 🔍 주요 기술적 도전과 해결책

### 1. 음성 명령 인식 및 실행
- ChatGPT API를 활용한 자연어 음성 명령 처리
- 음성을 텍스트로 변환 후 의도 분석 및 RC카 제어 명령으로 해석
- 복잡한 명령어(예: "3미터 앞으로 가서 오른쪽으로 회전해")도 처리 가능
- 결과: 직관적이고 자연스러운 사용자 인터페이스 구현

### 2. 온디바이스 AI 최적화
라즈베리파이5의 제한된 성능과 과열 문제를 해결하기 위해:
- 카메라 해상도를 480x270으로 최적화
- 프레임 레이트를 초당 3프레임으로 제한
- 결과: 시스템 안정성 30% 향상, AI 정확도 유지

### 2. 병렬 처리 아키텍처
동시에 여러 작업을 처리하기 위한 설계:
- Python 멀티프로세싱 라이브러리 활용
- AI 연산 프로세스와 Flask 웹 스트리밍 프로세스 분리
- 결과: 시스템 반응성 및 안정성 향상

### 3. 모듈화된 코드 설계
- 기능별 모듈 분리 (카메라, 모터 제어, 통신, UI 등)
- 확장성 및 유지보수성 향상
- 결과: 개발 효율성 증가 및 버그 감소

## ⚙️ 시스템 아키텍처

```
+------------------+     +------------------+     +--------------------+
|  라즈베리파이5   |     |   MQTT 브로커    |     |  Node-RED 서버     |
|  (RC카 제어)     |<--->|   (AWS EC2)      |<--->|  (대시보드 UI)     |
+------------------+     +------------------+     +--------------------+
        |                        |                        |
        v                        v                        v
+------------------+     +------------------+     +--------------------+
|  Flask 웹 서버   |     | Firebase 데이터베이스 |     |  ChatGPT API      |
|  (영상 스트리밍) |     | (로그 저장)      |     |  (로그 분석)       |
+------------------+     +------------------+     +--------------------+
```

## 🛠️ 기술 스택

| 분야 | 기술 |
|------|------|
| **하드웨어** | Raspberry Pi 5, 카메라 모듈, 모터 HAT |
| **OS** | Linux (Ubuntu) |
| **언어** | Python |
| **AI/비전** | MediaPipe, OpenCV |
| **웹 서버** | Flask |
| **통신** | MQTT 프로토콜 |
| **모니터링** | Node-RED 대시보드 |
| **클라우드** | AWS EC2, Firebase Realtime DB |
| **AI 분석** | ChatGPT API |
| **음성 인식** | ChatGPT API, 음성-텍스트 변환 |

## 📦 설치 및 실행 방법

### 1. 필수 하드웨어
- Raspberry Pi 5
- Pi Camera Module
- Adafruit Motor HAT
- SenseHAT (선택 사항, LED 표시용)

### 2. 소프트웨어 설치
```bash
# 1. 저장소 클론
git clone https://github.com/yourusername/RcAct_RcCar.git
cd RcAct_RcCar

# 2. 가상환경 설정 및 활성화
python3 -m venv myenv
source myenv/bin/activate

# 3. 필수 패키지 설치
pip install -r requirements.txt
```

### 3. 환경 설정
```bash
# config.py 파일 편집
nano config.py

# MQTT 브로커 주소, 카메라 설정 등을 환경에 맞게 수정
```

### 4. 실행
```bash
# 메인 프로그램 실행
python main.py

# 웹 인터페이스 접속
# 브라우저에서 http://라즈베리파이IP:5000 접속
```

## 🧩 시스템 모듈 구조

- **config.py**: 모든 설정값 중앙 집중화
- **hardware/**: 하드웨어 제어 모듈
  - **motor_controller.py**: DC 모터 및 서보 모터 제어
  - **sense_hat_controller.py**: SenseHAT LED 및 센서 처리
  - **camera_controller.py**: Pi Camera 제어
- **pose/**: 자세 인식 관련 모듈
  - **pose_detector.py**: MediaPipe 활용 포즈 감지
- **voice/**: 음성 인식 관련 모듈
  - **voice_command.py**: 음성 명령 처리 및 해석
  - **chatgpt_client.py**: ChatGPT API 연동
- **communication/**: 통신 모듈
  - **mqtt_handler.py**: MQTT 통신 처리
- **web/**: 웹 인터페이스 관련 모듈
  - **app.py**: Flask 웹서버 및 비디오 스트리밍
- **main.py**: 메인 시스템 통합 및 실행

## 🎮 제어 명령어

### 자세 인식 명령

| 자세 | 동작 |
|------|------|
| 오른팔 수평 굽히기 | 전진 |
| 왼팔 수평 굽히기 | 후진 |
| 양팔 수평 굽히기 | 정지 및 사진 촬영 |
| 양팔 위로 뻗기 | 특수 동작 (회전) |
| 기타 자세 | 사람 추적 모드 |

### 음성 명령 예시

| 음성 명령 | 동작 |
|----------|------|
| "앞으로 가줘" | 전진 |
| "뒤로 가" | 후진 |
| "멈춰" | 정지 |
| "오른쪽으로 돌아" | 우회전 |
| "3미터 앞으로 간 다음 왼쪽으로 돌아" | 복합 동작 실행 |
| "사진 찍어줘" | 사진 촬영 |

## 📈 성능 분석

| 최적화 전 | 최적화 후 | 개선 효과 |
|-----------|-----------|----------|
| FPS: 3 | FPS: 30 | CPU 사용량 70% 감소 |
| 해상도: 1920X1080 | 해상도: 480x270 | 메모리 사용량 70% 감소 |
| 시스템 과열 발생 | 안정적 동작 | 안정성 80% 향상 |

## 🔜 향후 개선 계획

- [ ] 더 다양한 자세 인식 명령어 추가
- [ ] 강화학습을 통한 자율주행 기능 개발
- [ ] 모바일 앱 인터페이스 개발
- [ ] 장애물 감지 및 회피 기능 추가

