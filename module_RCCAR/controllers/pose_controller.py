import mediapipe as mp
import numpy as np
import cv2

class PoseController:
    def __init__(self):
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5)
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles

    def calculate_angle(self, a, b, c):
        a = np.array(a)
        b = np.array(b)
        c = np.array(c)
        
        radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
        angle = np.abs(radians * 180.0 / np.pi)
        
        if angle > 180.0:
            angle = 360 - angle
        
        return angle

    def detect_pose(self, landmarks):
                leftShoulder = [landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,
                            landmarks[self.mp_pose.PoseLandmark.LEFT_SHOULDER.value].y]
                leftElbow = [landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value].x,
                            landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value].y]
                leftWrist = [landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value].x,
                            landmarks[self.mp_pose.PoseLandmark.LEFT_WRIST.value].y]
                
                rightShoulder = [landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,
                                landmarks[self.mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y]
                rightElbow = [landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value].x,
                            landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value].y]
                rightWrist = [landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value].x,
                            landmarks[self.mp_pose.PoseLandmark.RIGHT_WRIST.value].y]

                # 각도 계산
                leftAngle = self.calculate_angle(leftShoulder, leftElbow, leftWrist)
                rightAngle = self.calculate_angle(rightShoulder, rightElbow, rightWrist)
                leftShoulderAngle = self.calculate_angle(rightShoulder, leftShoulder, leftElbow)
                rightShoulderAngle = self.calculate_angle(leftShoulder, rightShoulder, rightElbow)

                # 손목의 y좌표가 팔꿈치보다 낮은지 확인 (자연스러운 팔 내림 자세)
                left_arm_down = leftWrist[1] > leftElbow[1]
                right_arm_down = rightWrist[1] > rightElbow[1]
                
                # 손목이 어깨보다 위에 있는지 확인 (만세 자세)
                left_arm_up = leftWrist[1] < leftShoulder[1]
                right_arm_up = rightWrist[1] < rightShoulder[1]

                # IDLE 상태 감지: 양팔이 자연스럽게 내려져 있고, 팔꿈치가 약간 구부러진 상태
                if (left_arm_down and right_arm_down and
                    160 < leftAngle and 160 < rightAngle and
                    80 < leftShoulderAngle < 110 and 80 < rightShoulderAngle < 110):
                    print("IDLE")
                    return "IDLE"
                
                # 만세 자세 감지: 양팔을 완전히 위로 들어올린 상태
                elif (left_arm_up and right_arm_up and
                     leftAngle > 150 and rightAngle > 150 and
                    leftShoulderAngle > 80 and rightShoulderAngle > 80 and leftShoulderAngle < 110 and rightShoulderAngle < 110):
                    print("YIPPEE")
                    return "YIPPEE"
                
                # 기존의 다른 포즈 감지
                elif (rightAngle > 70 and rightAngle < 110 and
                    leftAngle > 150 and leftShoulderAngle > 150):
                    print("RIGHT")
                    return "RIGHT"
                elif (leftAngle > 70 and leftAngle < 110 and
                    rightAngle > 150 and rightShoulderAngle > 150):
                    print("LEFT")
                    return "LEFT"
                elif (rightAngle > 80 and rightAngle < 100 and
                    leftAngle > 80 and leftAngle < 100 and
                    leftShoulderAngle > 150 and rightShoulderAngle > 150):
                    return "STOP"
                elif (rightAngle > 150 and leftAngle > 150 and
                    leftShoulderAngle > 150 and rightShoulderAngle > 150):
                    print("SPECIAL")
                    return "SPECIAL"
                else:
                    return "FOLLOW"
                
    def process_frame(self, frame):
        results = self.pose.process(frame)
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(
                frame,
                results.pose_landmarks,
                self.mp_pose.POSE_CONNECTIONS,
                landmark_drawing_spec=self.mp_drawing_styles.get_default_pose_landmarks_style()
            )
        return results