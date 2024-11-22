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

        leftAngle = self.calculate_angle(leftShoulder, leftElbow, leftWrist)
        rightAngle = self.calculate_angle(rightShoulder, rightElbow, rightWrist)
        leftShoulderAngle = self.calculate_angle(rightShoulder, leftShoulder, leftElbow)
        rightShoulderAngle = self.calculate_angle(leftShoulder, rightShoulder, rightElbow)

        if (rightAngle > 70 and rightAngle < 110 and
            leftAngle > 150 and leftShoulderAngle > 150):
            return "FORWARD"
        elif (leftAngle > 70 and leftAngle < 110 and
              rightAngle > 150 and rightShoulderAngle > 150):
            return "BACKWARD"
        elif (rightAngle > 80 and rightAngle < 100 and
              leftAngle > 80 and leftAngle < 100 and
              leftShoulderAngle > 150 and rightShoulderAngle > 150):
            return "STOP"
        elif (rightAngle > 150 and leftAngle > 150 and
              leftShoulderAngle > 150 and rightShoulderAngle > 150):
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