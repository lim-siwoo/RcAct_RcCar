# hardware/camera.py
from picamera2 import Picamera2
import cv2

class CameraManager:
    def __init__(self, size_x, size_y):
        self.size_x = size_x
        self.size_y = size_y
        self.picam2 = Picamera2()
        self._setup_camera()

    def _setup_camera(self):
        camera_config = self.picam2.create_video_configuration(
            main={'size': (1920, 1080)},
            controls={
                'AfMode': 2,
                'AfRange': 1,
                'LensPosition': 0.0,
                'FrameRate': 3
            },
            buffer_count=2
        )
        self.picam2.configure(camera_config)
        self.picam2.start()

    def capture_frame(self):
        frame = self.picam2.capture_array()
        frame = cv2.resize(frame, (self.size_x, self.size_y))
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        return cv2.flip(frame, -1)

    def close(self):
        self.picam2.close()

    def capture_image(self, filename):
        self.picam2.capture_file(filename)
