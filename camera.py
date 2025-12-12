from picamera2 import Picamera2
import cv2

class Camera:
    def __init__(self, width=640, height=480):
        self.picam = Picamera2()
        self.config = self.picam.create_video_configuration(
            main={"size": (width, height), "format": "RGB888"}
        )
        self.picam.configure(self.config)
        self.picam.start()

    def get_frame(self):
        frame = self.picam.capture_array()
        return frame
