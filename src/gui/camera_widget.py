#!/us/bin/env python3
from PyQt5.QtWidgets import QLabel
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import QTimer
import cv2

class CameraWidget(QLabel):
    def __init__(self):
        super().__init__()
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            self.cap = None
        else:
            self.timer = QTimer()
            self.timer.timeout.connect(self.update_camera)
            self.timer.start(30)  # ~30 FPS
            self.setFixedSize(480, 480)  # or whatever size you want
            self.setStyleSheet("background-color: black;")
            self.setText("Camera View")

    def update_frame(self):

        if self.cap.isOpened():
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                h, w, ch = frame.shape
                bytes_per_line = ch * w
                qimg = QImage(frame.data, w, h, bytes_per_line, QImage.Format_RGB888)
                self.setPixmap(QPixmap.fromImage(qimg).scaled(
                    self.width(), self.height(), aspectRatioMode=1))
            else:
                self.setText("Camera read error.")
        else:
            self.setText("Camera not available.")

    def shutdown(self):
        if self.cap and self.cap.isOpened():
            self.cap.release()
