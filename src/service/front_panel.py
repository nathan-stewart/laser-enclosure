import sys
import zmq
import threading
import json
import cv2
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QTableWidget, QTableWidgetItem,
    QVBoxLayout, QHBoxLayout, QWidget, QLabel
)
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtCore import Qt, QTimer

class GPIOMonitor(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Laser Enclosure Front Panel")
        self.resize(1000, 600)

        # IO Table
        self.table = QTableWidget(0, 4)
        self.table.setHorizontalHeaderLabels(["Source", "Name", "State", "Details"])

        # Camera View
        self.camera_label = QLabel("Camera initializing...")
        self.camera_label.setAlignment(Qt.AlignCenter)
        self.camera_label.setMinimumSize(320, 240)

        # Layouts
        table_layout = QVBoxLayout()
        table_layout.addWidget(self.table)
        left_widget = QWidget()
        left_widget.setLayout(table_layout)

        main_layout = QHBoxLayout()
        main_layout.addWidget(left_widget, 2)
        main_layout.addWidget(self.camera_label, 3)

        container = QWidget()
        container.setLayout(main_layout)
        self.setCentralWidget(container)

        self.io_states = {}  # {(source, name): (state, details)}

        # Start ZMQ listener in a thread
        self.zmq_thread = threading.Thread(target=self.listen_zmq, daemon=True)
        self.zmq_thread.start()

        # Camera setup
        self.cap = cv2.VideoCapture(0)
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_camera)
        self.timer.start(30)  # ~30 FPS

    def listen_zmq(self):
        ctx = zmq.Context()
        sub = ctx.socket(zmq.SUB)
        sub.connect("tcp://localhost:5556")
        sub.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all topics

        while True:
            try:
                msg = sub.recv_json()
                self.handle_zmq_message(msg)
            except Exception as e:
                print(f"ZMQ error: {e}")

    def handle_zmq_message(self, msg):
        # Parse topic to get source and name
        topic = msg.get("topic", "")
        parts = topic.split("/")
        if len(parts) < 3:
            return
        source, name = parts[1], parts[2]
        state = msg.get("state", "")
        details = json.dumps(msg, indent=2)

        key = (source, name)
        self.io_states[key] = (state, details)
        self.update_table()

    def update_table(self):
        self.table.setRowCount(len(self.io_states))
        for row, ((source, name), (state, details)) in enumerate(self.io_states.items()):
            self.table.setItem(row, 0, QTableWidgetItem(source))
            self.table.setItem(row, 1, QTableWidgetItem(name))
            self.table.setItem(row, 2, QTableWidgetItem(state))
            self.table.setItem(row, 3, QTableWidgetItem(details))

    def update_camera(self):
        ret, frame = self.cap.read()
        if ret:
            # Convert the image from BGR to RGB format
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # Convert the image to QImage format
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qimage = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            # Set the QImage to the label
            self.camera_label.setPixmap(QPixmap.fromImage(qimage))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    win = GPIOMonitor()
    win.show()
    sys.exit(app.exec_())