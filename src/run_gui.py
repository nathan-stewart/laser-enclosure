#!/usr/bin/env python3
import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout
from gui.state_panel import StatePanel
from gui.camera_widget import CameraWidget
from gui.hal_client import HALClient

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Laser Enclosure Monitor")
        self.layout = QVBoxLayout()

        # State panel (inputs/outputs/env)
        self.state_panel = StatePanel()
        self.layout.addWidget(self.state_panel)

        # Camera feed
        self.camera = CameraWidget()
        self.layout.addWidget(self.camera)

        # HAL Client
        self.hal = HALClient()
        self.hal.state_updated.connect(self.state_panel.update_state)

        self.setLayout(self.layout)

    def closeEvent(self, event):
        self.camera.shutdown()
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec_())
