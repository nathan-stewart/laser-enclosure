#!/usr/bin/env python3

import sys
from PyQt5.QtWidgets import QApplication, QWidget, QHBoxLayout
from gui.hal_client import HALClient
from gui.state_panel import StatePanel
from gui.camera_widget import CameraWidget
from gui.button_panel import ButtonPanel

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Laser Enclosure Monitor")
        self.setFixedSize(800, 480)  # <== Force display size

        self.layout = QHBoxLayout()
        self.button_panel = ButtonPanel()
        self.state_panel = StatePanel()
        self.camera = CameraWidget()

        self.layout.addWidget(self.button_panel)
        self.layout.addWidget(self.state_panel)
        self.layout.addWidget(self.camera)

        self.setLayout(self.layout)

        # HAL connection
        self.hal = HALClient()
        self.hal.state_updated.connect(self.state_panel.update_state)

    def closeEvent(self, event):
        self.camera.shutdown()
        event.accept()

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
