#!/usr/bin/env python3

from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel

class ButtonPanel(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()

        self.f1 = QLabel("F1")
        self.f2 = QLabel("F2")
        self.f3 = QLabel("F3")
        self.f4 = QLabel("F4")

        for btn_label in [self.f1, self.f2, self.f3, self.f4]:
            self.layout.addWidget(btn_label)

        self.setLayout(self.layout)
