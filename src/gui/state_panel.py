#!/us/bin/env python3
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QSizePolicy, QProgressBar, QScrollArea, QFrame
)
from PyQt5.QtCore import Qt

class StatePanel(QWidget):
    def __init__(self):
        super().__init__()

        self.labels = {}
        self.env_bars = {}

        layout = QVBoxLayout()

        # Scrollable area for all labels
        scroll_area = QScrollArea()
        scroll_widget = QWidget()
        self.label_layout = QVBoxLayout(scroll_widget)
        scroll_area.setWidget(scroll_widget)
        scroll_area.setWidgetResizable(True)

        layout.addWidget(scroll_area)
        self.setLayout(layout)

    def update_state(self, state):
        for k, v in state.items():
            if k not in self.labels:
                lbl = QLabel()
                lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
                self.label_layout.addWidget(lbl)
                self.labels[k] = lbl
            lbl = self.labels[k]
            display = '✔' if v else '✘' if v is not None else '—'
            lbl.setText(f"{k}: {display}")
            lbl.setStyleSheet(f"color: {'green' if v else 'red' if v is not None else 'gray'};")
