#!/us/bin/env python3
from PyQt5.QtWidgets import (
    QWidget, QVBoxLayout, QLabel, QSizePolicy, QProgressBar, QGridLayout
)
from PyQt5.QtCore import Qt

class StatePanel(QWidget):
    def __init__(self):
        super().__init__()

        self.labels = {}

        layout = QVBoxLayout()

        # Grid layout for label display
        self.label_grid = QGridLayout()
        layout.addLayout(self.label_grid)
        self.setLayout(layout)

    def update_state(self, state):
        row, col = 0, 0
        sorted_items = sorted(state.items(), key=lambda x: x[0])
        for k, v in sorted_items:
            if k not in self.labels:
                lbl = QLabel()
                lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
                self.labels[k] = lbl
                self.label_grid.addWidget(lbl, row, col)
                col += 1
                if col > 1:
                    col = 0
                    row += 1
            lbl = self.labels[k]
            display = '✔' if v else '✘' if v is not None else '—'
            lbl.setText(f"{k}: {display}")
            lbl.setStyleSheet(f"color: {'green' if v else 'red' if v is not None else 'gray'};")
