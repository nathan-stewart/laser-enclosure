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
        items = state.items()
        for k, v in items:
            if k not in self.labels:
                lbl = QLabel()
                lbl.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
                self.labels[k] = lbl
                self.label_grid.addWidget(lbl, row, col)
                row += 1
                if row >= len(items) // 2:
                    row = 0
                    col += 1
            lbl = self.labels[k]
            if k in ['i_air_supply', 'i_co2_supply']:
                display = f"{v:.2f} V" if v is not None else f"{k}: —"
            elif k in ['i_ambient_temp', 'i_ambient_humidity', 'i_ambient_pressure']:
                display = f"{v:.2f}" if v is not None else f"{k}: —"
            else:
                display = '✔' if v else '✘' if v is not None else '—'
            lbl.setText(f"{display}:{k}")
            lbl.setStyleSheet(f"color: {'green' if v else 'red' if v is not None else 'gray'};")

