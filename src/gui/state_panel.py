#!/us/bin/env python3
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QLabel

class StatePanel(QWidget):
    def __init__(self):
        super().__init__()
        self.layout = QVBoxLayout()
        self.inputs = QLabel("Inputs: —")
        self.outputs = QLabel("Outputs: —")
        self.env = QLabel("Environment: —")

        self.layout.addWidget(self.inputs)
        self.layout.addWidget(self.outputs)
        self.layout.addWidget(self.env)
        self.setLayout(self.layout)

    def update_state(self, state):
        self.inputs.setText(f"Inputs: {state.get('inputs')}")
        self.outputs.setText(f"Outputs: {state.get('outputs')}")
        self.env.setText(f"Env: {state.get('env')}")
