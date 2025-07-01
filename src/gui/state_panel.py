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
        input_items = {k: v for k, v in state.items() if k.startswith("i_")}
        output_items = {k: v for k, v in state.items() if k.startswith("o_")}
        env_items = {k: v for k, v in state.items() if k in ("i_airtemp", "i_humidity")}

        self.inputs.setText(f"Inputs: {input_items}")
        self.outputs.setText(f"Outputs: {output_items}")
        self.env.setText(f"Environment: {env_items}")
