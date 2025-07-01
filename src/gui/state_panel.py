#!/us/bin/env python3
from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QPlainTextEdit, QSizePolicy

class StatePanel(QWidget):
    def __init__(self):
        super().__init__()

        self.inputs = QPlainTextEdit()
        self.inputs.setReadOnly(True)
        self.inputs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        self.outputs = QPlainTextEdit()
        self.outputs.setReadOnly(True)
        self.outputs.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # Side-by-side layout
        io_layout = QHBoxLayout()
        io_layout.addWidget(self.inputs)
        io_layout.addWidget(self.outputs)

        # Main layout
        main_layout = QVBoxLayout()
        main_layout.addLayout(io_layout)
        self.setLayout(main_layout)

    def update_state(self, state):
        input_items = {k: v for k, v in state.items() if k.startswith("i_")}
        output_items = {k: v for k, v in state.items() if k.startswith("o_")}
        env_items = {k: v for k, v in state.items() if k in ("i_airtemp", "i_humidity")}

        # Combine inputs and environment
        input_text = "\n".join(f"{k}: {v}" for k, v in input_items.items())
        env_text = "\n".join(f"{k}: {v}" for k, v in env_items.items())
        full_input_text = input_text + ("\n\n" + env_text if env_text else "")

        self.inputs.setPlainText(full_input_text)
        self.outputs.setPlainText("\n".join(f"{k}: {v}" for k, v in output_items.items()))
