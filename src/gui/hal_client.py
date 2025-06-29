#!/us/bin/env python3
from PyQt5.QtCore import QObject, pyqtSignal, QTimer
import zmq
import json

class HALClient(QObject):
    state_updated = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.ctx = zmq.Context()
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect("tcp://localhost:5556")  # or whatever port
        self.sub.setsockopt_string(zmq.SUBSCRIBE, '')

        self.timer = QTimer()
        self.timer.timeout.connect(self.poll)
        self.timer.start(100)

    def poll(self):
        while self.sub.poll(timeout=0):
            msg = self.sub.recv_json()
            if msg.get("type") == "state":
                self.state_updated.emit(msg["state"])
