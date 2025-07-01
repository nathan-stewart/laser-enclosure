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
        self.sub.connect("tcp://127.0.0.1:5556")  # or whatever port
        self.sub.setsockopt_string(zmq.SUBSCRIBE, 'hal')

        self.timer = QTimer()
        self.timer.timeout.connect(self.poll)
        self.timer.start(100)


    def poll(self):
        try:
            while self.sub.poll(timeout=10):
                topic, payload = self.sub.recv_multipart()
                if topic == b'hal':
                    msg = json.loads(payload.decode())
                    if 'state' in msg:
                        self.state_updated.emit(msg["state"])
        except zmq.ZMQError as e:
            print("ZMQ Error:", e)
