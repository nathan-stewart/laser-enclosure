#encoder.py
from i2c_devices import configure_seesaw

class Encoder:
    def __init__(self, i2c, addr=0x36, name=None):
        self.i2c = i2c
        self.addr = addr
        self.name = name or f"SeeSaw@{hex(addr)}"
        self.dev = None
        self.last_position = 0
        self.delta = 0

    def configure(self):
        self.dev = configure_seesaw(self.i2c, self.addr)
        if self.dev:
            self.last_position = self.dev.encoder_position

    def read_delta(self):
        if not self.dev:
            return 0
        current = self.dev.encoder_position
        delta = current - self.last_position
        self.last_position = current
        return delta
