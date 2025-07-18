#ambient.py
from i2c_devices import configure_bme280

class Ambient:
    def __init__(self, i2c, addr=0x76, name=None):
        self.i2c = i2c
        self.addr = addr
        self.name = name or f"BME280@{hex(addr)}"
        self.dev = None
        self.inputs = {}  # logical_name -> measurement_key ('temperature', 'humidity', 'pressure')

    def configure(self):
        if self.dev:
            return
        self.dev = configure_bme280(self.i2c, self.addr)

    def input(self, channel, logical_name):
        # channel is one of 'temperature', 'humidity', 'pressure'
        self.inputs[logical_name] = channel

    def read(self):
        if not self.dev:
            return {name: None for name in self.inputs}

        result = {}
        for name, attr in self.inputs.items():
            try:
                result[name] = getattr(self.dev, attr)
            except AttributeError:
                result[name] = None
        return result