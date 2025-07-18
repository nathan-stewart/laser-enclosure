#adc.py
class Adc:
    def __init__(self, i2c, addr=0x48, name=None):
        self.i2c = i2c
        self.addr = addr
        self.name = name or f"ADS@{hex(addr)}"
        self.inputs = {}  # logical_name -> channel
        self.dev = None

    def configure(self):
        self.dev = configure_ads1115(self.i2c, self.addr)

    def input(self, channel, logical_name):
        self.inputs[logical_name] = channel

    def read(self):
        if not self.dev:
            return {name: None for name in self.inputs}
        return {
            name: self.dev.read_voltage(channel)
            for name, channel in self.inputs.items()
        }
