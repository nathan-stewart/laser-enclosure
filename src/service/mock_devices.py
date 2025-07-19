# mock_devices.py

class MockMCP23017:
    def __init__(self, i2c=None, address=0x20):
        self.address = address
        self.pins = [MockPin() for _ in range(16)]

    def get_pin(self, pin):
        return self.pins[pin]
    
class MockPin:
    def __init__(self):
        self.direction = None  # "input" or "output"
        self.pullup = False
        self._value = False

    @property
    def value(self):
        return self._value

    @value.setter
    def value(self, val):
        self._value = bool(val)


class MockADS1115:
    def __init__(self, i2c=None, address=0x48):
        self.address = address
        self.last_value = 1.23  # Simulated voltage

    def read_voltage(self, channel=0):
        return self.last_value


class MockBME280:
    def __init__(self, i2c=None, address=0x76):
        self.temperature = 22.0
        self.humidity = 45.0
        self.pressure = 1013.25

class MockSeeSaw:
    def __init__(self, i2c=None, address=0x36):
        self.encoder_position = 0

    def simulate_turn(self, delta):
        self.encoder_position += delta

class MockGpioController:
    def __init__(self):
        self._inputs = {}
        self._outputs = {}
        self._state = {}

    def input(self, name, bcm, pullup=True):
        self._inputs[name] = bcm
        self._state[name] = False

    def output(self, name, bcm, initial=False):
        self._outputs[name] = bcm
        self._state[name] = initial

    def read(self, name):
        if name not in self._inputs:
            raise ValueError(f"Input pin '{name}' not configured")
        return self._state[name]

    def write(self, name, value):
        if name not in self._outputs:
            raise ValueError(f"Output pin '{name}' not configured")
        self._state[name] = bool(value)

    def set_input(self, name, value):
        """Simulate a pin change in test mode"""
        if name not in self._inputs:
            raise ValueError(f"Input pin '{name}' not configured")
        self._state[name] = bool(value)

    def cleanup(self):
        self._inputs.clear()
        self._outputs.clear()
        self._state.clear()
