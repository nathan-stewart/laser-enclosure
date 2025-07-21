# mock_devices.py
class MockGpioController:
    def __init__(self):
        self._inputs = {}
        self._outputs = {}
        self._state = {}

    def input(self, bcm, name, pullup=True):
        self._inputs[name] = bcm
        self._state[name] = False

    def output(self, bcm, name, initial=False):
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

class MockMCP23017:
    DIR_OUT = 1
    DIR_IN = 0
    def __init__(self, address):
        self.address = address
        self.directions = {}  # pin -> DIR_IN or DIR_OUT
        self.states = {}      # pin -> value

    def set_pin_direction(self, pin, direction):
        self.directions[pin] = direction
        if direction == MockMCP23017.DIR_OUT:
            self.states.setdefault(pin, False)

    def digital_write(self, pin, value):
        if self.directions.get(pin) != MockMCP23017.DIR_OUT:
            raise RuntimeError(f"Pin {pin} not configured as output")
        self.states[pin] = value

    def digital_read(self, pin):
        if self.directions.get(pin) != MockMCP23017.DIR_IN:
            raise RuntimeError(f"Pin {pin} not configured as input")
        return self.states.get(pin, False)


class MockADS1115:
    def __init__(self, address=0x48):
        self.address = address
        self.last_value = 1.23  # Simulated voltage

    def read_voltage(self, channel=0):
        return self.last_value


class MockBME280:
    def __init__(self, address=0x76):
        self.temperature = 22.0
        self.humidity = 45.0
        self.pressure = 1013.25

class MockQTEncoder:
    def __init__(self, address=0x36):
        self.encoder_position = 0

    def simulate_turn(self, delta):
        self.encoder_position += delta

