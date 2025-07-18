# mock_devices.py

class MockMCP23017:
    def __init__(self, i2c=None, address=0x20):
        self.address = address
        self.pin_states = [False] * 16

    def get_pin(self, pin):
        return MockPin(self.pin_states, pin)
    
    def configure():
        pass # do nothing in mock case

class MockPin:
    def __init__(self, state_array, pin_number):
        self.state_array = state_array
        self.pin_number = pin_number
        self.direction = None
        self.pullup = False

    def switch_to_input(self, pull=False):
        self.direction = "input"
        self.pullup = pull

    def switch_to_output(self, value=False):
        self.direction = "output"
        self.state_array[self.pin_number] = value

    def value(self):
        return self.state_array[self.pin_number]

    def set_value(self, val):  # optional for manual testing
        self.state_array[self.pin_number] = val


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