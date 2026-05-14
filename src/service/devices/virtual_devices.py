# service/devices/virtual_devices.py

class VirtualInputs:
    def __init__(self, name="VirtualInputs"):
        self.name = name
        self.inputs = {}
        self._state = {}

    def input(self, logical_name, initial=0):
        if logical_name in self.inputs:
            raise ValueError(f"Duplicate virtual input '{logical_name}'")
        self.inputs[logical_name] = logical_name
        self._state[logical_name] = initial

    def configure(self):
        return self

    def read(self):
        for name, value in self._state.items():
            yield name, value

    def write(self, logical_name, value):
        if logical_name not in self._state:
            raise KeyError(f"Unknown virtual input '{logical_name}'")
        self._state[logical_name] = value

    def set(self, logical_name, value):
        self.write(logical_name, value)