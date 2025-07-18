#expander.py
import logging
from i2c_devices import configure_mcp23017

MAX_INPUTS = 8
MAX_OUTPUTS = 8

class Expander:
    def __init__(self, i2c, addr, name):
        self.i2c = i2c
        self.addr = addr
        self.name = name
        self.inputs = {}   # logical_name: (pin_number, pullup)
        self.outputs = {}  # logical_name: (pin_number, initial_value)
        self.dev = None

    def input(self, pin, logical_name, *, pullup=False):
        if logical_name in self.inputs or logical_name in self.outputs:
            raise ValueError(f"Duplicate logical name '{logical_name}' on {self.name}")
        if len(self.inputs) >= self.MAX_INPUTS:
            raise ValueError(f"Too many inputs on {self.name} (max {self.MAX_INPUTS})")
        self.inputs[logical_name] = (pin, pullup)

    def output(self, pin, logical_name, *, initial=False):
        if logical_name in self.outputs or logical_name in self.inputs:
            raise ValueError(f"Duplicate logical name '{logical_name}' on {self.name}")
        if len(self.outputs) >= self.MAX_OUTPUTS:
            raise ValueError(f"Too many outputs on {self.name} (max {self.MAX_OUTPUTS})")
        self.outputs[logical_name] = (pin, initial)
    
    def configure(self):
        if self.dev:
            return
        
        self.dev = configure_mcp23017(self.i2c, self.addr)
        if self.dev:
            for name, (pin, pullup) in self.inputs.items():
                p = self.dev.get_pin(pin)
                p.direction = Direction.INPUT
                p.pullup = pullup
            for name, (pin, initial) in self.outputs.items():
                p = self.dev.get_pin(pin)
                p.direction = Direction.OUTPUT
                p.value = 1 if initial else 0

    def read_all(self):
        if not self.dev:
            return {name: None for name in self.inputs}
        return {name: self.dev.get_pin(pin).value
                for name, (pin, _) in self.inputs.items()}

    def write(self, logical_name, value):
        if self.dev and logical_name in self.outputs:
            pin, _ = self.outputs[logical_name]
            self.dev.get_pin(pin).value = 1 if value else 0
