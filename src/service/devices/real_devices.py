# i2c_devices.py
import sys
import RPi.GPIO as GPIO
import board
import busio
import adafruit_mcp230xx.mcp23017
from adafruit_ads1x15.analog_in import AnalogIn
import adafruit_ads1x15.ads1115
import Adafruit_BME280
from adafruit_seesaw import digitalio, rotaryio, seesaw
from collections import deque

i2c = busio.I2C(board.SCL, board.SDA)

class EMAFilter:
    def __init__(self, alpha=0.2):
        self.alpha = alpha
        self.filtered = None

    def add(self, val):
        if self.filtered is None:
            self.filtered = val
        else:
            self.filtered = self.alpha * val + (1 - self.alpha) * self.filtered
        return self.filtered

class RpiGpio:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self.inputs = {}
        self.outputs = {}
        self.debounce = {}
        self.last_stable = {}

    def __delete__(self):
        GPIO.cleanup()

    def input(self, bcm, name, pullup=True):
        GPIO.setup(bcm, GPIO.IN, pull_up_down=GPIO.PUD_UP if pullup else GPIO.PUD_DOWN)
        self.debounce[name] = deque(maxlen=3)  # Using deque for efficient appending and popping
        self.inputs[name] = bcm
        self.debounce[name] = deque(maxlen=3)  # Initialize debounce queue for this input
        self.last_stable[name] = None

    def output(self, bcm, name, initial=False):
        GPIO.setup(bcm, GPIO.OUT, initial=GPIO.HIGH if initial else GPIO.LOW)
        self.outputs[name] = bcm

    def read(self):
        result = {}
        for name in list(self.inputs.keys()):
            self.debounce[name].append(GPIO.input(self.inputs[name]))
            if all(v == self.debounce[name][0] for v in self.debounce[name]):
                self.last_stable[name] = self.debounce[name][0]
        yield name, self.last_stable[name]


    def write(self, name, value):
        if name not in self.outputs:
            raise ValueError(f"Output pin '{name}' not configured")
        GPIO.output(self.outputs[name], GPIO.HIGH if value else GPIO.LOW)

    def configure(self):
        #
        pass


class MCP23017:
    MAX_INPUTS = 8
    MAX_OUTPUTS = 8

    def __init__(self, addr, name=None):
        self.addr = addr
        self.name = name or f"MCP23017@{hex(addr)}"
        self.inputs = {}    # logical_name: (pin_number, pullup)
        self.outputs = {}   # logical_name: (pin_number, initial_value)
        self.debounce = {}  # pin_number: deque
        self.last_stable = {}  # pin_number: bool
        self.dev = None

    def input(self, pin, logical_name, *, pullup=False):
        if logical_name in self.inputs or logical_name in self.outputs:
            raise ValueError(f"Duplicate logical name '{logical_name}' on {self.name}")
        if len(self.inputs) >= MCP23017.MAX_INPUTS:
            raise ValueError(f"Too many inputs on {self.name} (max {MCP23017.MAX_INPUTS})")
        self.inputs[logical_name] = (pin, pullup)
        self.debounce[pin] = deque(maxlen=3)
        self.last_stable[pin] = False

    def output(self, pin, logical_name, *, initial=False):
        if logical_name in self.outputs or logical_name in self.inputs:
            raise ValueError(f"Duplicate logical name '{logical_name}' on {self.name}")
        if len(self.outputs) >= MCP23017.MAX_OUTPUTS:
            raise ValueError(f"Too many outputs on {self.name} (max {MCP23017.MAX_OUTPUTS})")
        self.outputs[logical_name] = (pin, initial)

    def configure(self):
        if not self.dev:
            try:
                i2c.writeto(self.addr, b"")  # probe
                self.dev = AdafruitMCP23017(i2c, address=self.addr)

                for name, (pin, pullup) in self.inputs.items():
                    p = self.dev.get_pin(pin)
                    p.direction = Direction.INPUT
                    p.pullup = pullup

                for name, (pin, initial) in self.outputs.items():
                    p = self.dev.get_pin(pin)
                    p.direction = Direction.OUTPUT
                    p.value = bool(initial)
            except OSError:
                self.dev = None

    def read(self):

        print(self.dev)
        print(self.inputs)
        print(self.outputs)
        print(self.debounce)
        print(self.last_stable)
        if not self.dev:
            for name in self.inputs:
                yield None, None
        else:
            for name, (pin, _) in self.inputs.items():
                self.debounce[pin].append(self.dev.get_pin(pin).value)

                if len(self.debounce[pin]) == 3 and all(v == self.debounce[pin][0] for v in self.debounce[pin]):
                    self.last_stable[pin] = self.debounce[pin][0]
                yield name, self.last_stable[name]

    def write(self, logical_name, value):
        if self.dev and logical_name in self.outputs:
            pin, _ = self.outputs[logical_name]
            self.dev.get_pin(pin).value = bool(value)

class ADS1115:
    def __init__(self, addr=0x48, name=None):
        self.addr = addr
        self.name = name or f"ADS@{hex(addr)}"
        self.inputs = {}  # logical_name -> channel
        self.filters = {}
        self.dev = None

    def input(self, channel, logical_name):
        self.inputs[logical_name] = channel
        self.filters[logical_name] = EMAFilter(0.1)


    def read(self):
        if not self.dev:
            for name in self.inputs:
                yield name, None
        else:
            for name, channel in self.inputs.items():
                yield name, self.filters[name].add(self.dev.read_voltage(channel))

    def configure(self):
        if not self.dev:
            try:
                i2c.writeto(self.addr, b"")  # Dummy write to probe
                self.dev = adafruit_ads1x15.ADS1115(i2c, address=addr)
            except OSError:
                self.dev = None

class BME280:
    def __init__(self, addr=0x76, name=None):
        self.addr = addr
        self.name = name or f"BME280@{hex(addr)}"
        self.dev = None
        self.inputs = {}  # logical_name -> measurement_key ('temperature', 'humidity', 'pressure')

    def configure(self):
        if not self.dev:
            try:
                i2c.writeto(self.addr, b"")  # Dummy write to probe
                self.dev = Adafruit_BME280.BME280(i2c, address=addr)
            except OSError:
                self.dev = None

    def input(self, channel, logical_name):
        # channel is one of 'temperature', 'humidity', 'pressure'
        self.inputs[logical_name] = channel

    def read(self):
        if not self.dev:
            for name in self.inputs:
                yield name, None

        for name, channel in self.inputs.items():
            yield name, getattr(self.dev, channel)

class QTEncoder:
    def __init__(self, addr=0x36, name=None):
        self.addr = addr
        self.name = name or f"SeeSaw@{hex(addr)}"
        self.dev = None
        self.last_position = 0
        self.delta = 0

    def configure(self):
        if not self.dev:
            try:
                i2c.writeto(self.addr, b"")  # Dummy write to probe
                self.dev = seesaw.Seesaw(i2c, address=addr)
                self.last_position = self.dev.encoder_position
            except OSError:
                self.dev = None

    def read_delta(self):
        if not self.dev:
            yield 0
        current = self.dev.encoder_position
        delta = current - self.last_position
        self.last_position = current
        yield delta
