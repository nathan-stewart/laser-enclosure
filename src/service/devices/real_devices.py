# i2c_devices.py
import sys
import board
import busio
i2c_bus1 = busio.I2C(board.SCL, board.SDA)

from adafruit_mcp230xx.mcp23017 import MCP23017
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_bme280 import Adafruit_BME280_I2C
from adafruit_seesaw import seesaw

class RpiGpio:
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        self._inputs = {}
        self._outputs = {}
        self._state = {}

    def __delete__(self):
        GPIO.cleanup()

    def input(self, name, bcm, pullup=True):
        GPIO.setup(bcm, GPIO.IN, pull_up_down=GPIO.PUD_UP if pullup else GPIO.PUD_DOWN)
        self._inputs[name] = bcm
        self._state[name] = None

    def output(self, name, bcm, initial=False):
        GPIO.setup(bcm, GPIO.OUT, initial=GPIO.HIGH if initial else GPIO.LOW)
        self._outputs[name] = bcm
        self._state[name] = initial

    def read(self, name):
        if name not in self._inputs:
            raise ValueError(f"Input pin '{name}' not configured")
        return GPIO.input(self._inputs[name])

    def write(self, name, value):
        if name not in self._outputs:
            raise ValueError(f"Output pin '{name}' not configured")
        GPIO.output(self._outputs[name], GPIO.HIGH if value else GPIO.LOW)
        self._state[name] = bool(value)


class ADS1115:
    def __init__(self, i2c, addr=0x48, name=None):
        self.i2c = i2c
        self.addr = addr
        self.name = name or f"ADS@{hex(addr)}"
        self.inputs = {}  # logical_name -> channel
        self.dev = None

    def configure(self):
        if self.dev:
            return
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

class MCP23017:
    MAX_INPUTS = 8
    MAX_OUTPUTS = 8

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
        if len(self.inputs) >= MCP23017.MAX_INPUTS:
            raise ValueError(f"Too many inputs on {self.name} (max {MCP23017.MAX_INPUTS})")
        self.inputs[logical_name] = (pin, pullup)

    def output(self, pin, logical_name, *, initial=False):
        if logical_name in self.outputs or logical_name in self.inputs:
            raise ValueError(f"Duplicate logical name '{logical_name}' on {self.name}")
        if len(self.outputs) >= MCP23017.MAX_OUTPUTS:
            raise ValueError(f"Too many outputs on {self.name} (max {MCP23017.MAX_OUTPUTS})")
        self.outputs[logical_name] = (pin, initial)

    def configure(self):
        if self.dev:
            return

        try:
            i2c.writeto(self.addr, b"")  # Dummy write to probe
            self.dev = MCP23017(self.i2c, address=self.addr)
        except OSError:
            pass
        self.dev = None

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


def configure_ads1115(i2c, addr):
    try:
        i2c.writeto(addr, b"")  # Dummy write to probe
        return ADS1115(i2c, address=addr)
    except OSError:
        pass
    return None

def configure_bme280(i2c, addr):
    try:
        i2c.writeto(addr, b"")  # Dummy write to probe
        return Adafruit_BME280_I2C(i2c, address=addr)
    except OSError:
        pass
    return None

def configure_seesaw(i2c, addr):
    try:
        i2c.writeto(addr, b"")  # Dummy write to probe
        return seesaw.Seesaw(i2c, address=addr)
    except OSError:
        pass
    return None

def read_ads1115(dev, channel=0): return dev.read_voltage(channel)

class BME280:
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

class QTEncoder:
    def __init__(self, i2c, addr=0x36, name=None):
        self.i2c = i2c
        self.addr = addr
        self.name = name or f"SeeSaw@{hex(addr)}"
        self.dev = None
        self.last_position = 0
        self.delta = 0

    def configure(self):
        if self.dev:
            return
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
