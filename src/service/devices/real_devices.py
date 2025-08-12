# i2c_devices.py
import sys
import RPi.GPIO as GPIO
import board
import busio
from adafruit_mcp230xx.mcp23017 import MCP23017 as AdafruitMCP23017
from adafruit_ads1x15.ads1115 import ADS1115 as AdafruitADS1115
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_bme280.basic import Adafruit_BME280_I2C
from adafruit_seesaw.seesaw import Seesaw
from adafruit_seesaw.rotaryio import IncrementalEncoder
from digitalio import Direction
from collections import deque
import threading

i2c = busio.I2C(board.SCL, board.SDA)
i2c_lock = threading.Lock()

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
        self.name = "RaspberryPi GPIO"
        self.inputs = {}
        self.outputs = {}
        self.debounce = {}
        self.last_stable = {}

    def __delete__(self):
        GPIO.cleanup()

    def input(self, bcm, name, pullup=True):
        GPIO.setup(bcm, GPIO.IN, pull_up_down=GPIO.PUD_UP if pullup else GPIO.PUD_DOWN)
        self.debounce[name] = deque(maxlen=3)
        self.inputs[name] = bcm
        self.last_stable[name] = None

    def output(self, bcm, name, initial=False):
        GPIO.setup(bcm, GPIO.OUT, initial=GPIO.HIGH if initial else GPIO.LOW)
        self.outputs[name] = bcm

    def read(self):
        result = {}
        for name in list(self.inputs.keys()):
            self.debounce[name].append(not GPIO.input(self.inputs[name])) # active low
            if all(v == self.debounce[name][0] for v in self.debounce[name]):
                self.last_stable[name] = self.debounce[name][0]
        yield name, self.last_stable[name]


    def write(self, name, value):
        if name not in self.outputs:
            return 
        GPIO.output(self.outputs[name], GPIO.LOW if value else GPIO.HIGH) # active low

class Expander:
    MAX_INPUTS = 8
    MAX_OUTPUTS = 8

    def __init__(self, addr, name=None):
        self.addr = addr
        self.name = name or f"MCP23017@{hex(addr)}"
        self.inputs = {}       # logical_name -> (pin, pullup)
        self.outputs = {}      # logical_name -> (pin, initial)
        self.debounce = {}     # pin -> deque([bool]*3)
        self.last_stable = {}  # pin -> bool
        self.dev = None
        self._in_pins = {}     # pin -> pin object (input)
        self._out_pins = {}    # pin -> pin object (output)

    def input(self, pin, logical_name, *, pullup=False):
        if self.dev:
            raise RuntimeError("f{self.name} Cannot add inputs after configure()")
        if logical_name in self.inputs or logical_name in self.outputs:
            raise ValueError(f"Duplicate logical name '{logical_name}' on {self.name}")
        if len(self.inputs) >= Expander.MAX_INPUTS:
            raise ValueError(f"Too many inputs on {self.name} (max {Expander.MAX_INPUTS})")
        self.inputs[logical_name] = (pin, pullup)
        self.debounce[pin] = deque(maxlen=3)
        for _ in range(3):
            self.debounce[pin].append(False)
        self.last_stable[pin] = False

    def output(self, pin, logical_name, *, initial=False):
        if self.dev:
            raise RuntimeError("Cannot add outputs after configure()")
        if logical_name in self.outputs or logical_name in self.inputs:
            raise ValueError(f"Duplicate logical name '{logical_name}' on {self.name}")
        if len(self.outputs) >= Expander.MAX_OUTPUTS:
            raise ValueError(f"Too many outputs on {self.name} (max {Expander.MAX_OUTPUTS})")
        self.outputs[logical_name] = (pin, initial)

    def configure(self):
        if self.dev:
            raise RuntimeError(f"{self.name} Already configured")
        
        try:
            with i2c_lock:
                i2c.writeto(self.addr, b"")  # probe
                self.dev = AdafruitMCP23017(i2c, address=self.addr)

                # Inputs
                for _, (pin, pullup) in self.inputs.items():
                    p = self.dev.get_pin(pin)
                    p.direction = Direction.INPUT
                    p.pullup = bool(pullup)
                    self._in_pins[pin] = p
                    # Seed debouncer from hardware
                    raw = bool(p.value)
                    self.debounce[pin] = deque([raw, raw, raw], maxlen=3)
                    self.last_stable[pin] = raw

                # Outputs
                for _, (pin, initial) in self.outputs.items():
                    p = self.dev.get_pin(pin)
                    p.direction = Direction.OUTPUT
                    p.value = bool(initial)
                    self._out_pins[pin] = p

        except OSError:
            self.dev = None

    def read(self):
        if not self.dev:
            for name in self.inputs:
                yield name, None
            return
        with i2c_lock:
            for name, (pin, _) in self.inputs.items():
                try:
                    v = bool(self._in_pins[pin].value)
                except OSError:
                    # keep previous stable value on error
                    yield name, self.last_stable[pin]
                    continue

                dq = self.debounce[pin]
                dq.append(v)
                if dq[0] == dq[1] == dq[2]:
                    self.last_stable[pin] = dq[2]
                yield name, self.last_stable[pin]

    def write(self, logical_name, value):
        if not self.dev:
            return 
        
        if logical_name not in self.outputs:
            raise KeyError(f"Unknown output '{logical_name}'")
        pin, _ = self.outputs[logical_name]
        with i2c_lock:
            try:
                self._out_pins[pin].value = bool(value)
            except OSError:
                pass # optionally log and continue

class AnalogRead:
    def __init__(self, addr=0x48, name=None):
        self.addr = addr
        self.name = name or f"ADS@{hex(addr)}"
        self.inputs = {}  # logical_name -> channel
        self.filters = {}
        self.channels = {} # hold the AnalogIn objects
        self.dev = None

    def input(self, channel, logical_name):
        if self.dev:
            raise RuntimeError(f"Cannot add inputs after {self.name} is configured")

        if len (self.inputs.keys()) >= 4:
            raise ValueError(f"Too many inputs on {self.name} (max 4)")
        self.inputs[logical_name] = channel
        self.filters[logical_name] = EMAFilter(0.1)

    def read(self):        
        if not self.dev:
            for name in self.inputs:
                yield name, None
        else:
            with i2c_lock:
                for name, ch in self.channels.items():
                    try:
                        v = self.channels[name].voltage
                    except OSError:
                        # transient bus error; attempt soft reset/reinit
                        self._recover_ads()
                        v = None
                    yield name, self.filters[name].add(v) if v is not None else None

    def configure(self, addr=0x48):
        if self.dev is not None:
            raise RuntimeError(f"{self.name} can only be configured once")

        if not self.dev:
            try:
                with i2c_lock:
                    i2c.writeto(self.addr, b"")  # Dummy write to probe
                    self.dev = AdafruitADS1115(i2c, address=addr)
                    self.dev.data_rate = 128
                    self.dev.gain = 1
                    for logical_name, channel in self.inputs.items():
                        if channel < 0 or channel > 3:
                            raise ValueError(f"Invalid channel {channel} for {self.name}")
                        if logical_name in self.channels:
                            raise ValueError(f"Duplicate logical name '{logical_name}' on {self.name}")
                        self.channels[logical_name] = AnalogIn(self.dev, channel)

            except OSError:
                self.dev = None


class Environment:
    VALID_KEYS = {"temperature", "humidity", "pressure"}
    def __init__(self, addr=0x76, name=None):
        self.addr = addr
        self.name = name or f"BME280@{hex(addr)}"
        self.dev = None
        self.inputs = {}  # logical_name -> measurement_key ('temperature', 'humidity', 'pressure')
        self.filters = {}

    def configure(self):
        if self.dev:
            raise RuntimeError(f"{self.name} can only be configured once")
        try:
            with i2c_lock:
                i2c.writeto(self.addr, b"")  # probe
                self.dev = Adafruit_BME280_I2C(i2c, address=self.addr)
                self.filters = {name: EMAFilter(0.1) for name in self.inputs}
        except OSError:
            self.dev = None

    def input(self, measurement_key, logical_name, alpha=0.2):
        if self.dev:
            raise RuntimeError("Cannot add inputs after device is configured")

        if measurement_key not in self.VALID_KEYS:
            raise ValueError(f"Invalid measurement '{measurement_key}'. "
                             f"Use one of {sorted(self.VALID_KEYS)}")
        with i2c_lock:
            self.inputs[logical_name] = measurement_key
            self.filters[logical_name] = EMAFilter(alpha)

    def read(self):
        # snapshot under lock so config/read can’t race
        with self._lock:
            dev = self.dev
            mapping = dict(self.inputs)
            filters = dict(self.filters)

        if not dev:
            for logical in mapping or self.inputs:
                yield logical, None
            return

        with i2c_lock:
            for logical, key in mapping.items():
                try:
                    v = getattr(dev, key)   # 'temperature' | 'humidity' | 'pressure'
                    yield logical, filters[logical].add(v)
                except OSError:
                    yield logical, None

    def read(self):
        if not self.dev:
            for name in self.inputs.keys():
                yield name, None
            return
        with i2c_lock:
            for name in self.inputs.keys():
                yield name, getattr(self.dev, name)

class RotaryEncoder:
    def __init__(self, addr=0x36, name=None):
        self.addr = addr
        self.name = name or f"SeeSaw@{hex(addr)}"
        self.dev = None
        self.last_position = 0
        self.delta = 0

    def configure(self):
        if self.dev:
            raise RuntimeError(f"{self.name} can only be configured once")
        else:
            try:
                with i2c_lock:
                    i2c.writeto(self.addr, b"")  # Dummy write to probe
                    self.dev = seesaw.Seesaw(i2c, address=addr)
                    self.last_position = self.dev.encoder_position
            except OSError:
                self.dev = None

    def read_delta(self):
        if not self.dev:
            return
        
        parameter = "encoder_delta"
        if not self.dev:
            yield parameter, 0
        else:
            with i2c_lock:
                current = self.dev.encoder_position
                delta = current - self.last_position
                self.last_position = current
                yield parameter, delta
