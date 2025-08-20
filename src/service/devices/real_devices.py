# real_devices.py
import sys
import RPi.GPIO as GPIO
import board
import busio
from adafruit_mcp230xx.mcp23017 import MCP23017 as AdafruitMCP23017
from adafruit_ads1x15.ads1115 import ADS1115 as AdafruitADS1115
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_bme280.basic import Adafruit_BME280_I2C
from digitalio import Direction, Pull
from adafruit_seesaw.rotaryio import IncrementalEncoder
from adafruit_seesaw import digitalio, neopixel, rotaryio, seesaw
import threading
from collections import deque

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

    def __del__(self):
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
        pins = list(self.inputs.keys())
        for name in pins:
            self.debounce[name].append(not GPIO.input(self.inputs[name]))  # active low
            if len(self.debounce[name]) == 3 and all(v == self.debounce[name][0] for v in self.debounce[name]):
                self.last_stable[name] = self.debounce[name][0]
            yield name, self.last_stable[name]

    def dump_output(self, name):
        return GPIO.input(self.outputs[name])

    def write(self, name, value):
        if name not in self.outputs:
            return
        GPIO.output(self.outputs[name], GPIO.HIGH if value else GPIO.LOW) # active low

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
            raise RuntimeError(f"{self.name} Cannot add inputs after configure()")

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
            raise RuntimeError(f"{self.name} Cannot add outputs after configure()")

        if logical_name in self.outputs or logical_name in self.inputs:
            raise ValueError(f"Duplicate logical name '{logical_name}' on {self.name}")

        if len(self.outputs) >= Expander.MAX_OUTPUTS:
            raise ValueError(f"Too many outputs on {self.name} (max {Expander.MAX_OUTPUTS})")

        self.outputs[logical_name] = (pin, initial)


    def configure(self):
        if self.dev:
            return None
        try:
            with i2c_lock:
                i2c.writeto(self.addr, b"")  # probe
                self.dev = AdafruitMCP23017(i2c, address=self.addr)
                # Inputs
                for _, (pin, pullup) in self.inputs.items():
                    p = self.dev.get_pin(pin)
                    p.direction = Direction.INPUT
                    p.pull = Pull.UP if pullup else None
                    self._in_pins[pin] = p
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
            self.dev = None  # remains disconnected
        return self.dev

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
                    # Optional: trigger reconnect
                    # self.dev = None
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
                    # Optional: trigger reconnect
                    # self.dev = None
                    pass

class AnalogRead:
    def __init__(self, addr=0x48, name=None):
        self.addr = addr
        self.name = name or f"ADS@{hex(addr)}"
        self.inputs = {}  # logical_name -> channel
        self.filters = {}
        self.channels = {} # hold the AnalogIn objects
        self.dev = None
        self.error_count = 0
        self.max_errors = 10

    def input(self, channel, logical_name):
        if self.dev:
            raise RuntimeError(f"Cannot add inputs after {self.name} is configured")
        if len(self.inputs) >= 4:
            raise ValueError(f"Too many inputs on {self.name} (max 4)")
        self.inputs[logical_name] = channel
        self.filters[logical_name] = EMAFilter(0.1)

    def configure(self):
        if self.dev:
            return None
        try:
            with i2c_lock:
                i2c.writeto(self.addr, b"")
                self.dev = AdafruitADS1115(i2c, address=self.addr)
                self.dev.data_rate = 128
                self.dev.gain = 1
                self.channels = {}
                for logical_name, channel in self.inputs.items():
                    if not (0 <= channel <= 3):
                        raise ValueError(f"Invalid channel {channel} for {self.name}")
                    self.channels[logical_name] = AnalogIn(self.dev, channel)
            self._errors = 0
        except OSError:
            self.dev = None
        return self.dev

    def recover_ads(self):
        # Mark disconnected; configure_thread will rebuild
        self.dev = None
        self.channels.clear()
        self.errors = 0

    def read(self):
        if not self.dev:
            for name in self.inputs:
                yield name, None
            return
        with i2c_lock:
            for name, ain in self.channels.items():
                try:
                    v = ain.voltage
                    self.errors = 0
                except OSError:
                    self.errors += 1
                    if self.errors >= 3:
                        self.recover_ads()
                    v = None
                yield name, (self.filters[name].add(v) if v is not None else None)


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
            return None
        try:
            with i2c_lock:
                i2c.writeto(self.addr, b"")
                self.dev = Adafruit_BME280_I2C(i2c, address=self.addr)
            # build filters for current mapping
            self.filters = {name: self.filters.get(name, EMAFilter(0.1)) for name in self.inputs}
        except OSError:
            self.dev = None
        return self.dev

    def input(self, measurement_key, logical_name, alpha=0.2):
        if self.dev:
            raise RuntimeError("Cannot add inputs after device is configured")
        if measurement_key not in self.VALID_KEYS:
            raise ValueError(f"Invalid measurement '{measurement_key}'. Use one of {sorted(self.VALID_KEYS)}")
        self.inputs[logical_name] = measurement_key
        self.filters.setdefault(logical_name, EMAFilter(alpha))

    def read(self):
        if not self.dev:
            for logical in self.inputs:
                yield logical, None
            return
        with i2c_lock:
            for logical, key in self.inputs.items():
                try:
                    v = getattr(self.dev, key)  # °C, %RH, hPa
                    yield logical, self.filters[logical].add(v)
                except OSError:
                    # Optional: self.dev=None to trigger reconnect
                    yield logical, None

class RotaryEncoder:
    def __init__(self, addr=0x36, name=None):
        self.addr = addr
        self.name = name or f"SeeSaw@{hex(addr)}"
        self.dev = None
        self.enc = None
        self.last_position = 0
        self.delta = 0

    def configure(self):
        if self.dev:
            return None
        try:
            with i2c_lock:
                i2c.writeto(self.addr, b"")
                self.dev = seesaw.Seesaw(i2c, addr=self.addr)
                self.enc = IncrementalEncoder(self.dev)
                self.last_position = int(self.enc.position)
                # turn off the LED
                self.pixel = neopixel.NeoPixel(self.dev, 6, 1)
                self.pixel.brightness = 0.0

        except OSError:
            self.dev = None
            self.enc = None
            self.last_position = 0
        return self.dev

    def read_delta(self):
        param = "encoder_delta"
        if not self.enc:
            yield param, 0
            return
        with i2c_lock:
            try:
                current = int(self.enc.position)
            except OSError:
                yield param, 0
                return
        delta = current - self.last_position
        self.last_position = current
        yield param, delta
