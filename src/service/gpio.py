#gpio.py
class GpioController:
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
