# devices/__init__.py

import argparse

USE_MOCK = False  # default, override this in your app setup

def configure_mock(use_mock: bool):
    global USE_MOCK
    USE_MOCK = use_mock

if USE_MOCK:
    from .mock_devices import (
        MockGpioController as RpiGpio,
        MockExpander as Expander,
        MockAnalogRead as AnalogRead,
        MockEnvironment as Environment,
        MockRotaryEncoder as RotaryEncoder,
    )
else:
    from .real_devices import (
        RpiGpio as Gpio,
        Expander,
        AnalogRead,
        Environment,
        RotaryEncoder,
    )
