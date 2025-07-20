# devices/__init__.py

import argparse

USE_MOCK = False  # default, override this in your app setup

def configure_mock(use_mock: bool):
    global USE_MOCK
    USE_MOCK = use_mock

if USE_MOCK:
    from .mock_devices import (
        MockGpioController as Gpio,
        MockMCP23017 as MCP23017,
        MockADS1115 as ADS1115,
        MockBME280 as BME280,
        MockSeeSaw as SeeSaw,
    )
else:
    from .real_devices import (
        RpiGpio as Gpio,
        MCP23017,
        ADS1115,
        BME280,
        SeeSaw,
    )
