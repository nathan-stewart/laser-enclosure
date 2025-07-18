# i2c_devices.py
import sys
USE_MOCK = False

if not USE_MOCK:
    from mock_devices import (
        MockMCP23017 as MCP23017,
        MockADS1115 as ADS1115,
        MockBME280 as BME280,
        MockSeeSaw as SeeSaw,
    )
else:
    import board
    import busio
    i2c_bus1 = busio.I2C(board.SCL, board.SDA)

    from adafruit_mcp230xx.mcp23017 import MCP23017
    from adafruit_ads1x15.analog_in import AnalogIn
    from adafruit_ads1x15.ads1115 import ADS1115
    from adafruit_bme280 import Adafruit_BME280_I2C
    from adafruit_seesaw import seesaw

# If i2c not available or no device found it throws an OSError and returns None
# if USE_MOCK is true, the device returned is safe for no i2c available

def configure_mcp23017(i2c, addr): 
    try:
        if not USE_MOCK:
            i2c.writeto(addr, b"")  # Dummy write to probe
        return  MCP23017(i2c, address=addr)
    except OSError:
        pass
    return None
    
def configure_ads1115(i2c, addr): 
    try:
        if not USE_MOCK:
            i2c.writeto(addr, b"")  # Dummy write to probe
        return ADS1115(i2c, address=addr)
    except OSError:
        pass
    return None
    
def configure_bme280(i2c, addr): 
    try:
        if not USE_MOCK:
            i2c.writeto(addr, b"")  # Dummy write to probe
        return Adafruit_BME280_I2C(i2c, address=addr)
    except OSError:
        pass
    return None

def configure_seesaw(i2c, addr):
    try:
        if not USE_MOCK:
            i2c.writeto(addr, b"")  # Dummy write to probe
        return seesaw.Seesaw(i2c, address=addr)
    except OSError:
        pass
    return None
                
def read_ads1115(dev, channel=0): return dev.read_voltage(channel)
