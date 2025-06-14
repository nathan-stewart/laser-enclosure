#!/usr/bin/env python3
import zmq
import time
import json
import sys
import os
import threading
from RPi import GPIO
import busio
import board
import Adafruit_DHT
from adafruit_circuitpython_ads1x15 import ADS1115, AnalogIn
from hal_mcp23017 import MCP23017Handler
from pinmap import RPi_INPUT_PINS, RPi_OUTPUT_PINS, MCP23017_PINS
import pinmap

# Constants
INPUT_TIMEOUT = 0.5  # seconds
ENV_TIMEOUT = 60.0  # seconds
heartbeat_lock = threading.Lock()

# Identify unique MCP23017 and ADS1115 addresses
MCP23017_ADDRESSES = list(set([addr for addr, pin in MCP23017_PINS.values()]))
ADS1115_ADDRESSES = list(set([addr for addr, pin in pinmap.ADS1115_PINS.values()]))

# Shared heartbeat timestamps
last_input_poll = time.time()
last_sensor_poll = time.time()

# ZMQ setup
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:5556")
print("ZeroMQ publisher bound to tcp://*:5556")

rep = ctx.socket(zmq.REP)
rep.bind("tcp://*:5557")
print("ZeroMQ replier bound to tcp://*:5557")

# State tracking
rpi_gpio_states = {}
mcp23017_states = {name: 0 for name in MCP23017_PINS.keys()}
dht11_state = {'temperature': None, 'humidity': None}

def configure_gpio():
    """Configure Raspberry Pi GPIO pins."""
    GPIO.setmode(GPIO.BCM)
    try:
        for pin in RPi_INPUT_PINS:
            GPIO.setup(RPi_INPUT_PINS[pin], GPIO.IN, pull_up_down=GPIO.PUD_UP)
        for pin in RPi_OUTPUT_PINS:
            GPIO.setup(RPi_OUTPUT_PINS[pin], GPIO.OUT, initial=GPIO.LOW)
        print("RPi GPIOs configured.")
    except KeyError as e:
        print(f"Configuration error: RPi_GPIO_PINS dictionary is missing key {e}.", file=sys.stderr)
configure_gpio()

def configure_adc():
    """Detect and configure ADS1115 devices."""
    ads_devices = {}
    try:
        for name, (address, channel) in pinmap.ADS1115_PINS.items():
            if address not in ads_devices:
                try:
                    # Initialize the ADS1115 device if not already done
                    ads = ADS1115(busio.I2C(board.SCL, board.SDA), address=address)

                    # Test device presence by reading a known channel
                    test_channel = AnalogIn(ads, ADS1115.P0)
                    _ = test_channel.value  # Ensure the device is functional

                    # Add the device to the list of active devices
                    ads_devices[address] = ads
                    print(f"ADS1115 configured at address 0x{address:02X}.")
                except Exception as e:
                    print(f"ADS1115 at address 0x{address:02X} not present or failed to initialize: {e}", file=sys.stderr)
    except Exception as e:
        print(f"Error during ADS1115 configuration: {e}", file=sys.stderr)
    return ads_devices

# Call the function during initialization
ads_devices = configure_adc()

def configure_mcp_devices():
    """Detect and configure MCP23017 devices."""
    active_devices = []
    try:
        for address in MCP23017_ADDRESSES:
            try:
                mcp = MCP23017Handler([address])  # Initialize handler for this address
                active_devices.append((address, mcp))
                print(f"MCP23017 configured at address 0x{address:02X}.")
            except Exception as e:
                print(f"Error configuring MCP23017 at address 0x{address:02X}: {e}", file=sys.stderr)
    except Exception as e:
        print(f"Error during MCP23017 configuration: {e}", file=sys.stderr)
    return active_devices

# Call the function during initialization
mcp_devices = configure_mcp_devices()

def monitor_inputs():
    """Monitor input pins (RPi GPIO, MCP23017) and publish state changes."""
    global last_input_poll

    while True:
        # Monitor RPi GPIO pins
        for name, pin in RPi_INPUT_PINS.items():
            try:
                current_state = GPIO.input(pin)
                if current_state != rpi_gpio_states.get(name):
                    rpi_gpio_states[name] = current_state
                    msg = {
                        "topic": f"input/{name}",
                        "state": "active" if current_state == 0 else "inactive",
                    }
                    pub.send_json(msg)
            except Exception as e:
                if rpi_gpio_states.get(name) is not None:
                    print(f"Error reading RPi GPIO {name} (pin {pin}): {e}", file=sys.stderr)
                    rpi_gpio_states[name] = None
                    msg = {
                        "topic": f"input/{name}",
                        "state": "error",
                        "error": str(e),
                    }
                    pub.send_json(msg)

        # Monitor MCP23017 pins for active devices
        for address, mcp in mcp_devices:
            for name, (device_address, pin) in MCP23017_PINS.items():
                if device_address == address:
                    try:
                        current_state = mcp.read_pin(pin)
                        if current_state is not None and current_state != mcp23017_states.get(name):
                            mcp23017_states[name] = current_state
                            msg = {
                                "topic": f"input/{name}",
                                "state": "active" if current_state == 0 else "inactive",
                            }
                            pub.send_json(msg)
                    except Exception as e:
                        if mcp23017_states.get(name) is not None:
                            print(f"Error reading MCP23017 {name} (address 0x{device_address:02X}, pin {pin}): {e}", file=sys.stderr)
                            mcp23017_states[name] = None
                            msg = {
                                "topic": f"input/{name}",
                                "state": "error",
                                "error": str(e),
                            }
                            pub.send_json(msg)
        # Scan ADS1115 devices for analog input changes
        for address, ads in ads_devices.items():
            try:
                # Example: Read from channel P0
                channel = AnalogIn(ads, ADS1115.P0)
                value = channel.value
                msg = {
                    "topic": f"adc/{address:02X}/P0",
                    "value": value,
                }
                pub.send_json(msg)
            except Exception as e:
                print(f"Error reading ADS1115 at address 0x{address:02X}: {e}", file=sys.stderr)

        time.sleep(0.025)  # 40Hz polling interval

def monitor_1Hz():
    """Monitor sensors and publish state changes."""
    global last_sensor_poll, dht11_state

    while True:
        try:
            humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT11, RPi_INPUT_PINS['i_dht11'])
            temp_changed = temperature is not None and temperature != dht11_state['temperature']
            hum_changed = humidity is not None and humidity != dht11_state['humidity']

            if temp_changed or hum_changed:
                dht11_state['temperature'] = temperature
                dht11_state['humidity'] = humidity
                msg = {
                    "topic": "sensor/dht11",
                    "temperature": temperature,
                    "humidity": humidity,
                    "timestamp": time.time()
                }
                pub.send_json(msg)
        except Exception as e:
            print(f"Error reading DHT11: {e}", file=sys.stderr)

        time.sleep(1.0)  # 1Hz polling interval
