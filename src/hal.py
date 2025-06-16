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
import adafruit_dht
from adafruit_circuitpython_ads1x15 import ADS1115, AnalogIn
from hal_mcp23017 import MCP23017Handler
from pinmap import RPi_INPUT_PINS, RPi_OUTPUT_PINS, MCP23017_PINS
import pinmap

from sdnotify import SystemdNotifier
notifier = SystemdNotifier()

i2c_bus1 = busio.I2C(board.SCL, board.SDA)
# Constants
INPUT_TIMEOUT = 0.5  # seconds
ENV_TIMEOUT = 60.0  # seconds
heartbeat_lock = threading.Lock()

# Identify unique MCP23017 and ADS1115 addresses
MCP23017_ADDRESSES = list(set([addr for addr, pin in MCP23017_PINS.values()]))
ADS1115_ADDRESSES = list(set([addr for addr, pin in pinmap.ADS1115_PINS.values()]))

# Shared heartbeat timestamps
last_40hz_poll = time.time()
last_1hz_poll = time.time()

dht11 = None
ads1115 = None
mcp23017 = None


# ZMQ setup
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:5556")
print("ZeroMQ publisher bound to tcp://*:5556")

rep = ctx.socket(zmq.REP)
rep.bind("tcp://*:5557")
print("ZeroMQ replier bound to tcp://*:5557")

def handle_commands():
    while True:
        try:
            msg = rep.recv_json()
            # Example command structure: {"cmd": "set", "pin": "o_k1_laser", "state": True}
            cmd = msg.get("cmd")
            if cmd == "set":
                pin = msg["pin"]
                state = msg["state"]
                if pin in RPi_OUTPUT_PINS:
                    GPIO.output(RPi_OUTPUT_PINS[pin], GPIO.HIGH if state else GPIO.LOW)
                    rep.send_json({"status": "ok"})
                elif pin in MCP23017_PINS:
                    addr, mcp_pin = MCP23017_PINS[pin]
                    for _, handler in mcp_devices:
                        if handler.addresses[0] == addr:
                            success = handler.write_pin(addr, mcp_pin, state)
                            rep.send_json({"status": "ok" if success else "fail"})
                            break
                    else:
                        rep.send_json({"status": "unknown mcp"})
                else:
                    rep.send_json({"status": "unknown pin"})
            else:
                rep.send_json({"status": "unsupported command"})
        except Exception as e:
            try:
                rep.send_json({"status": "error", "error": str(e)})
            except:
                 print(f"Error handling ZMQ command and replying: {e}", file=sys.stderr)


# State tracking
rpi_gpio_states = {}
mcp23017_states = {name: 0 for name in MCP23017_PINS.keys()}


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

def configure_adc(i2c_bus):
    ads_by_address = {}
    ads_devices = {}
    for name, (address, channel) in pinmap.ADS1115_PINS.items():
        try:
            if address not in ads_by_address:
                ads = ADS1115(i2c_bus, address=address)
                ads_by_address[address] = ads
                print(f"ADS1115 configured at address 0x{address:02X}.")
            else:
                ads = ads_by_address[address]
            chan = getattr(ADS1115, f'P{channel}')
            analog_in = AnalogIn(ads, chan)
            ads_devices[name] = {
                'ads': ads,
                'analog_in': analog_in,
                'address': address,
                'channel': channel
            }
        except Exception as e:
            print(f"ADS1115 {name} at address 0x{address:02X} channel {channel} not present or failed to initialize: {e}", file=sys.stderr)
    return ads_devices


# Call the function during initialization
ads_devices = configure_adc(i2c_bus1)

def configure_mcp_devices(i2c_bus):
    """Detect and configure MCP23017 devices."""
    active_devices = []
    try:
        for address in MCP23017_ADDRESSES:
            try:
                mcp = MCP23017Handler(i2c_bus,[address])  # Initialize handler for this address
                active_devices.append((address, mcp))
                print(f"MCP23017 configured at address 0x{address:02X}.")
            except Exception as e:
                print(f"Error configuring MCP23017 at address 0x{address:02X}: {e}", file=sys.stderr)
    except Exception as e:
        print(f"Error during MCP23017 configuration: {e}", file=sys.stderr)
    return active_devices

# Call the function during initialization
mcp_devices = configure_mcp_devices(i2c_bus1)

def read_gpio():
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

def read_expanders():
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

def read_adc():
    for name, adc_info in ads_devices.items():
        try:
            value = adc_info['analog_in'].value
            msg = {
                "topic": f"adc/{name}",
                "value": value,
            }
            pub.send_json(msg)
        except Exception as e:
            print(f"Error reading ADS1115 {name} at 0x{adc_info['address']:02X}: {e}", file=sys.stderr)

def monitor_40Hz():
    """Monitor input pins (RPi GPIO, MCP23017) and publish state changes."""
    global last_40hz_poll

    while True:
        read_gpio()
        read_expanders()
        read_adc()
        with heartbeat_lock:
            last_40hz_poll = time.time()
        time.sleep(0.025)  # 40Hz polling interval

def configure_dht11():
    """Configure DHT11 sensor."""
    try:
        # Initialize the DHT11 sensor
        return adafruit_dht.DHT11(RPi_INPUT_PINS['i_dht11'])
    except Exception as e:
        return None
    
dht11_state = {'temperature': None, 'humidity': None}
dht11 = configure_dht11()
if dht11 is None:
    print("DHT11 sensor not configured. Check wiring and pin assignment.", file=sys.stderr)
else:
    print("DHT11 sensor found.")


def monitor_1Hz():
    """Monitor sensors and publish state changes."""
    global last_1hz_poll, dht11_state

    while True:
        try:
            if dht11:
                temperature = dht11.temperature
                humidity = dht11.humidity
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
        with heartbeat_lock:
            last_1hz_poll = time.time()
        time.sleep(1.0)  # 1Hz polling interval

if __name__ == "__main__":
    threading.Thread(target=handle_commands, daemon=True).start()
    threading.Thread(target=monitor_40Hz, daemon=True).start()
    threading.Thread(target=monitor_1Hz, daemon=True).start()
    try:
        while True:
            with heartbeat_lock:
                elapsed = time.time() - last_40hz_poll
            if elapsed < INPUT_TIMEOUT:
                # Placeholder for petting actual WDT
                print("Watchdog pet.")
                notifier.notify("WATCHDOG=1")
            else:
                print(f"Watchdog skipped! Last update {elapsed:.2f}s ago.")
            time.sleep(1.0)

    except KeyboardInterrupt:
        GPIO.cleanup()
        print("HAL shutdown gracefully.")
