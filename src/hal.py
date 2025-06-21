#!/usr/bin/env python3
import sys
import logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s [%(levelname)s] %(message)s",
    handlers=[
        logging.StreamHandler(sys.stdout)
    ]
)
log = logging.getLogger("hal")

import zmq
import time
import json
import os
import threading
from RPi import GPIO
import busio
import board
import adafruit_dht
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_mcp230xx.mcp23017 import MCP23017
import pinmap
from pinmap import RPi_INPUT_PINS, RPi_OUTPUT_PINS, MCP23017_PINS, DHT11_PINS
from sdnotify import SystemdNotifier

notifier = SystemdNotifier()
i2c_bus1 = busio.I2C(board.SCL, board.SDA)

wait_40Hz = 1.0 / 40.0
wait_5s = 5.0
TIMEOUT_40Hz = 0.5  # seconds
TIMEOUT_5s = 60.0  # seconds
heartbeat_lock = threading.Lock()
stop_event = threading.Event()
last_40Hz_poll = time.time()
last_5s_poll = time.time()
error_threshold = 5

MCP23017_ADDRESSES = list(set([addr for addr, pin in MCP23017_PINS.values()]))
ADS1115_ADDRESSES = list(set([addr for addr, pin in pinmap.ADS1115_PINS.values()]))

mcp_devices = []
ads_devices = {}
dht11_dev = None

# State tracking
current_state = {}
for p in RPi_INPUT_PINS.keys():       current_state[p] = None
for p in RPi_OUTPUT_PINS.keys():      current_state[p] = None
for p in pinmap.MCP23017_PINS.keys(): current_state[p] = None
for p in pinmap.ADS1115_PINS.keys():  current_state[p] = None
current_state['temperature'] = None
current_state['humidity']    = None
current_state['error_count'] = {
    "adc": 0,
    "mcp": 0,
    "dht11": 0
}
previous_state = current_state.copy()

# ZMQ setup
ctx = zmq.Context()
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:5556")
log.info("ZeroMQ publisher bound to tcp://*:5556")

rep = ctx.socket(zmq.REP)
rep.bind("tcp://*:5557")
log.info("ZeroMQ replier bound to tcp://*:5557")

def configure_gpio():
    """Configure Raspberry Pi GPIO pins."""
    GPIO.setmode(GPIO.BCM)
    try:
        for name in RPi_INPUT_PINS:
            GPIO.setup(RPi_INPUT_PINS[name], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            log.debug(f"RPi GPIO input pin {name} configured on BCM pin {RPi_INPUT_PINS[name]}.")
        for name in RPi_OUTPUT_PINS:
            GPIO.setup(RPi_OUTPUT_PINS[name], GPIO.OUT, initial=GPIO.LOW)
        log.info("RPi GPIOs configured.")
    except KeyError as e:
        log.error(f"Configuration error: RPi_GPIO_PINS dictionary is missing key {e}.")

def configure_adc(i2c_bus):
    ads_by_address = {}
    ads_devices = {}
    for name, (address, channel) in pinmap.ADS1115_PINS.items():
        try:
            if address not in ads_by_address:
                ads = ADS1115(i2c_bus, address=address)
                ads_by_address[address] = ads
                log.info(f"ADS1115 configured at address 0x{address:02X}.")
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
            log.error(f"ADS1115 {name} at address 0x{address:02X} channel {channel} not present or failed to initialize: {e}")
    return ads_devices

def configure_mcp_devices(i2c_bus):
    """Detect and configure MCP23017 devices."""
    active_devices = []
    try:
        for address in MCP23017_ADDRESSES:
            try:
                mcp = MCP23017(i2c_bus, address=address)
                # Set all pins as inputs with pull-ups enabled
                for pin in range(16):
                    mcp.setup(pin, mcp.IN)
                    mcp.pullups |= (1 << pin)
                active_devices.append((address, mcp))
                log.info(f"MCP23017 configured at address 0x{address:02X}.")
            except Exception as e:
                log.error(f"Error configuring MCP23017 at address 0x{address:02X}: {e}")
    except Exception as e:
        log.error(f"Error during MCP23017 configuration: {e}")
    return active_devices


def configure_dht11():
    """Configure DHT11 sensor."""
    try:
        # Initialize the DHT11 sensor
        dht11 = adafruit_dht.DHT11(DHT11_PINS['i_dht11']) 
        
        log.info("DHT11 sensor found.")
        return dht11
    except Exception as e:
        log.error(f"DHT11 sensor not configured {e}.")
        return None


def read_gpio():
    for name, pin in RPi_INPUT_PINS.items():
        try:
            current_state[name] = GPIO.input(pin)
        except Exception as e:
            raise RuntimeError(f"GPIO read error on pin {name}: {e}")

def read_expanders():
    # Monitor MCP23017 pins for active devices
    for address, mcp in mcp_devices:
        for name, (device_address, pin) in MCP23017_PINS.items():
            if device_address == address:
                try:
                    current_state[name] = mcp.input(pin)
                    if current_state['error_count']['mcp'] > 0:
                        log.info(f"MCP23017 read error recovery at address 0x{address:02X}")
                        current_state['error_count']['mcp'] = 0
                except Exception as e:
                    current_state['error_count']['mcp'] += 1
                    if current_state['error_count']['mcp'] == error_threshold:
                        log.error(f"MCP23017 read error at address 0x{address:02X}: {e}")

def read_adc():
    for name, adc_info in ads_devices.items():
        try:
            current_state[name] = adc_info['analog_in'].value
            if current_state['error_count']['adc'] > 0:
                log.info(f"ADC read error recover at address 0x{adc_info['address']:02X}")
                current_state['error_count']['adc'] = 0
        except Exception as e:
            current_state['error_count']['adc'] += 1
            if current_state['error_count']['adc'] == error_threshold:
                log.error(f"ADC read error at address 0x{adc_info['address']:02X}: {e}")

def read_environment():
    global dht11_dev, current_state, last_5s_poll
    if dht11_dev:
        try:
            current_state['temperature'] = dht11_dev.temperature
            current_state['humidity'] = dht11_dev.humidity
            last_5s_poll = time.time()
            if current_state['error_count']['dht11'] > 0:
                log.info("Recovered from DHT11 error.")
                current_state['error_count']['dht11'] = 0
        except RuntimeError as e:
            current_state['error_count']['dht11'] += 1
            if current_state['error_count']['dht11'] == error_threshold:
                log.warning(f"DHT11 read error: {e}")

def monitor_inputs():
    """Monitor input pins (RPi GPIO, MCP23017) and publish state changes."""
    global current_state, last_40Hz_poll
    while not stop_event.is_set():
        read_gpio()
        read_expanders()
        read_adc()
        with heartbeat_lock:
            last_40Hz_poll = time.time()
            if time.time() - last_40Hz_poll > TIMEOUT_40Hz:
                raise RuntimeError("Input read timeout")
        publish_deltas()
        stop_event.wait(wait_40Hz) # 40 Hz polling interval

def monitor_env():
    """Monitor environment and publish state changes."""
    global last_5s_poll, current_state
    while not stop_event.is_set():
        read_environment()
        with heartbeat_lock:
            last_5s_poll = time.time()
            if time.time() - last_5s_poll > TIMEOUT_40Hz:
                raise RuntimeError("Input read timeout")
        publish_deltas()
        stop_event.wait(wait_5s)  # 5 second polling interval

def build_state_response():
    return {
        "input": {k: current_state[k] for k in RPi_INPUT_PINS},
        "output": {k: current_state[k] for k in RPi_OUTPUT_PINS},
        "adc": {k: current_state[k] for k in pinmap.ADS1115_PINS},
        "sensor": {
            "temperature": current_state["temperature"],
            "humidity": current_state["humidity"]
        },
        "expander": {k: current_state[k] for k in MCP23017_PINS},
        "error_count": current_state["error_count"]
    }

def publish_deltas():
    global current_state, previous_state
    deltas = {k: v for k, v in current_state.items() if previous_state.get(k) != v}
    if deltas:
        msg = {
            "topic": "state/update",
            "deltas": deltas,
            "timestamp": time.time()
        }
        pub.send_json(msg)
        log.debug(f"Published state update: {deltas}")
    previous_state.update(current_state)

def set_output(pin, value):
    """Set the state of an output pin."""
    global current_state
    try:
        if pin in RPi_OUTPUT_PINS:
            GPIO.output(RPi_OUTPUT_PINS[pin], GPIO.HIGH if value else GPIO.LOW)
            current_state[pin] = GPIO.input(RPi_OUTPUT_PINS[pin])
            if current_state[pin] != (1 if value else 0):
                raise RuntimeError(f"Failed to set RPi output pin {pin} to {value}")
        elif pin in MCP23017_PINS:
            addr, mcp_pin = MCP23017_PINS[pin]
            for device_addr, mcp in mcp_devices:
                if device_addr == addr:
                    try:
                        mcp.setup(mcp_pin, mcp.OUT)
                        mcp.output(mcp_pin, 1 if value else 0)
                        current_state[pin] = 1 if value else 0
                    except Exception as e:
                        raise RuntimeError(f"Error setting MCP23017 pin {pin}: {e}")
                    break
        else:
            raise ValueError(f"Unknown pin: {pin}")

        msg = {
            "status": "ok",
            "pin": pin,
            "value": value
        }
        return msg

    except Exception as e:
        msg = {
            "status": "error",
            "error": str(e),
            "pin": pin,
            "value": value
        }
    pub.send_json(msg)

def handle_commands():
    global current_state, ads_devices, mcp_devices, dht11_dev
    while True:
        try:
            msg = rep.recv_json()
            cmd = msg.get("cmd")
            if cmd == "set":
                rep.send_json(set_output(msg["pin"], msg["state"]))
            elif cmd == "get_state":
                rep.send_json(build_state_response())
            else:
                rep.send_json({"status": "unsupported command"})
        except Exception as e:
            try:
                rep.send_json({"status": "error", "error": str(e)})
            except Exception:
                log.error(f"Error handling ZMQ command and replying: {e}")



if __name__ == "__main__":
    configure_gpio()
    ads_devices = configure_adc(i2c_bus1)
    mcp_devices = configure_mcp_devices(i2c_bus1)
    dht11_dev = configure_dht11()

    # Initial state scan before starting threads
    read_gpio()
    read_expanders()
    read_adc()

    threading.Thread(target=handle_commands, daemon=True).start()
    threading.Thread(target=monitor_inputs, daemon=True).start()
    threading.Thread(target=monitor_env, daemon=True).start()

    try:
        while not stop_event.is_set():
            # these are only monitoring the threads, not the devices
            with heartbeat_lock:
                if time.time() - last_40Hz_poll > TIMEOUT_40Hz:
                    raise RuntimeError("Heartbeat timeout: No input read in the last 0.5 seconds")
                if time.time() - last_5s_poll > TIMEOUT_5s:
                    raise RuntimeError("Heartbeat timeout: No environment read in the last 60 seconds")
                notifier.notify("WATCHDOG=1")
            stop_event.wait(TIMEOUT_40Hz)

    except KeyboardInterrupt:
        stop_event.set()  # Signal threads to stop
        time.sleep(0.1)  # Give threads a moment to exit
        GPIO.cleanup()
        log.info("HAL shutdown gracefully.")

    except Exception as e:
        log.exception("Unhandled exception in main loop")
        stop_event.set()
   
