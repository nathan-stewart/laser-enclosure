#!/usr/bin/env python3
import sys
import logging
import argparse
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
wait_30s = 30.0
wait_config = 10.0  # seconds
TIMEOUT_40Hz = 0.5  # seconds
TIMEOUT_30s = 300.0  # seconds
heartbeat_lock = threading.Lock()
stop_event = threading.Event()
gpio_configured = threading.Event()
last_40Hz_poll = time.time()
last_30s_poll = time.time()
error_threshold = 5
missing_devices = []


pub = None
rep = None

MCP23017_ADDRESSES = list(set([addr for addr, pin in MCP23017_PINS.values()]))
ADS1115_ADDRESSES = list(set([addr for addr, pin in pinmap.ADS1115_PINS.values()]))

mcp_devices = []
ads_devices = {}
dht11_dev = None

# State tracking
current_state = {}
for pin_dict in (RPi_INPUT_PINS, RPi_OUTPUT_PINS, pinmap.MCP23017_PINS, pinmap.ADS1115_PINS):
    for pin_name in pin_dict:
        current_state[pin_name] = None
current_state['i_airtemp'] = None
current_state['i_humidity']    = None
current_state['error_count'] = {
    "adc": 0,
    "mcp": 0,
    "dht11": 0
}
previous_state = current_state.copy()

def configure_gpio():
    global current_state, gpio_configured, log
    """Configure Raspberry Pi GPIO pins."""
    GPIO.setmode(GPIO.BCM)
    try:
        for name in RPi_INPUT_PINS:
            GPIO.setup(RPi_INPUT_PINS[name], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            log.debug(f"RPi GPIO input pin {name} configured on BCM pin {RPi_INPUT_PINS[name]}.")
        log.debug(f"{RPi_OUTPUT_PINS}")
        for name in RPi_OUTPUT_PINS:
            GPIO.setup(RPi_OUTPUT_PINS[name], GPIO.OUT, initial=GPIO.LOW)
            current_state[name] = GPIO.input(RPi_OUTPUT_PINS[name])
            log.debug(f"RPi GPIO output pin {name} configured on BCM pin {RPi_OUTPUT_PINS[name]}.")
        log.debug("RPi GPIOs configured.")
        gpio_configured.set()  # Signal that GPIO configuration is complete
    except KeyError as e:
        log.error(f"Configuration error: RPi_GPIO_PINS dictionary is missing key {e}.")

def configure_adc(i2c_bus):
    global ads_devices, log
    for address in ADS1115_ADDRESSES:
        try:
            ads = ADS1115(i2c_bus, address=address)
            # Register both expected channels from this address
            for name, (addr, chan) in pinmap.ADS1115_PINS.items():
                if addr == address:
                    ads_devices[name] = {
                        'ads': ads,
                        'analog_in': AnalogIn(ads, getattr(ADS1115, f'P{chan}')),
                        'address': address,
                        'channel': chan
                    }
            if address in missing_devices:
                missing_devices.remove(address)
            log.info(f"ADS1115 detected at address 0x{address:02X}.")
        except Exception as e:
            if address not in missing_devices:
                missing_devices.append(address)
                log.warning(f"Failed to configure ADS1115 at 0x{address:02X}: {e}")


def configure_expanders(i2c_bus):
    global mcp_devices, log
    """Detect and configure MCP23017 devices with pin directions from MCP23017_PINS."""
    found_addresses = set()

    for name, (address, pin) in MCP23017_PINS.items():
        if address in found_addresses:
            continue
        try:
            mcp = MCP23017(i2c_bus, address=address)
            mcp_devices.append((address, mcp))
            found_addresses.add(address)
            log.info(f"MCP23017 configured at address 0x{address:02X}.")
            if address in missing_devices:
                missing_devices.remove(address)
        except Exception as e:
            if address not in missing_devices:
                missing_devices.append(address)
                log.error(f"Error configuring MCP23017 at address 0x{address:02X}: {e}")

    for name, (address, pin) in MCP23017_PINS.items():
        for addr, mcp in mcp_devices:
            if addr == address:
                try:
                    if name in pinmap.RPi_OUTPUT_PINS:
                        # This is unlikely, but just in case
                        mcp.setup(pin, mcp.OUT)
                        mcp.output(pin, 0)  # default LOW
                    else:
                        mcp.setup(pin, mcp.IN)
                        mcp.pullups |= (1 << pin)
                except Exception as e:
                    log.error(f"Failed to configure MCP23017 pin {name} on address 0x{address:02X}: {e}")


def configure_dht11():
    global dht11_dev, log
    """Configure DHT11 sensor."""
    try:
        # Initialize the DHT11 sensor
        dht11_dev = adafruit_dht.DHT11(DHT11_PINS['i_dht11'])
        log.info("DHT11 sensor found.")
        if 'dht11' in missing_devices:
            missing_devices.remove('dht11')
    except Exception as e:
        if 'dht11' in missing_devices:
            missing_devices.remove('dht11')
        log.error(f"DHT11 sensor not configured {e}.")

def configure():
    global mcp_devices, ads_devices, dht11_dev, log
    configure_gpio()
    while not stop_event.is_set():
        try:
            if not mcp_devices:
                configure_expanders(i2c_bus1)
            if not ads_devices:
                configure_adc(i2c_bus1)
            if not dht11_dev:
                configure_dht11()
            if mcp_devices and ads_devices and dht11_dev:
                log.info("All devices configured successfully.")
                break
        except Exception as e:
            pass
        stop_event.wait(wait_config)


def read_gpio():
    global current_state, log
    for name, pin in RPi_INPUT_PINS.items():
        try:
            current_state[name] = GPIO.input(pin)
        except Exception as e:
            raise RuntimeError(f"GPIO read error on pin {name}: {e}")

def read_expanders():
    global current_state, mcp_devices, log
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
    global current_state, ads_devices, log
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
    global dht11_dev, current_state, last_30s_poll, log
    if dht11_dev:
        try:
            current_state['i_airtemp'] = dht11_dev.temperature
            current_state['i_humidity'] = dht11_dev.humidity
            last_30s_poll = time.time()
            if current_state['error_count']['dht11'] > 0:
                log.info("Recovered from DHT11 error.")
                current_state['error_count']['dht11'] = 0
        except RuntimeError as e:
            current_state['error_count']['dht11'] += 1
            if current_state['error_count']['dht11'] == error_threshold:
                log.warning(f"DHT11 read error: {e}")

def monitor_inputs():
    """Monitor input pins (RPi GPIO, MCP23017) and publish state changes."""
    global current_state, last_40Hz_poll, previous_state, pub, log
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
    global last_30s_poll, current_state, previous_state, pub, log
    while not stop_event.is_set():
        read_environment()
        with heartbeat_lock:
            last_30s_poll = time.time()
            if time.time() - last_30s_poll > TIMEOUT_40Hz:
                raise RuntimeError("Input read timeout")
        publish_deltas()
        stop_event.wait(wait_30s)  # 30 second polling interval

def build_state_response():
    global current_state, previous_state, log
    rep.send_json( {
        "state": {
            **{k: v for k, v in current_state.items()
               if k not in ("i_airtemp", "i_humidity", "error_count")},
            "i_airtemp": current_state["i_airtemp"],
            "i_humidity": current_state["i_humidity"]
        },
        "error_count": current_state["error_count"]
    })

def publish_deltas():
    global current_state, previous_state, pub, log
    deltas = {k: v for k, v in current_state.items() if previous_state.get(k) != v}
    if deltas:
        msg = {
            "topic": "state/update",
            "deltas": deltas,
            "timestamp": time.time()
        }
        pub.send_json(msg)
        log.debug(f"Published state update: {deltas}")
        log.debug(f"Msg: {msg}")
    previous_state.update(current_state)

def set_output(pin, value):
    """Set the state of an output pin."""
    global current_state, mcp_devices, rep, log
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

    except Exception as e:
        msg = {
            "status": "error",
            "error": str(e),
            "pin": pin,
            "value": value
        }
    pub.send_json(msg)

def handle_commands():
    global current_state, ads_devices, mcp_devices, dht11_dev, rep, log
    while True:
        try:
            msg = rep.recv_json()
            cmd = msg.get("cmd")
            if cmd == "set":
                set_output(msg["pin"], msg["state"])
            elif cmd == "get_state":
                build_state_response()
            else:
                rep.send_json({"status": "unsupported command"})
        except Exception as e:
            try:
                rep.send_json({"status": "error", "error": str(e)})
            except Exception:
                log.error(f"Error handling ZMQ command and replying: {e}")

def thread_wrapper(func):
    try:
        log.exception(f"Thread {func.__name__} started")
        func()
    except Exception as e:
        log.exception(f"Thread {func.__name__} crashed: {e}")
        stop_event.set()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="HAL Watcher")
    parser.add_argument(
        "--log", "-l",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Set the logging level (default: INFO)"
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log),
        format="%(asctime)s [%(levelname)s] %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)]
    )
    log = logging.getLogger("HAL_Watcher")

    log.info("Starting HAL ...")
    # ZMQ setup
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind("tcp://*:5556")
    log.info("ZeroMQ publisher bound to tcp://*:5556")

    rep = ctx.socket(zmq.REP)
    rep.bind("tcp://*:5557")
    log.info("ZeroMQ replier bound to tcp://*:5557")

    threading.Thread(target=thread_wrapper, args=(configure,), daemon=True).start()
    gpio_configured.wait()  # Wait for GPIO configuration to complete
    threading.Thread(target=thread_wrapper, args=(handle_commands,), daemon=True).start()
    threading.Thread(target=thread_wrapper, args=(monitor_inputs,), daemon=True).start()
    threading.Thread(target=thread_wrapper, args=(monitor_env,), daemon=True).start()

    try:
        while not stop_event.is_set():
            # these are only monitoring the threads, not the devices
            with heartbeat_lock:
                if time.time() - last_40Hz_poll > TIMEOUT_40Hz:
                    raise RuntimeError("Heartbeat timeout: No input read in the last 0.5 seconds")
                if time.time() - last_30s_poll > TIMEOUT_30s:
                    raise RuntimeError("Heartbeat timeout: No environment read in the last 60 seconds")
                notifier.notify("WATCHDOG=1")
            stop_event.wait(TIMEOUT_40Hz)
        GPIO.cleanup()
        log.info("HAL shut down normally.")

    except KeyboardInterrupt:
        stop_event.set()  # Signal threads to stop
        time.sleep(0.1)  # Give threads a moment to exit
        GPIO.cleanup()
        log.info("HAL shut down gracefully.")

    except Exception as e:
        log.exception("Unhandled exception in main loop")
        stop_event.set()
        GPIO.cleanup()

    rep.close()
    pub.close()
    ctx.term()
