#!/usr/bin/env python3
# i2c_devices.py
import sys
import logging
import argparse
import zmq
import copy
import time
import json
import os
import struct
import threading
import expander

if sys.argv is None:
    argv = sys.argv[1:]  # exclude script name
parser = argparse.ArgumentParser(description="HAL Watcher")
parser.add_argument(
    "--log", "-l",
    default="INFO",
    choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
    help="Set the logging level (default: INFO)"
)

parser.add_argument(
    "--mock", "-m",
    action="store_true",
    help="Run HAL with mock I2C devices for testing on non-Pi systems"
)
args = parser.parse_args()

sys.path.insert(0, os.path.dirname(__file__))
from pinmap import *

if args.mock:
    import i2c_devices
    i2c_devices.USE_MOCK = True
from i2c_devices import configure_mcp23017, read_mcp23017

from sdnotify import SystemdNotifier
from collections import deque

wait_40Hz = 1.0 / 40.0
wait_60s = 60.0
wait_config = 10.0  # seconds
TIMEOUT_40Hz = 0.5  # seconds
TIMEOUT_60s = 180  # seconds
heartbeat_lock = threading.Lock()
stop_event = threading.Event()
gpio_configured = threading.Event()
state_lock = threading.Lock()
last_40Hz_poll = 0
last_60s_poll = 0
error_threshold = 5
DEBOUNCE_LEN = 3

pub = None
rep = None

SEESAW_ADDRESS = 0x36
debounce = {}           # name -> deque

expanders = {}
expanders[0x20] = Expander(name=f"MCP@{hex(0x20)}")       
expanders[0x20].input(0,'i_fp0')
expanders[0x20].input(1,'i_fp1')
expanders[0x20].input(2,'i_fp2')
expanders[0x20].input(3,'i_fp3')
expanders[0x20].input(4,'i_btn_estop')
expanders[0x20].input(5,'i_btn_fire')
expanders[0x20].output(0, 'o_fp0')
expanders[0x20].output(1, 'o_fp1')
expanders[0x20].output(2, 'o_fp2')
expanders[0x20].output(3, 'o_fp3')
expanders[0x20].configure()

expanders[0x21] = Expander(name=f"MCP@{hex(0x21)}")
expanders[0x21].input(0, 'i_mask_encoder')
expanders[0x21].input(1, 'i_axis_x')
expanders[0x21].input(2, 'i_axis_z')
expanders[0x21].input(3, 'i_coarse')
expanders[0x21].input(4, 'i_fine')
expanders[0x21].output(0, 'o_mask_encoder')
expanders[0x21].configure()

adc = Adc(adc_dev, name="ADC")
adc.input(0, "i_air_supply")
adc.input(1, "i_co2_supply")
adc.configure()

bme280 = Ambient(name=f"BME280@{hex(0x76)}")
bme280.input('temperature', 'ambient_temp')
bme280.input('humidity',    'ambient_humidity')
bme280.input('pressure',    'ambient_pressure')
bme280.configure()

last_stable_state = {}  # name -> last confirmed value
log = None
last_heartbeat = time.time()

def control_heartbeat_listener():
    global last_heartbeat
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect("tcp://localhost:5558")
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    while True:
        try:
            msg = sub.recv_string()
            if msg == "heartbeat":
                last_heartbeat = time.time()
        except zmq.ZMQError:
            pass

def monitor_control_heartbeat():
    global last_heartbeat
    while True:
        if time.time() - last_heartbeat > 2:
            print("WARNING: No heartbeat from control process. Taking emergency action.")
            # e.g., shut off laser GPIO
            shutdown_laser()
        time.sleep(1)

def shutdown_laser():
    print("Laser power disabled due to control heartbeat loss.")
    for output in RPi_OUTPUT_PINS.keys():
        set_output(output, 0)

# State tracking
with state_lock:
    current_state = {}
    for pin_dict in (RPi_INPUT_PINS, RPi_OUTPUT_PINS, pinmap.MCP23017_PINS, pinmap.ADS1115_PINS):
        for pin_name in pin_dict:
            current_state[pin_name] = None
    current_state['i_airtemp'] = None
    current_state['i_humidity']    = None
    current_state['i_ambient_pressure'] = None
    current_state['error_count'] = {
        "ads1115": 0,
        "mcp23017": 0,
        "bme280": 0
    }
    previous_state = copy.deepcopy(current_state)

def configure_gpio():
    global current_state, debounce, last_stable_state
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

        # initialize debounce tracking
        for name in list(RPi_INPUT_PINS.keys()):
            debounce[name] = deque(maxlen=DEBOUNCE_LEN)
            last_stable_state[name] = None

        gpio_configured.set()  # Signal that GPIO configuration is complete
    except KeyError as e:
        log.error(f"Configuration error: RPi_GPIO_PINS dictionary is missing key {e}.")

def configure():
    global debounce, last_stable_state
    configure_gpio()

    while not stop_event.is_set():
        try:
            configure_mcp23017()                    
            configure_ads1115()
            configure_encoder()
            configure_bme280()

            if len(missing_devices) == 0:
                log.info("All devices configured successfully.")
                break

        except Exception as e:
            pass
        stop_event.wait(wait_config)

def read_gpio():
    global debounce
    for name, pin in RPi_INPUT_PINS.items():
        try:
            val = GPIO.input(pin)
            debounce[name].append(val)
        except Exception as e:
            raise RuntimeError(f"GPIO read error on pin {name}: {e}")


def read_environment():
    global current_state
    if BME280_ADDRESS in missing_devices:
        return
    try:
        temp, pressure, humidity = read_bme280()
        current_state['i_airtemp'] = temp
        current_state['i_humidity'] = humidity
        current_state['i_ambient_pressure'] = pressure
        current_state['error_count']['bme280'] = 0
        if current_state['error_count']['bme280'] > 0:
            log.info("Recovered from BME280 error.")
            current_state['error_count']['bme280'] = 0
    except (Exception, RuntimeError) as e:
        current_state['error_count']['bme280'] += 1
        if current_state['error_count']['bme280'] == error_threshold:
            log.warning(f"BME280 read error: {e}")

def monitor_40Hz():
    global current_state, previous_state, last_40Hz_poll
    while not stop_event.is_set():
        with state_lock:
            with heartbeat_lock:
                last_40Hz_poll = time.time()

            read_gpio()
            for pin, history in debounce.items():
                if len(history) == DEBOUNCE_LEN and all(v == history[0] for v in history):
                    if last_stable_state[pin] != history[0]:
                        last_stable_state[pin] = history[0]
                        current_state[pin] = history[0]

            for address, exp in expanders.items():
                for name, (pin, _) in exp.inputs.items():
                    val = exp.dev.get_pin(pin).value
                    debounce[name].append(val)

            current_state.update(adc.read())
            
            read_encoder_deltas()


            # if previous_state != current_state:
            publish_state()
            previous_state = copy.deepcopy(current_state)

        stop_event.wait(wait_40Hz)


def monitor_60s():
    """Monitor environment and publish state changes."""
    global last_60s_poll, current_state, pub
    while not stop_event.is_set():
        with state_lock:
            with heartbeat_lock:
                last_60s_poll = time.time()
                if time.time() - last_60s_poll > TIMEOUT_60s:
                    raise RuntimeError("Input read timeout")
            read_environment()
            # publish_state()
            log.debug(f"Current state: {json.dumps(current_state, indent=2)}")
        stop_event.wait(wait_60s)

def publish_state():
    global current_state
    pub.send_multipart([b'hal', json.dumps({"state": current_state}).encode()])

def set_output(pin, value):
    """Set the state of an output pin."""
    global current_state, rep, mcp23017_devices

    try:
        with state_lock:
            if pin in RPi_OUTPUT_PINS:
                # Set RPi GPIO pin
                GPIO.output(RPi_OUTPUT_PINS[pin], GPIO.HIGH if value else GPIO.LOW)
                current_state[pin] = GPIO.input(RPi_OUTPUT_PINS[pin])
                if current_state[pin] != (1 if value else 0):
                    raise RuntimeError(f"Failed to set RPi output pin {pin} to {value}")

            elif pin in MCP23017_PINS:
                # Set MCP23017 pin
                addr, mcp_pin = MCP23017_PINS[pin]
                mcp = mcp23017_devices.get(addr)
                if not mcp:
                    raise RuntimeError(f"MCP23017 at 0x{addr:02X} not available")

                mcp.setup(mcp_pin, mcp.OUT)
                mcp.output(mcp_pin, 1 if value else 0)
                current_state[pin] = 1 if value else 0

            else:
                raise ValueError(f"Unknown pin: {pin}")

            rep.send_json({
                "status": "ok",
                "pin": pin,
                "value": value
            })

    except Exception as e:
        log.error(f"set_output error on pin '{pin}': {e}")
        rep.send_json({
            "status": "error",
            "pin": pin,
            "value": value,
            "error": str(e)
        })

def handle_commands():
    global current_state, mcp23017_devices, rep
    while not stop_event.is_set():
        try:
            msg = rep.recv_json()
            cmd = msg.get("cmd")
            if cmd == "set":
                set_output(msg["pin"], msg["state"])
            else:
                rep.send_json({"status": "unsupported command"})
        except Exception as e:
            try:
                rep.send_json({"status": "error", "error": str(e)})
            except Exception:
                log.error(f"Error handling ZMQ command and replying: {e}")

def thread_wrapper(func):
    try:
        log.info(f"Thread {func.__name__} started")
        func()
    except Exception as e:
        log.exception(f"Thread {func.__name__} crashed: {e}")
        stop_event.set()


def main(argv=None):
    global pub, rep, ctx, log

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

    threads = []
    threads.append(threading.Thread(target=thread_wrapper, args=(configure,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, args=(handle_commands,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, args=(monitor_40Hz,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, args=(monitor_60s,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, args=(control_heartbeat_listener), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, args=(monitor_control_heartbeat), daemon=True))
    
    threads[0].start()
    gpio_configured.wait()  # Wait for GPIO configuration to complete
    for thread in threads[1:]:
        thread.start()
    log.info("HAL threads started.")

    try:
        while not stop_event.is_set():
            # these are only monitoring the threads, not the devices
            with heartbeat_lock:
                if time.time() - last_40Hz_poll > TIMEOUT_40Hz:
                    raise RuntimeError("Heartbeat timeout: No input read in the last 0.5 seconds")
                if time.time() - last_60s_poll > TIMEOUT_60s:
                    raise RuntimeError("Heartbeat timeout: No environment read in the last 60 seconds")
                notifier.notify("WATCHDOG=1")
            stop_event.wait(TIMEOUT_40Hz)
        log.info("HAL shutting down")

    except KeyboardInterrupt:
        log.info("Terminated by user")

    except Exception as e:
        log.exception("Unhandled exception in main loop")

    stop_event.set()  # Signal threads to stop
    time.sleep(0.1)  # Give threads a moment to exit
    GPIO.cleanup()
    for thread in threads:
        thread.join()

    rep.close()
    pub.close()
    ctx.term()
    log.info("HAL shutdown complete.")

if __name__ == "__main__":
    main(sys.argv)