#!/usr/bin/env python3
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

import service.devices
service.devices.configure_mock(args.mock)
from service.devices import Gpio, MCP23017, ADS1115, BME280, QTEncoder

from sdnotify import SystemdNotifier
notifier = SystemdNotifier()

wait_40Hz = 1.0 / 40.0
wait_60s = 60.0
wait_config = 0.5  # seconds # rescan periodically if lost comms
TIMEOUT_40Hz = 0.5  # seconds
TIMEOUT_60s = 180  # seconds
heartbeat_lock = threading.Lock()
stop_event = threading.Event()
state_lock = threading.Lock()
last_40Hz_poll = 0
last_60s_poll = 0
error_threshold = 5
DEBOUNCE_LEN = 3

pub = None
rep = None

current_state = {}
previous_state = {}

gpio = Gpio()
gpio.input(22, "i_m7")
gpio.input(27, "i_m8")
gpio.input(17, "i_lid")
for name in gpio.inputs.keys():
    current_state[name] = 0

gpio.output(7,  "o_k1_laser",    0)
gpio.output(8,  "o_k2_hpa",      0)
gpio.output(25, "o_k3_fire",     0)
gpio.output(24, "o_k4_light",    0)
gpio.output(23, "o_k5_lpa",      0)
gpio.output(18, "o_k6_dry_fan",  0)
gpio.output(12, "o_k7_exhaust",  0)
gpio.output(16, "o_k8_dry_heat", 0)
for name in gpio.outputs.keys():
    current_state[name] = 0

expanders = {}
expanders[0x20] = MCP23017(addr=0x20)
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

expanders[0x21] = MCP23017(addr=0x21)
expanders[0x21].input(0, 'i_mask_encoder')
expanders[0x21].input(1, 'i_axis_x')
expanders[0x21].input(2, 'i_axis_z')
expanders[0x21].input(3, 'i_coarse')
expanders[0x21].input(4, 'i_fine')
expanders[0x21].output(0, 'o_mask_encoder')

for expander in expanders.values():
    for name in expander.inputs.keys():
        current_state[name] = None
    for name in expander.outputs.keys():
        current_state[name] = None

encoder = QTEncoder()
current_state["encoder_delta"] = None

adc = ADS1115()
adc.input(0, "i_air_supply")
adc.input(1, "i_co2_supply")
for name in adc.inputs.keys():
    current_state[name] = None

ambient = BME280()
ambient.input('temperature', 'i_ambient_temp')
ambient.input('humidity',    'i_ambient_humidity')
ambient.input('pressure',    'i_ambient_pressure')
for name in ambient.inputs.keys():
    current_state[name] = None

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
            log.warning("WARNING: No heartbeat from control process. Taking emergency action.")
            # e.g., shut off laser GPIO
            shutdown_laser()
        time.sleep(1)

def shutdown_laser():
    log.warning("Laser power disabled due to control heartbeat loss.")
    for output in gpio.outputs:
        set_output(output, 0)

def configure_thread():
    while not stop_event.is_set():
         # these return if configured - need to add loss detection
        for expander in expanders.values():
            expander.configure()
        adc.configure()
        ambient.configure()
        stop_event.wait(wait_config)

def monitor_40Hz():
    global current_state, last_40Hz_poll
    while not stop_event.is_set():
        with state_lock:
            with heartbeat_lock:
                last_40Hz_poll = time.time()

            for name, value in gpio.read():
                if name:
                    current_state[name] = value

            for expander in expanders.values():
                for name, value in expander.read():
                    if name:
                        current_state[name] = value

            for name, value in adc.read():
                if name:
                    current_state[name] = value

            # for name, value in next(encoder.read_delta()):
                # current_state[name] = value

            publish_state()
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
                for name, value in ambient.read():
                    if name:
                        current_state[name] = value

            log.debug(f"Current state: {json.dumps(current_state, indent=2)}")
        stop_event.wait(wait_60s)

def publish_state():
    global previous_state
    pub.send_multipart([b'hal', json.dumps({"state": current_state}).encode()])
    previous_state = copy.deepcopy(current_state)


def set_output(name, value):
    """Set the state of an output pin."""
    global current_state, rep, mcp23017_devices

    with state_lock:
        if name in gpio.outputs:
            gpio.write(name, value)
            return True

        else:
            for expander in expanders.values():
                if name in expander.outputs:
                    expander.write(name, value)
                    return True
    return False

def handle_commands():
    global current_state, mcp23017_devices, rep
    while not stop_event.is_set():
        try:
            msg = rep.recv_json()
            cmd = msg.get("cmd")
            if cmd == "set":
                if set_output(msg["pin"], msg["state"]):
                    rep.send_json({"status": "ok", "pin": msg["pin"], "state": msg["state"]})
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
    threads.append(threading.Thread(target=thread_wrapper, name="ConfigureThread", args=(configure_thread,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, name="CommandHandler", args=(handle_commands,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, name="40Hz", args=(monitor_40Hz,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, name="60s", args=(monitor_60s,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, name="HeartbeatListener", args=(control_heartbeat_listener,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, name="HeartbeatMonitor", args=(monitor_control_heartbeat,), daemon=True))

    threads[0].start()
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

    for thread in threads:
        thread.join()

    rep.close()
    pub.close()
    ctx.term()
    log.info("HAL shutdown complete.")

if __name__ == "__main__":
    main(sys.argv)
