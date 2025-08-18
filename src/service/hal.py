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
import signal

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
from service.devices import Gpio, Expander, AnalogRead, Environment, RotaryEncoder

from sdnotify import SystemdNotifier
notifier = SystemdNotifier()

wait_inputs = 1.0 / 40.0   # 40Hz for inputs
wait_sensors = 1.0 / 10.0  # 10Hz for sensors
wait_env = 5.0             # 30 seconds for environment
wait_config = 2  # seconds -  rescan periodically if lost comms
timeout_input = 0.5  # seconds
timeout_env = 180  # seconds
heartbeat_lock = threading.Lock()
stop_event = threading.Event()
state_lock = threading.Lock()
publish_lock = threading.Lock()
last_input_poll = last_sensor_poll = last_env_poll = time.time()
error_threshold = 5
DEBOUNCE_LEN = 3

def graceful_stop(signum, frame):
    log.info(f"Received signal {signum}, shutting down gracefully...")
    stop_event.set()

for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
    signal.signal(sig, graceful_stop)

pub = None
rep = None

current_state = {}
previous_snapshot = {}

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
expanders[0x20] = Expander(addr=0x20)
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

expanders[0x21] = Expander(addr=0x21)
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

encoder = RotaryEncoder()
current_state["encoder_delta"] = 0

adc = AnalogRead()
adc.input(0, "i_air_supply")
adc.input(1, "i_co2_supply")
for name in adc.inputs.keys():
    current_state[name] = None

ambient = Environment()
ambient.input('temperature', 'i_ambient_temp')
ambient.input('humidity',    'i_ambient_humidity')
ambient.input('pressure',    'i_ambient_pressure')
for name in ambient.inputs.keys():
    current_state[name] = None

log = None
last_heartbeat = time.time()

# virtual outputs may only drive physical outputs - only intended for gpios
virtual_outputs: dict[str,tuple[str,...]] = {"o_dehumidifier" : ("o_k8_dry_heat","o_k6_dry_fan")}
def compute_locked_pins(virtual_outputs, gpio, expanders):
    # all physical output names we own
    phys = set(gpio.outputs.keys()) | {
        n for e in expanders.values() for n in e.outputs.keys()
    }
    # all members referenced by virtuals
    members = {p for pins in virtual_outputs.values() for p in pins}
    # lock only the ones that are real physical outputs
    return phys & members
locked_pins = compute_locked_pins(virtual_outputs, gpio, expanders)

# Initialize virtuals to off
with state_lock:
    for virt in virtual_outputs.keys():
        current_state[virt] = 0

def publish_state(snapshot):
    with publish_lock:
        # change filter
        if snapshot == previous_snapshot:
            return
        
        pub.send_multipart([b"hal", json.dumps({"state": snapshot}).encode()])
        previous_snapshot.clear()
        previous_snapshot.update(snapshot)

def write_pin(pin, val):
    """Write one physical pin."""
    v = 1 if val else 0
    if pin in gpio.outputs:
        log.info("Setting GPIO %s to %d", pin, v)
        gpio.write(pin, v)
        return True
    for exp in expanders.values():
        if pin in exp.outputs:
            exp.write(pin, v)
            return True
    log.error("Unknown output pin %s", pin)
    return False

def set_output(name, value):
    v = 1 if value else 0
    # expand targets: virtual -> members, else itself
    targets = virtual_outputs.get(name, (name,))

    with state_lock:
        ok = True
        # only touch pins that need a change
        to_change = [p for p in targets if current_state.get(p, 0) != v]

        for pin in to_change:
            if not write_pin(pin, v):
                ok = False
            else:
                current_state[pin] = v

        changed = bool(to_change)
        snap = dict(current_state)

    if changed:
        publish_state(snap)
    return ok


def control_heartbeat_listener():
    global last_heartbeat
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect("tcp://localhost:5558")
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.setsockopt(zmq.RCVTIMEO, 1000)  # 1s
    while not stop_event.is_set():
        try:
            msg = sub.recv_string()
            if msg == "heartbeat":
                last_heartbeat = time.time()
        except zmq.Again:
            continue
        except Exception:
            log.exception("heartbeat listener")
            break

def monitor_control_heartbeat():
    global last_heartbeat
    while not stop_event.is_set():
        if time.time() - last_heartbeat > 2:
            log.warning("WARNING: No heartbeat from control process. Taking emergency action.")
            # e.g., shut off laser GPIO
            shutdown_laser()
        time.sleep(1)

def shutdown_laser():
    log.warning("Laser power disabled due to control heartbeat loss.")
    for name in list(gpio.outputs.keys()):
        set_output(name, 0)

def configure_thread():
    while not stop_event.is_set():
        try:
            for expander in expanders.values():
                if expander.configure():
                    log.info(f"Configured expander at address {expander.name}")
            if adc.configure():
                log.info(f"Configured ADC at address {adc.name}")
            if ambient.configure():
                log.info(f"Configured ambient sensors {ambient.name}")
            if encoder.configure():
                log.info(f"Configured encoder at address {encoder.name}")
        except Exception as e:
            log.warning("Configure pass error: %s", e)
        stop_event.wait(wait_config)

def monitor_inputs():
    global current_state, last_input_poll
    while not stop_event.is_set():
        with state_lock:
            with heartbeat_lock:
                last_input_poll = time.time()

            for name, value in gpio.read():
                if name and name.startswith('i_'):
                    current_state[name] = int(bool(value))

            for expander in expanders.values():
                for name, value in expander.read():
                    if name and name.startswith('i_'):
                        current_state[name] = int(bool(value))

            for name, delta in encoder.read_delta():
                if current_state[name] is None:
                    current_state[name] = 0
                current_state[name] += delta

            snapshot = dict(current_state)

        publish_state(snapshot)
        stop_event.wait(wait_inputs)

def monitor_sensors():
    global current_state, last_sensor_poll
    while not stop_event.is_set():
        with state_lock:
            with heartbeat_lock:
                last_sensor_poll = time.time()

            for name, value in adc.read():
                if name:
                    current_state[name] = value

            snapshot = dict(current_state)

        publish_state(snapshot)
        stop_event.wait(wait_sensors)

def monitor_env():
    global last_env_poll
    while not stop_event.is_set():
        with state_lock:
            last_env_poll = time.time()
            for name, value in ambient.read():
                if name:
                    current_state[name] = value
            snapshot = dict(current_state)
        publish_state(snapshot)
        stop_event.wait(wait_env)


def handle_commands():
    # optional, but lets the loop exit when stop_event is set
    try:
        rep.setsockopt(zmq.RCVTIMEO, 1000)  # 1s
    except Exception:
        pass

    while not stop_event.is_set():
        try:
            msg = rep.recv_json()  # blocks up to RCVTIMEO if set
        except zmq.Again:
            continue  # timeout -> check stop_event and loop
        except Exception:
            log.exception("handle_commands: recv failed")
            continue

        try:
            cmd = (msg.get("cmd") or "").lower()

            if cmd == "set":
                # strict payload
                if "pin" not in msg or "state" not in msg:
                    rep.send_json({"status": "error", "error": "missing 'pin' or 'state'"})
                    continue
                name = msg["pin"]
                val  = 1 if msg["state"] else 0

                ok = set_output(name, val)
                if ok:
                    rep.send_json({"status": "ok"})
                else:
                    rep.send_json({"status": "error", "error": "read-only or unknown pin"})
                continue

            if cmd == "get_status":
                with state_lock:
                    snap = dict(current_state)
                    for virt, pins in virtual_outputs.items():
                        snap[virt] = int(any(snap.get(p, 0) for p in pins))
                rep.send_json({"status": "ok", "state": snap, "ts": time.time()})
                continue

            rep.send_json({"status": "unsupported command"})

        except Exception as e:
            # We DID receive a request; safe to reply once.
            rep.send_json({"status": "error", "error": str(e)})
            log.exception("handle_commands: processing error")

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

    notifier.notify("READY=1")

    threads = []
    threads.append(threading.Thread(target=thread_wrapper, name="ConfigureThread", args=(configure_thread,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, name="CommandHandler", args=(handle_commands,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, name="InputPolling", args=(monitor_inputs,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, name="SensorPolling", args=(monitor_sensors,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, name="EnvPolling", args=(monitor_env,), daemon=True))
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
                if time.time() - last_input_poll > timeout_input:
                    raise RuntimeError(f"Heartbeat timeout: No input read in the last {timeout_input:0.2f} seconds")
                if time.time() - last_env_poll > timeout_env:
                    raise RuntimeError(f"Heartbeat timeout: No environment read in the last {timeout_env} seconds")
                notifier.notify("WATCHDOG=1")
            stop_event.wait(timeout_input)
        log.info("HAL shutting down")

    except KeyboardInterrupt:
        log.info("Terminated by user")

    except Exception as e:
        log.exception("Unhandled exception in main loop")

    stop_event.set()  # Signal threads to stop
    time.sleep(0.1)  # Give threads a moment to exit
    notifier.notify("STOPPING=1")
    for thread in threads:
        thread.join()

    rep.close()
    pub.close()
    ctx.term()
    log.info("HAL shutdown complete.")

if __name__ == "__main__":
    main(sys.argv)
