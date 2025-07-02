#!/usr/bin/env python3
import sys
import os
import pwd
import argparse
import logging
import math
import zmq, json, time, threading
sys.path.insert(0, os.path.dirname(__file__))
import pinmap
import threading

state_lock = threading.Lock()
stop_event = threading.Event()
state_hal = {}
log = None
pub = None
sub = None

BACKLIGHT_PATH = "/sys/class/backlight/rpi_backlight/brightness"
BACKLIGHT_FULL = 255
BACKLIGHT_DIM = 20
IDLE_TIMEOUT = 5 * 60  # seconds 

last_activity = time.time()
last_backlight_state = None

RULES = {
    "o_k1_laser"  : lambda i_btn_estop, i_btn_fire : not (i_btn_estop or i_btn_fire),
    "o_k2_hpa"    : lambda i_btn_estop, i_m7 : not i_btn_estop and i_m7,
    "o_k3_fire"   : lambda i_btn_fire : i_btn_fire,
    "o_k5_lpa"    : lambda i_btn_estop, i_m8 : not i_btn_estop and i_m8,
    "o_k7_exhaust": lambda i_btn_estop, i_m8 : not i_btn_estop and i_m8,
    # "o_k4_light":    # lights are software button controlled at application layer
    # "o_k6_dry_fan":  # dehumidifier fan is controlled by dewpoint check
    # "o_k8_dry_heat": # dehumidifier heat is controlled by dewpoint check
}

def dew_point_c(temp_c, rh_percent):
    a = 17.62
    b = 243.12
    gamma = (a * temp_c) / (b + temp_c) + math.log(rh_percent / 100.0)
    dp = (b * gamma) / (a - gamma)
    return dp

def dewpoint_check():
    dewpoint_hyst_on = 3.0
    dewpoint_hyst_off = 6.0
    dehumidifier_period = 30.0

    while not stop_event.is_set():
        try:
            if "i_airtemp" in state_hal and "i_humidity" in state_hal:
                temperature = state_hal["i_airtemp"]
                humidity = state_hal["i_humidity"]
                dewpoint = dew_point_c(temperature, humidity)
                delta = dewpoint - temperature
                with state_lock:
                    if delta > dewpoint_hyst_on:
                        new_val = 1
                    elif delta < dewpoint_hyst_off:
                        new_val = 0
                    else:
                        new_val = None  # No change

                    if new_val is not None:
                        for key in ("o_k6_dry_fan", "o_k8_dry_heat"):
                            if state_hal.get(key) != new_val:
                                state_hal[key] = new_val
                log.debug("Dewpoint check: Temp: %.2f, Humidity: %.2f, Dewpoint: %.2f, Delta: %.2f",temperature, humidity, dewpoint, delta)
        except Exception as e:
            log.error(f"Error in dewpoint check: {e}")
        stop_event.wait(dehumidifier_period)

def publish_to_hal():
    with state_lock:
        outputs = {k: v for k, v in state_hal.items() if k.startswith("o_")}
    msg = json.dumps({"set": outputs})
    try:
        req.send_string(msg)
        ack = req.recv_string()
        if ack != "ok":
            log.error("HAL did not acknowledge output update:", ack)
    except Exception as e:
        log.error("Failed to send outputs to HAL:", e)

def apply_rules():
    log.debug("applying rules: %s", RULES)
    with state_lock:
        inputs = state_hal.copy()
        for output, rule in RULES.items():
            try:
                args = {k: inputs[k] for k in rule.__code__.co_varnames}
                result = int(bool(rule(**args)))
                if state_hal.get(output) != result:
                    state_hal[output] = result
            except KeyError as e:
                log.error(f"Missing input for {output}: {e}")


def active_users(exclude_user='kiosk'):
    users = set()
    try:
        with open("/var/run/utmp", "rb") as f:
            while True:
                entry = f.read(384)
                if not entry:
                    break
                username = entry[:32].split(b'\x00', 1)[0].decode(errors='ignore')
                if username and username != exclude_user:
                    users.add(username)
    except Exception:
        pass
    return users

def hal_listener():
    global last_activity
    while not stop_event.is_set():
        topic, raw = sub.recv_multipart()
        if topic == b'hal':
            with state_lock:
                state_hal.update(json.loads(raw).get("state", {}))
                last_activity = time.time()
            apply_rules()
            log.debug("HAL State Updated:", state_hal)

def set_backlight(level):
    try:
        with open(BACKLIGHT_PATH, "w") as f:
            f.write(str(level))
    except Exception as e:
        log.warning(f"Could not set backlight: {e}")

def backlight_monitor():
    global last_backlight_state
    while not stop_event.is_set():
        now = time.time()
        idle = (now - last_activity) > IDLE_TIMEOUT
        not_logged_in = len(active_users()) == 0
        log.debug(f"HAL Watcher - idle: {idle}, not_logged_in={not_logged_in}")
        desired = BACKLIGHT_DIM if (idle and not_logged_in) else BACKLIGHT_FULL
        if desired != last_backlight_state:
            set_backlight(desired)
            last_backlight_state = desired
        stop_event.wait(10.0)

def main(argv):
    global pub, sub, req, log
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
    log = logging.getLogger("Rules")

    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.connect("tcp://localhost:5556")      # HAL PUB
    sub.setsockopt(zmq.SUBSCRIBE, b'hal')
    req = context.socket(zmq.REQ)
    req.connect("tcp://localhost:5557")      # HAL REQ/REP
    pub = context.socket(zmq.PUB)
    pub.bind("tcp://*:5558")                 # Supervisor PUB

    threads = []
    threads.append(threading.Thread(target=hal_listener))
    threads.append(threading.Thread(target=dewpoint_check))
    threads.append(threading.Thread(target=backlight_monitor))

    log.info("Starting control threads...")
    for thread in threads:
        thread.start()

    try:
        while not stop_event.is_set():
            stop_event.wait(1.0)
    except KeyboardInterrupt:
        log.info("Shutting down...")
        stop_event.set()

    for thread in threads:
        thread.join()
    log.info("Exiting.")


if __name__ == "__main__":
    main(sys.argv)