#!/usr/bin/env python3
import sys
import os
import pwd
import argparse
import logging
import math
import zmq, json, time, threading
sys.path.insert(0, os.path.dirname(__file__))
import threading
import subprocess

state_lock = threading.Lock()
stop_event = threading.Event()
state_hal = {}
log = None
pub = None
sub = None

IDLE_TIMEOUT = 5 * 60  # seconds

last_activity = time.time()
last_display_state = None

RULES = {
    "o_k1_laser"    : lambda i_btn_estop, i_btn_fire : not (i_btn_estop or i_btn_fire),
    "o_k2_hpa"      : lambda i_btn_estop, i_m7 : not i_btn_estop and i_m7,
    "o_k3_fire"     : lambda i_btn_fire : i_btn_fire,
    "o_k5_lpa"      : lambda i_btn_estop, i_m8 : not i_btn_estop and i_m8,
    "o_k7_exhaust"  : lambda i_btn_estop, i_m8 : not i_btn_estop and i_m8,
    "o_k4_light"    : lambda : None, # Handled in application
    "o_k6_dry_fan"  : lambda : None, # Handled in dewpoint check thread
    "o_k8_dry_heat" : lambda : None, # Handled in dewpoint check thread
}

def start_heartbeat():
    while True:
        pub.send_string("heartbeat")
        time.sleep(1)

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
            if "i_ambient_temp" in state_hal and "i_ambient_humidity" in state_hal:
                temperature = state_hal["i_ambient_temp"]
                humidity = state_hal["i_ambient_humidity"]
                if isinstance(temperature, (int,float)) and isinstance(humidity, (int,float)):
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
            log.error(f"HAL did not acknowledge output update: {ack}")
    except Exception as e:
        log.error(f"Failed to send outputs to HAL: {e}")

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

USER_INPUTS = {"i_lid", "i_fp0", "i_fp1", "i_fp1", "i_fp2", "i_fp3", "i_btn_estop", "i_btn_fire", "i_mask_encoder", "i_axis_x", "i_axis_z", "i_coarse", "i_fine"}
ENCODER_INPUTS = {"i_mask_encoder"}  # adjust if you rename or track deltas elsewhere
ENCODER_DELTA_THRESHOLD = 1  # define what counts as significant


def hal_listener():
    global last_activity, previous_inputs
    while not stop_event.is_set():
        topic, raw = sub.recv_multipart()
        if topic == b'hal':
            state = json.loads(raw).get("state", {})
            with state_lock:
                state_hal.update(state)

                # Check for changes in user inputs
                changed = False
                for key in USER_INPUTS:
                    prev = previous_inputs.get(key)
                    curr = state.get(key)
                    if key in ENCODER_INPUTS:
                        try:
                            delta = abs(int(curr or 0) - int(prev or 0))
                            if delta >= ENCODER_DELTA_THRESHOLD:
                                changed = True
                        except Exception:
                            pass
                    elif curr is not None and curr != prev:
                        changed = True
                        log.debug(f"Input changed: {key} from {prev} to {curr}")

                if changed:
                    last_activity = time.time()

                previous_inputs = {k: state.get(k) for k in USER_INPUTS}

            apply_rules()

def is_laser_active():
    return state_hal.get("k1_laser", 0) == 1

def set_backlight(state: bool):
    cmd = ["vcgencmd", "display_power", "1" if state else "0"]
    try:
        subprocess.run(cmd, env=env, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to set display power: {e}")

def idle_monitor():
    global last_backlight_state
    while not stop_event.is_set():
        now = time.time()
        idle_time = now - last_activity

        # Consider laser use as activity
        idle = idle_time > IDLE_TIMEOUT and not is_laser_active()
        log.debug(f"idle={idle}, idle_time={idle_time}, laser_active={is_laser_active()}")

        desired = not (idle)

        if desired != last_backlight_state:
            log.debug(f"backlight updated: {desired}")
            set_backlight(desired)
            last_backlight_state = desired

        stop_event.wait(30.0)

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
    threads.append(threading.Thread(target=idle_monitor))
    threads.append(threading.Thread(target=start_heartbeat, daemon=True))

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
