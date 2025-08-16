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
import signal

state_lock = threading.Lock()
stop_event = threading.Event()
state_hal = {}
log = None
pub = None
sub = None

def graceful_stop(signum, frame):
    log.info(f"Received signal {signum}, shutting down gracefully...")
    stop_event.set()

for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
    signal.signal(sig, graceful_stop)


# Track activity for backlight control
USER_INPUTS = {"i_lid", "i_fp0", "i_fp1", "i_fp2", "i_fp3", "i_btn_estop", "i_btn_fire", "i_mask_encoder", "i_axis_x", "i_axis_z", "i_coarse", "i_fine"}
ENCODER_INPUTS = {"i_mask_encoder"}  # adjust if you rename or track deltas elsewhere
ENCODER_DELTA_THRESHOLD = 1  # define what counts as significant
previous_inputs = {}

# backlight control
IDLE_TIMEOUT = 5 * 60  # seconds
last_activity = time.time()
last_display_state = None
rh_on = 45.0  # turn ON at/above this RH
rh_off = 40.0 # turn OFF at/below this RH

def getb(s, k):  # boolean read with default 0
    return bool(s.get(k, 0))

def num_ok(x):
    return isinstance(x, (int, float)) and math.isfinite(x)

def clamp01(x):
    return 0.0 if x < 0.0 else 100.0 if x > 100.0 else x

# Rules for outputs - this is the brain stem - only handles low level rules
RULES = {
    "o_k1_laser"    : lambda: i_btn_estop, i_btn_fire : not (i_btn_estop or i_btn_fire),
    "o_k2_hpa"      : lambda: i_btn_estop, i_m7 : not i_btn_estop and i_m7,
    "o_k3_fire"     : lambda: i_btn_fire : i_btn_fire,
    "o_k5_lpa"      : lambda: i_btn_estop, i_m8 : not i_btn_estop and i_m8,
    "o_k7_exhaust"  : lambda: i_btn_estop, i_m8 : not i_btn_estop and i_m8,
    "o_k4_light"    : lambda:  None, # Handled in application
    "o_k6_dry_fan"  : dewpoint_rule,
    "o_k8_dry_heat" : dewpoint_rule,
}

def start_heartbeat():
    while True:
        pub.send_string("heartbeat")
        time.sleep(1)

def num_ok(x):
    return isinstance(x, (int, float)) and math.isfinite(x)
def dewpoint_rule(i_ambient_humidity=None,
                  o_k6_dry_fan=0, o_k8_dry_heat=0,
                  rh_on=45.0, rh_off=40.0,
                  min_on=120.0, min_off=120.0,
                  stale_timeout=180.0):
    now = time.monotonic()
    st = dewpoint_rule.__dict__      # tiny state store
    last_change = st.get("last_change", 0.0)
    last_seen   = st.get("last_seen", now)
    running = bool(o_k6_dry_fan) or bool(o_k8_dry_heat)

    # sample ingest
    rh = None
    if num_ok(i_ambient_humidity):
        rh = max(0.0, min(100.0, float(i_ambient_humidity)))
        st["last_seen"] = now

    # stale sensor → fail safe OFF (respect min_on)
    if now - st.get("last_seen", 0.0) > stale_timeout:
        if running and (now - last_change) >= min_on:
            st["last_change"] = now
            return 0
        return None

    # value hysteresis + explicit dwells
    new_val = None
    if rh is not None:
        if running and rh <= rh_off and (now - last_change) >= min_on:
            new_val = 0
        elif (not running) and rh >= rh_on and (now - last_change) >= min_off:
            new_val = 1

    if new_val is not None:
        st["last_change"] = now
        return new_val
    return None


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

def hal_listener():
    global last_activity, previous_inputs
    log.info("Starting hal listener...")
    while not stop_event.is_set():
        try:
            parts = sub.recv_multipart()
            if not parts:
                continue
            raw = parts[-1]
            msg = json.loads(raw)
            state = msg.get("state", msg)

            with state_lock:
                state_hal.update(state)
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
            # <<< release lock first
            apply_rules()
        except Exception as e:
            log.exception("hal_listener: receive/parse error")
            time.sleep(0.25)

def is_laser_active():
    return state_hal.get("o_k1_laser", 0) == 1

def set_backlight(state: bool):
    env = os.environ.copy()
    env["DISPLAY"] = ":0"
    # Optional: add XAUTHORITY if needed, uncomment if problems occur
    # env["XAUTHORITY"] = "/home/kiosk/.Xauthority"

    cmd = ["vcgencmd", "display_power", "1" if state else "0"]
    try:
        subprocess.run(cmd, env=env, check=True)
    except subprocess.CalledProcessError as e:
        print(f"Failed to set display power: {e}")

def idle_monitor():
    global last_display_state
    while not stop_event.is_set():
        now = time.time()
        idle_time = now - last_activity

        # Consider laser use as activity
        idle = idle_time > IDLE_TIMEOUT and not is_laser_active()
        log.debug(f"idle={idle}, idle_time={idle_time}, laser_active={is_laser_active()}")

        desired = not (idle)

        if desired != last_display_state:
            log.debug(f"backlight updated: {desired}")
            set_backlight(desired)
            last_display_state = desired

        stop_event.wait(30.0)

def hal_set(key, value):
    with state_lock:
        state_hal[key] = int(bool(value))
        outputs = {k: v for k, v in state_hal.items() if k.startswith("o_")}
    msg = json.dumps({"set": outputs})
    try:
        req.send_string(msg)
        log.debug(f"Sent outputs to HAL: {msg}")
        ack = req.recv_string()
        if ack != "ok":
            log.error(f"HAL did not acknowledge output update: {ack}")
    except Exception as e:
        log.error(f"Failed to send outputs to HAL: {e}")


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
    sub.setsockopt(zmq.SUBSCRIBE, b'')
    req = context.socket(zmq.REQ)
    req.connect("tcp://localhost:5557")      # HAL REQ/REP
    pub = context.socket(zmq.PUB)
    pub.bind("tcp://*:5558")                 # Supervisor PUB

    threads = []
    threads.append(threading.Thread(target=hal_listener))
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
