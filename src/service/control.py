#!/usr/bin/env python3
import sys
import os
import pwd
import argparse
import logging
import math
import inspect
import threading
import subprocess
import signal
import zmq
import json
import time
import threading
sys.path.insert(0, os.path.dirname(__file__))

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
ACTIVITY_INPUTS = {"i_lid", "i_fp0", "i_fp1", "i_fp2", "i_fp3", "i_btn_estop", "i_btn_fire", "i_mask_encoder", "i_axis_x", "i_axis_z", "i_coarse", "i_fine"}
RULE_INPUTS = set(ACTIVITY_INPUTS) | {"i_ambient_humidity"}
ENCODER_INPUTS = {"i_mask_encoder"}  # adjust if you rename or track deltas elsewhere
ENCODER_DELTA_THRESHOLD = 1  # define what counts as significant
previous_inputs = {}

# backlight control
IDLE_TIMEOUT = 5 * 60  # seconds
last_activity = time.monotonic()
last_display_state = None

def getb(s, k):  # boolean read with default 0
    return bool(s.get(k, 0))

def num_ok(x):
    return isinstance(x, (int, float)) and math.isfinite(x)

def clamp01(x):
    return 0.0 if x < 0.0 else 100.0 if x > 100.0 else x

def dewpoint_rule(i_ambient_humidity=None, o_dehumidifier=0,
                  rh_on=50.0, rh_off=45.0,
                  min_on=30.0, min_off=30.0,
                  stale_timeout=180.0):
    now = time.monotonic()
    st = dewpoint_rule.__dict__
    last_change = st.get("last_change", 0.0)
    last_seen   = st.get("last_seen", 0.0)

    running = bool(o_dehumidifier)

    rh = None
    if isinstance(i_ambient_humidity, (int, float)) and math.isfinite(i_ambient_humidity):
        rh = max(0.0, min(100.0, float(i_ambient_humidity)))
        last_seen = now

    # stale -> fail safe OFF (respect min_on)
    if now - last_seen > stale_timeout:
        if running and (now - last_change) >= min_on:
            st["last_change"] = now; st["last_seen"] = last_seen
            return 0
        st["last_seen"] = last_seen
        return None

    cmd = None
    if rh is not None:
        if running and rh <= rh_off and (now - last_change) >= min_on:
            cmd = 0
        elif (not running) and rh >= rh_on and (now - last_change) >= min_off:
            cmd = 1

    if cmd is not None:
        st["last_change"] = now; st["last_seen"] = last_seen
        return cmd

    st["last_seen"] = last_seen
    return None

# Rules for outputs - this is the brain stem - only handles low level rules
RULES = {
    "o_k1_laser"      : lambda i_btn_estop=0, i_btn_fire=0  : not (i_btn_estop or i_btn_fire),
    "o_k2_hpa"        : lambda i_btn_estop=0, i_m7=0        : not i_btn_estop and i_m7,
    "o_k3_fire"       : lambda i_btn_fire=0,  i_btn_estop=0 : i_btn_fire and (not i_btn_estop),
    "o_k5_lpa"        : lambda i_btn_estop=0, i_m8=0        : not i_btn_estop and i_m8,
    "o_k7_exhaust"    : lambda i_btn_estop=0, i_m8=0        : not i_btn_estop and i_m8,
    "o_k4_light"      : (lambda : None), # Handled in application
    "o_dehumidifier"  : dewpoint_rule,
}

def start_heartbeat():
    while not stop_event.is_set():
        try:
            pub.send_string("heartbeat")
        except Exception:
            log.exception("heartbeat send failed")
        stop_event.wait(1.0)

def hal_set(name, value, timeout_ms=1000):
    payload = {"cmd": "set", "pin": name, "state": int(bool(value))}
    try:
        req.setsockopt(zmq.RCVTIMEO, timeout_ms)
        req.setsockopt(zmq.SNDTIMEO, timeout_ms)
    except Exception:
        pass
    req.send_json(payload)
    rep = req.recv_json()
    ok = str(rep.get("status", "")).lower() == "ok"
    if not ok:
        log.error("HAL NACK: %s", rep)
    return ok

def hal_get_status(timeout_ms=500):
    try:
        req.setsockopt(zmq.RCVTIMEO, timeout_ms)
        req.setsockopt(zmq.SNDTIMEO, timeout_ms)
    except Exception:
        pass
    req.send_json({"cmd":"get_status"})
    return req.recv_json()  # {"status":"ok","state":{...},"version":N,"ts":...}

def bind_kwargs_for(rule, state):
    if rule is None:
        return {}
    sig = inspect.signature(rule)
    kwargs = {}
    for name, p in sig.parameters.items():
        kwargs[name] = state.get(name, p.default if p.default is not inspect._empty else None)
    return kwargs

def apply_rules(state, now=None, version=None):
    decisions = {}
    for out, rule in RULES.items():
        if rule is None:
            continue
        try:
            res = rule(**bind_kwargs_for(rule, state))
        except Exception as e:
            log.error("Rule %s failed: %s", out, e)
            continue
        if res is None:
            continue
        decisions[out] = int(bool(res))

    for k, v in decisions.items():
        cur = int(bool(state.get(k, 0)))
        if v != cur:
            hal_set(k, v)

def hal_listener():
    global last_activity, previous_inputs, state_hal_seq
    log.info("Starting hal listener...")
    sub.setsockopt(zmq.RCVTIMEO, 500)
    tick_every = 10.0
    next_tick = time.monotonic() + tick_every

    while not stop_event.is_set():
        any_changed = False
        try:
            parts = sub.recv_multipart()
        except zmq.Again:
            now = time.monotonic()
            if now >= next_tick:
                with state_lock:
                    snapshot = dict(state_hal)
                    state_hal_seq = state_hal_seq + 1 if 'state_hal_seq' in globals() else 1
                    version = state_hal_seq
                apply_rules(snapshot, now=now, version=version)
                next_tick = now + tick_every
            continue
        except Exception:
            log.exception("hal_listener: recv failed")
            time.sleep(0.05)
            continue

        frames = [parts]
        while True:
            try:
                frames.append(sub.recv_multipart(flags=zmq.NOBLOCK))
            except zmq.Again:
                break

        now = time.monotonic()
        with state_lock:
            for p in frames:
                try:
                    msg = json.loads(p[-1])
                    st = msg.get("state", msg)
                except Exception:
                    log.exception("hal_listener: bad json")
                    continue

                state_hal.update(st)

                # change detection
                for key in RULE_INPUTS:
                    prev = previous_inputs.get(key)
                    curr = state_hal.get(key)
                    if key in ENCODER_INPUTS:
                        try:
                            if abs(int(curr or 0) - int(prev or 0)) >= ENCODER_DELTA_THRESHOLD:
                                any_changed = True
                        except Exception:
                            pass
                    elif curr is not None and curr != prev:
                        any_changed = True

                previous_inputs = {k: state_hal.get(k) for k in RULE_INPUTS}
                if any(k in ACTIVITY_INPUTS and previous_inputs.get(k) is not None for k in RULE_INPUTS):
                    last_activity = now

            snapshot = dict(state_hal)
            state_hal_seq = state_hal_seq + 1 if 'state_hal_seq' in globals() else 1
            version = state_hal_seq

        if any_changed or now >= next_tick:
            apply_rules(snapshot, now=now, version=version)
            if now >= next_tick:
                next_tick = now + tick_every


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
        now = time.monotonic()
        idle_time = now - last_activity
        idle = (idle_time > IDLE_TIMEOUT) and not is_laser_active()
        desired = not (idle)

        if desired != last_display_state:
            log.debug(f"backlight updated: {desired}")
            set_backlight(desired)
            last_display_state = desired
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
    sub.setsockopt(zmq.SUBSCRIBE, b'')
    sub.setsockopt(zmq.RCVTIMEO, 500)
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
