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
USER_INPUTS = {"i_lid", "i_fp0", "i_fp1", "i_fp2", "i_fp3", "i_btn_estop", "i_btn_fire", "i_mask_encoder", "i_axis_x", "i_axis_z", "i_coarse", "i_fine"}
RELEVANT_FOR_RULES = set(USER_INPUTS) | {"i_ambient_humidity"}
ENCODER_INPUTS = {"i_mask_encoder"}  # adjust if you rename or track deltas elsewhere
ENCODER_DELTA_THRESHOLD = 1  # define what counts as significant
previous_inputs = {}

# backlight control
IDLE_TIMEOUT = 5 * 60  # seconds
last_activity = time.monotonic()
last_display_state = None
rh_on = 45.0  # turn ON at/above this RH
rh_off = 40.0 # turn OFF at/below this RH

def getb(s, k):  # boolean read with default 0
    return bool(s.get(k, 0))

def num_ok(x):
    return isinstance(x, (int, float)) and math.isfinite(x)

def clamp01(x):
    return 0.0 if x < 0.0 else 100.0 if x > 100.0 else x
def dewpoint_rule(i_ambient_humidity=None,
                  o_k6_dry_fan=0, o_k8_dry_heat=0,
                  rh_on=45.0, rh_off=40.0,
                  min_on=120.0, min_off=120.0,
                  stale_timeout=180.0,
                  pre=0.0, purge=0.0):
    """
    Returns either:
      - None (hold), or
      - {'o_k8_dry_heat': 0/1, 'o_k6_dry_fan': 0/1}
    Enforces: heat implies fan. Optional pre-run/purge if pre/purge > 0.
    """
    now = time.monotonic()
    st = dewpoint_rule.__dict__  # process-local state
    last_change = st.get("last_change", 0.0)
    last_seen   = st.get("last_seen", 0.0)
    fan_on_at   = st.get("fan_on_at", 0.0)
    purge_until = st.get("purge_until", 0.0)

    running_heat = bool(o_k8_dry_heat)
    running_fan  = bool(o_k6_dry_fan)

    # sample
    rh = None
    if isinstance(i_ambient_humidity, (int, float)) and math.isfinite(i_ambient_humidity):
        rh = max(0.0, min(100.0, float(i_ambient_humidity)))
        last_seen = now

    # stale-sensor failsafe (respect min_on)
    if now - last_seen > stale_timeout:
        if running_heat and (now - last_change) >= min_on:
            st["last_change"] = now
            st["last_seen"] = last_seen
            return {"o_k8_dry_heat": 0, "o_k6_dry_fan": 1 if purge > 0 else 0}
        st["last_seen"] = last_seen
        return None

    # decide heat via value + dwell
    new_heat = None
    if rh is not None:
        if running_heat and rh <= rh_off and (now - last_change) >= min_on:
            new_heat = 0
        elif (not running_heat) and rh >= rh_on and (now - last_change) >= min_off:
            new_heat = 1

    if new_heat is None:
        st["last_seen"] = last_seen
        # maintain purge if active
        if not running_heat and now < purge_until:
            return {"o_k8_dry_heat": 0, "o_k6_dry_fan": 1}
        return None

    # transitions
    if new_heat == 1:
        # ensure fan on; optionally enforce pre-run
        fan_next = 1
        heat_next = 1
        if pre > 0:
            if not running_fan:
                fan_on_at = now
            if now - fan_on_at < pre:
                heat_next = 0  # delay heat this cycle
        if heat_next == 1:
            last_change = now
    else:  # new_heat == 0
        heat_next = 0
        if purge > 0:
            fan_next = 1
            purge_until = now + purge
        else:
            fan_next = int(running_fan)  # don’t force fan off here

        last_change = now

    # persist locals
    st["last_change"] = last_change
    st["last_seen"]   = last_seen
    st["fan_on_at"]   = fan_on_at
    st["purge_until"] = purge_until

    # enforce invariant: heat ⇒ fan
    if heat_next == 1 and 'fan_next' not in locals():
        fan_next = 1

    # default fan_next if unchanged
    if 'fan_next' not in locals():
        fan_next = int(running_fan)

    return {"o_k8_dry_heat": heat_next, "o_k6_dry_fan": fan_next}

# Rules for outputs - this is the brain stem - only handles low level rules
RULES = {
    "o_k1_laser"    : lambda: i_btn_estop, i_btn_fire : not (i_btn_estop or i_btn_fire),
    "o_k2_hpa"      : lambda: i_btn_estop, i_m7 : not i_btn_estop and i_m7,
    "o_k3_fire"     : lambda: i_btn_fire : i_btn_fire,
    "o_k5_lpa"      : lambda: i_btn_estop, i_m8 : not i_btn_estop and i_m8,
    "o_k7_exhaust"  : lambda: i_btn_estop, i_m8 : not i_btn_estop and i_m8,
    "o_k4_light"    : lambda:  None,    # Handled in application
    "o_k6_dry_fan"  : dewpoint_rule,
    "o_k8_dry_heat" : None,             # Handled by dewpoint_rule
}

def start_heartbeat():
    while not stop_event.is_set():
        try:
            pub.send_string("heartbeat")
        except Exception:
            log.exception("heartbeat send failed")
        stop_event.wait(1.0)

def hal_set(key, value):
    with state_lock:
        state_hal[key] = int(bool(value))
        outputs = {k: v for k, v in state_hal.items() if k.startswith("o_")}
    msg = json.dumps({"set": outputs})
    try:
        req.send_string(msg)
        log.debug("Sent outputs to HAL: %s", msg)
        ack = req.recv_string()
        if ack == "ok":
            return True
        try:
            rep = json.loads(ack)
            if rep.get("status", "").lower() == "ok" or rep.get("ok") is True:
                return True
            log.error("HAL NACK: %s", rep)
        except Exception:
            log.error("Unexpected HAL ack: %r", ack)
    except Exception as e:
        log.error("Failed to send outputs to HAL: %s", e)
    return False


def bind_kwargs_for(rule, state):
    if rule is None:
        return {}
    sig = inspect.signature(rule)
    kwargs = {}
    for name, p in sig.parameters.items():
        # prefer value from state; otherwise use the rule's default if any
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
        if isinstance(res, dict):
            for k, v in res.items():
                if v is None:
                    continue
                decisions[k] = int(bool(v))
        else:
            decisions[out] = int(bool(res))

    # (optional) safety overrides / priorities here

    for k, v in decisions.items():
        cur = int(bool(state.get(k, 0)))
        if v != cur:
            hal_set(k, v)

def hal_listener():
    global last_activity, previous_inputs, state_hal_seq
    log.info("Starting hal listener...")
    sub.setsockopt(zmq.RCVTIMEO, 500)   # 0.5s recv timeout
    tick_every = 10.0
    next_tick = time.monotonic() + tick_every

    while not stop_event.is_set():
        any_changed = False
        try:
            parts = sub.recv_multipart()  # blocks up to RCVTIMEO
        except zmq.Again:
            # timeout -> periodic tick
            now = time.monotonic()
            if now >= next_tick:
                with state_lock:
                    snapshot = dict(state_hal)
                    state_hal_seq = (state_hal_seq + 1) if 'state_hal_seq' in globals() else 1
                    version = state_hal_seq
                apply_rules(snapshot, now=now, version=version)
                next_tick = now + tick_every
            continue
        except Exception:
            log.exception("hal_listener: recv failed")
            time.sleep(0.05)  # backoff only on error
            continue

        # process first message + drain any burst
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
                    log.exception("hal_listener: bad json payload")
                    continue

                state_hal.update(st)

                # relevant change detection (inputs + any i_* your rules read)
                for key in RELEVANT_FOR_RULES:
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

                previous_inputs = {k: state_hal.get(k) for k in RELEVANT_FOR_RULES}

            if any_changed:
                last_activity = now

            # snapshot & version after ingesting the whole burst
            snapshot = dict(state_hal)
            state_hal_seq = (state_hal_seq + 1) if 'state_hal_seq' in globals() else 1
            version = state_hal_seq

        # release lock before heavy work
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
