#!/usr/bin/env python3
import logging
import math
import zmq, json, time, threading
import pinmap
import threading
'''
    "o_k1_laser": 22, # laser
    "o_k2_hpa": 23, # high pressure air assist
    "o_k3_fire": 24, # CO2 Extinguisher
    "o_k4_lpa": 5,  # Low Pressure Air Assist
    "o_k5_exhaust": 13, # Exhaust Fan
    "o_k6_light": 25, # Lights
    "o_k7_dry_fan": 6,  # Dehumidifier Fan
    "o_k8_dry_heat": 12, # Dehumidifier Heat

'''
state_lock = threading.Lock()
stop_event = threading.Event()
state_hal = {}
log = None

RULES = {
    "o_k1_laser"  : lambda i_btn_estop, i_btn_fire : not (i_btn_estop or i_btn_fire), # laser is blocked by controlled by estop and fire button
    "o_k2_hpa"    : lambda i_btn_estop, i_m7 : not i_btn_estop and i_m7, # high pressure air assist is controlled by estop and M7
    "o_k3_fire"   : lambda i_btn_fire : i_btn_fire, # CO2 extinguisher is controlled by fire button
    "o_k4_lpa"    : lambda i_btn_estop, i_m8 : not i_btn_estop and i_m8,
    "o_k5_exhaust": lambda i_btn_estop, i_m8 : not i_btn_estop and i_m8,
    # "o_k6_light":    #lights are software button controlled at application layer
    # "o_k7_dry_fan":  # dehumidifier fan is controlled by dewpoint check
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
        temperature = state_hal["i_temp"]
        humidity = state_hal["i_humidity"]
        dewpoint = dew_point_c(temperature, humidity)
        delta = dewpoint - temperature
        with state_lock:
            if dewpoint_hyst_on < delta < dewpoint_hyst_off:
                state_hal["o_k6_dry_fan"] = 1
                state_hal["o_k7_dry_heat"] = 1
            else:
                state_hal["o_k6_dry_fan"] = 0
                state_hal["o_k7_dry_heat"] = 0
        log.debug("Dewpoint check: Temp: %.2f, Humidity: %.2f, Dewpoint: %.2f, Delta: %.2f",
                  temperature, humidity, dewpoint, delta)
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
    updated = False
    log.debug("applying rules: %s", raw)
    with state_lock:
        inputs = state_hal.copy()
        for output, rule in RULES.items():
            try:
                args = {k: inputs[k] for k in rule.__code__.co_varnames}
                result = int(bool(rule(**args)))
                if state_hal.get(output) != result:
                    state_hal[output] = result
                    updated = True
            except KeyError as e:
                log.error(f"Missing input for {output}: {e}")
    if updated:
        publish_to_hal()


def hal_listener():
    while not stop_event.is_set():
        topic, raw = sub.recv_multipart()
        if topic == b'hal':
            log.debug("Received HAL update: %s", raw)
            with state_lock:
                state_hal.update(json.loads(raw).get("state", {}))
            apply_rules()  # Apply output logic based on updated inputs
            print("HAL State Updated:", state_hal)


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




