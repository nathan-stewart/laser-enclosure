#!/usr/bin/env python3
import math
import zmq, json, time, threading
import pinmap
import threading
'''
    "o_k1_laser": 22, # laser
    "o_k2_hpa": 23, # high pressure air assist
    "o_k3_fire": 24, # CO2 Extinguisher
    "o_k4_light": 25, # Lights
    "o_k5_lpa": 5,  # Low Pressure Air Assist
    "o_k6_dry_fan": 6,  # Dehumidifier Fan
    "o_k7_dry_heat": 12, # Dehumidifier Heat
    "o_k8_exhaust": 13, # Exhaust Fan
'''
state_lock = threading.Lock()
stop_event = threading.Event()
state_hal = {}

RULES = {
    "o_k1_laser = !(i_btn_estop || i_btn_fire)",
    "o_k2_hpa = !i_btn_estop && i_m7",
    "o_k3_fire = i_btn_fire",
    "o_k5_lpa =  !i_btn_estop && i_m8",
    "o_k8_exhaust = o_k1_laser && job_active"
}

# dewpoint hysteresis
dewpoint_hyst_on = 3.0 
dewpoint_hyst_off = 6.0
co2_enable = 15.0
air_enable = 15.0
dehumidifier_period = 30.0

def dew_point_c(temp_c, rh_percent):
    a = 17.62
    b = 243.12
    gamma = (a * temp_c) / (b + temp_c) + math.log(rh_percent / 100.0)
    dp = (b * gamma) / (a - gamma)
    return dp



def dewpoint_check():
    while not stop_event.is_set():
        temperature = state_hal["i_temp"]
        humidity = state_hal["i_humidity"]
        dewpoint = dew_point_c(temperature, humidity)
        delta = dewpoint - temperature
        with state_lock:
            global state_hal
            if dewpoint_hyst_on < delta < dewpoint_hyst_off:
                state_hal["o_k6_dry_fan"] = 1
                state_hal["o_k7_dry_heat"] = 1
            else:
                state_hal["o_k6_dry_fan"] = 0
                state_hal["o_k7_dry_heat"] = 0
        stop_event.wait(dehumidifier_period)


def hal_listener():
    while True:
        topic, raw = sub.recv_multipart()
        if topic == b'hal':
            with state_lock:
                state_hal.update(json.loads(raw).get("state", {}))

if __name__ == "__main__":
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

    while not stop_event.is_set():
        stop_event.wait(1.0)

    

