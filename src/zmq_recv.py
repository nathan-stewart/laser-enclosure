#!/usr/bin/env python3
import zmq
import json
import time
import threading
import os

context = zmq.Context()

# Subscriber socket for live updates
sub = context.socket(zmq.SUB)
sub.connect("tcp://localhost:5556")
sub.setsockopt(zmq.SUBSCRIBE, b'hal')

# Request socket for initial state
req = context.socket(zmq.REQ)
req.connect("tcp://localhost:5557")

# Flat dictionary of all states (inputs, outputs, sensors, etc.)
state = {}
lock = threading.Lock()
def listen():
    """Listen for live updates."""
    while True:
        try:
            topic, raw = sub.recv_multipart()
            if topic == b'hal':
                data = json.loads(raw)
                with lock:
                    state.update(data.get("state", {}))
        except Exception as e:
            print("Error receiving:", e)

def draw_loop():
    """Continuously draw the live status."""
    while True:
        os.system("clear")
        with lock:
            print("==== HAL Live Status ====\n")
            for pin in sorted(state.keys()):
                print(f"{pin:>16}: {state[pin]}")
        time.sleep(1)

if __name__ == "__main__":
    threading.Thread(target=listen, daemon=True).start()
    draw_loop()
