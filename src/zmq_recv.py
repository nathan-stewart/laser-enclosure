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
sub.setsockopt_string(zmq.SUBSCRIBE, "")  # Subscribe to all messages

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
            msg = sub.recv_json()
            with lock:
                if msg.get("topic") == "state/update":
                    for k, v in msg.get("deltas", {}).items():
                        state[k] = v
                elif msg.get("topic") == "state/full":
                    state.update(msg.get("state", {}))
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
