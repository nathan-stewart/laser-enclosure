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

# Track last known state
state = {
    "input": {},
    "output": {},
    "adc": {},
    "sensor": {}
}
lock = threading.Lock()

def get_initial_state():
    """Request the initial state from the HAL."""
    try:
        req.send_json({"cmd": "get_state"})
        response = req.recv_json()
        with lock:
            state.update(response)
    except Exception as e:
        print("Error retrieving initial state:", e)

def listen():
    """Listen for live updates."""
    while True:
        try:
            msg = sub.recv_json()
            with lock:
                topic = msg.get("topic", "")
                if topic.startswith("input/"):
                    pin = topic.split("/")[1]
                    state["input"][pin] = msg["state"]
                elif topic.startswith("adc/"):
                    pin = topic.split("/")[1]
                    state["adc"][pin] = msg["value"]
                elif topic == "sensor/dht11":
                    state["sensor"]["temperature"] = msg.get("temperature")
                    state["sensor"]["humidity"] = msg.get("humidity")
        except Exception as e:
            print("Error receiving:", e)

def draw_loop():
    """Continuously draw the live status."""
    while True:
        os.system("clear")
        with lock:
            print("==== HAL Live Status ====\n")
            print("Inputs:")
            for pin, pin_state in state["input"].items():
                print(f"  {pin}: {pin_state}")
            print("\nADC:")
            for pin, value in state["adc"].items():
                print(f"  {pin}: {value}")
            print("\nSensor:")
            print(f"  Temperature: {state['sensor'].get('temperature')}")
            print(f"  Humidity: {state['sensor'].get('humidity')}")
        time.sleep(1)

if __name__ == "__main__":
    get_initial_state()  # Retrieve initial state on startup
    threading.Thread(target=listen, daemon=True).start()
    draw_loop()
