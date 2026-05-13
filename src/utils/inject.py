#!/usr/bin/env python3
import sys, zmq, json

ENDPOINT = "tcp://127.0.0.1:5557"

def send(obj):
    ctx = zmq.Context.instance()
    s = ctx.socket(zmq.REQ)
    s.setsockopt(zmq.RCVTIMEO, 2000)
    s.setsockopt(zmq.SNDTIMEO, 2000)
    s.connect(ENDPOINT)
    s.send_json(obj)
    print(s.recv_json())

if __name__ == "__main__":
    if len(sys.argv) >= 2 and sys.argv[1] == "set":
        if len(sys.argv) != 4:
            print("usage: inject.py set PIN 0|1"); sys.exit(2)
        pin, state = sys.argv[2], int(sys.argv[3])
        send({"cmd":"set","pin":pin,"state":1 if state else 0})
    elif len(sys.argv) == 3:
        key, val = sys.argv[1], sys.argv[2]
        try:
            val = float(val)
        except ValueError:
            pass
        send({"cmd":"inject","state":{key: val}})
    else:
        print("usage:\n  inject.py KEY VALUE        # inject state (e.g. i_ambient_humidity 56.5)\n  inject.py set PIN 0|1      # drive output via HAL")
        sys.exit(2)
