#!/usr/bin/env python3
import sys, zmq, json

endpoint = "tcp://127.0.0.1:5557"  # HAL REP
key = sys.argv[1]                  # e.g. i_ambient_humidity
val = float(sys.argv[2])           # e.g. 56.5

ctx = zmq.Context.instance()
req = ctx.socket(zmq.REQ)
req.setsockopt(zmq.RCVTIMEO, 2000)
req.setsockopt(zmq.SNDTIMEO, 2000)
req.connect(endpoint)

req.send_json({"cmd": "inject", "state": {key: val}})
reply = req.recv_json()
print(reply)
