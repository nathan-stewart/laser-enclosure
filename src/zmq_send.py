import zmq
import json

context = zmq.Context()
req = context.socket(zmq.REQ)
req.connect("tcp://localhost:5557")

cmd = {
    "cmd": "set",
    "pin": "o_k1_laser",
    "state": True
}

print(f"Sending: {json.dumps(cmd)}")
req.send_json(cmd)
reply = req.recv_json()
print("Reply:", reply)
