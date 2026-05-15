#!/usr/bin/env python3
import sys
import zmq
import json

ENDPOINT = "tcp://127.0.0.1:5557"


def parse_value(s):
    if s in ("0", "1"):
        return int(s)
    try:
        return float(s)
    except ValueError:
        return s


def send(obj):
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.RCVTIMEO, 2000)
    sock.setsockopt(zmq.SNDTIMEO, 2000)
    sock.connect(ENDPOINT)

    sock.send_json(obj)
    return sock.recv_json()


def cmd_get(args):
    reply = send({"cmd": "get_status"})

    if reply.get("status") != "ok":
        print(json.dumps(reply, indent=2, sort_keys=True))
        return 1

    state = reply.get("state", {})

    if not args:
        print(json.dumps(state, indent=2, sort_keys=True))
        return 0

    missing = False
    out = {}

    for key in args:
        if key in state:
            out[key] = state[key]
        else:
            out[key] = None
            missing = True

    if len(args) == 1:
        print(out[args[0]])
    else:
        print(json.dumps(out, indent=2, sort_keys=True))

    return 1 if missing else 0


def cmd_set(args):
    if len(args) != 2:
        usage()
        return 2

    name, state = args
    val = parse_value(state)

    if name.startswith("v_"):
        reply = send({
            "cmd": "inject",
            "state": {name: val},
        })
    else:
        reply = send({
            "cmd": "set",
            "pin": name,
            "state": 1 if int(state) else 0,
        })

    print(json.dumps(reply, indent=2, sort_keys=True))
    return 0 if reply.get("status") == "ok" else 1

def cmd_inject(args):
    if len(args) != 2:
        usage()
        return 2

    key, val = args
    reply = send({
        "cmd": "inject",
        "state": {key: parse_value(val)},
    })

    print(json.dumps(reply, indent=2, sort_keys=True))
    return 0 if reply.get("status") == "ok" else 1


def usage():
    print("""usage:
  halctl.py get [KEY ...]
  halctl.py inject KEY VALUE
  halctl.py set PIN 0|1

examples:
  halctl.py get
  halctl.py get i_lid
  halctl.py inject i_ambient_humidity 56.5
  halctl.py set o_k7_exhaust 1
""")


def main(argv):
    if len(argv) < 2:
        usage()
        return 2

    cmd = argv[1]
    args = argv[2:]

    if cmd == "get":
        return cmd_get(args)
    if cmd == "set":
        return cmd_set(args)
    if cmd == "inject":
        return cmd_inject(args)

    usage()
    return 2


if __name__ == "__main__":
    sys.exit(main(sys.argv))