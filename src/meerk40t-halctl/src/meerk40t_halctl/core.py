import zmq


HAL_REP_ADDR = "tcp://127.0.0.1:5557"


class HalCtlError(RuntimeError):
    pass


def _send_hal_command(payload, timeout_ms=2000):
    ctx = zmq.Context.instance()
    sock = ctx.socket(zmq.REQ)
    sock.setsockopt(zmq.LINGER, 0)
    sock.setsockopt(zmq.RCVTIMEO, timeout_ms)
    sock.setsockopt(zmq.SNDTIMEO, timeout_ms)

    try:
        sock.connect(HAL_REP_ADDR)
        sock.send_json(payload)
        return sock.recv_json()
    except zmq.error.Again as e:
        raise HalCtlError("HAL command timed out") from e
    finally:
        sock.close()


def set_exhaust(value, *, source="cli"):
    if value in ("on", "1", 1, True):
        return _send_hal_command({
            "cmd": "exhaust",
            "mode": "on",
            "source": source,
        })

    if value in ("off", "0", 0, False):
        return _send_hal_command({
            "cmd": "exhaust",
            "mode": "off",
            "source": source,
        })

    try:
        seconds = int(value)
    except (TypeError, ValueError) as e:
        raise HalCtlError("exhaust value must be on, off, or integer seconds") from e

    if seconds < 0 or seconds > 3600:
        raise HalCtlError("exhaust seconds must be 0..3600")

    return _send_hal_command({
        "cmd": "exhaust",
        "mode": "timed",
        "duration":seconds,
        "source": source,
    })

def get_status(*, source="cli"):
    return _send_hal_command({
        "cmd": "get_status",
        "source": source,
    })


def set_output(pin, value, *, source="cli"):
    try:
        state = int(value)
    except (TypeError, ValueError) as e:
        raise HalCtlError("output state must be 0 or 1") from e

    if state not in (0, 1):
        raise HalCtlError("output state must be 0 or 1")

    return _send_hal_command({
        "cmd": "set",
        "pin": pin,
        "state": state,
        "source": source,
    })


def parse_value(value):
    if value in ("0", "1"):
        return int(value)

    try:
        return int(value)
    except (TypeError, ValueError):
        pass

    try:
        return float(value)
    except (TypeError, ValueError):
        pass

    return value


def inject_state(key, value, *, source="cli"):
    return _send_hal_command({
        "cmd": "inject",
        "state": {
            key: parse_value(value),
        },
        "source": source,
    })



def run_halctl(argv, *, source="cli"):
    if not argv:
        raise HalCtlError(
            "usage: halctl get|status | exhaust on|off|SECONDS | set PIN 0|1 | inject KEY VALUE"
        )

    cmd = argv[0]

    if cmd in ("get", "status"):
        if len(argv) != 1:
            raise HalCtlError("usage: halctl get")
        return get_status(source=source)

    if cmd == "exhaust":
        if len(argv) != 2:
            raise HalCtlError("usage: halctl exhaust on|off|SECONDS")
        return set_exhaust(argv[1], source=source)

    if cmd == "set":
        if len(argv) != 3:
            raise HalCtlError("usage: halctl set PIN 0|1")
        return set_output(argv[1], argv[2], source=source)

    if cmd == "inject":
        if len(argv) != 3:
            raise HalCtlError("usage: halctl inject KEY VALUE")
        return inject_state(argv[1], argv[2], source=source)

    raise HalCtlError(f"unknown command: {' '.join(argv)}")