#!/usr/bin/env python3
import gpiod
import time
import termios
import tty
import select
import sys

RPi_OUTPUT_PINS = [
    ("o_k1_laser", 7),    # laser 24v power
    ("o_k2_hpa", 8),      # high pressure air assist
    ("o_k3_fire", 25),     # CO2 Extinguisher
    ("o_k4_light", 24),    # Lights
    ("o_k5_lpa", 23),       # Low Pressure Air Assist - 12V
    ("o_k6_dry_fan", 18),   # Dehumidifier Fan - 12V
    ("o_k7_exhaust", 12),  # Exhaust Fan - AC
    ("o_k8_dry_heat", 16), # Dehumidifier Heat - AC
]


def getch_nonblocking():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

# Reserve all lines (relays) as outputs
chip = gpiod.Chip("gpiochip0")
lines = {}
for name, pin in RPi_OUTPUT_PINS:
    try:
        line = chip.get_line(pin)
        line.request(consumer=name, type=gpiod.LINE_REQ_DIR_OUT)
        lines[name] = line
        print(f"Reserved GPIO{pin} for {name}")
        line.set_value(1)
        time.sleep(0.25)
        line.set_value(0)
    except Exception as e:
        print(f"Couldn't reserve GPIO{pin} for {name}: {e}")

def raw_input_loop():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    try:
        pressed = set()
        while True:
            ch = getch_nonblocking()

            if ch:
                if ch == '\x1b':  # ESC to quit
                    break
                if ch in '12345678':
                    key = int(ch)
                    if key not in pressed:
                        pressed.add(key)
                        name, pin = RPi_OUTPUT_PINS[int(key)-1]
                        line = chip.get_line(pin)
                        val = 0 if line.get_value() else 1
                        line.set_value(val)
                        print(f"{name} : {val}")  # <-- Replace this with your GPIO toggle
                    else:
                        pressed.remove(key)  # <-- Reset pressed keys after short pause


    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

raw_input_loop()
