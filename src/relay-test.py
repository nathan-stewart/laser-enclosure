#!/usr/bin/env python3
import gpiod
import time
import termios
import tty
import select
import sys
from service.pinmap import RPi_OUTPUT_PINS

def getch_nonblocking():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

# Reserve all lines (relays) as outputs
chip = gpiod.Chip("gpiochip0")
lines = {}
for name, pin in RPi_OUTPUT_PINS.items():
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
    RPi_OUTPUT_KEYS = sorted([x for x in RPi_OUTPUT_PINS.keys()])
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
                        name = RPi_OUTPUT_KEYS[int(key) - 1]
                        pin = RPi_OUTPUT_PINS[name]
                        line = chip.get_line(pin)
                        val = 0 if line.get_value() else 1
                        line.set_value(val)
                        print(f"{name} : {val}")  # <-- Replace this with your GPIO toggle
                    else:
                        pressed.remove(key)  # <-- Reset pressed keys after short pause


    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

raw_input_loop()
