#!/usr/bin/env python3
import sys
import logging
import argparse
import zmq
import copy
import time
import json
import os
import threading
from RPi import GPIO
import busio
import board
from adafruit_ads1x15.analog_in import AnalogIn
from adafruit_ads1x15.ads1115 import ADS1115
from adafruit_mcp230xx.mcp23017 import MCP23017

sys.path.insert(0, os.path.dirname(__file__))
import pinmap
from pinmap import *
from sdnotify import SystemdNotifier
from collections import deque

notifier = SystemdNotifier()
i2c_bus1 = busio.I2C(board.SCL, board.SDA)

wait_40Hz = 1.0 / 40.0
wait_60s = 60.0
wait_config = 10.0  # seconds
TIMEOUT_40Hz = 0.5  # seconds
TIMEOUT_60s = 180  # seconds
heartbeat_lock = threading.Lock()
stop_event = threading.Event()
gpio_configured = threading.Event()
state_lock = threading.Lock()
last_40Hz_poll = 0
last_60s_poll = 0
error_threshold = 5
DEBOUNCE_LEN = 3

pub = None
rep = None

missing_devices = []
for address in MCP23017_ADDRESSES + ADS1115_ADDRESS + BME280_ADDRESS + ENCODER_ADDRESS:
    missing_devices.append(address)
ads1115_device = None

debounce = {}           # name -> deque
last_stable_state = {}  # name -> last confirmed value
log = None
last_heartbeat = time.time()


def heartbeat_listener():
    global last_heartbeat
    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect("tcp://localhost:5558")
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    while True:
        try:
            msg = sub.recv_string()
            if msg == "heartbeat":
                last_heartbeat = time.time()
        except zmq.ZMQError:
            pass

def heartbeat_monitor():
    global last_heartbeat
    while True:
        if time.time() - last_heartbeat > 2:
            print("WARNING: No heartbeat from control process. Taking emergency action.")
            # e.g., shut off laser GPIO
            shutdown_laser()
        time.sleep(1)

def shutdown_laser():
    print("Laser power disabled due to control heartbeat loss.")
    for output in RPi_OUTPUT_PINS.keys():
        set_output(output, 0)

# State tracking
with state_lock:
    current_state = {}
    for pin_dict in (RPi_INPUT_PINS, RPi_OUTPUT_PINS, pinmap.MCP23017_PINS, pinmap.ADS1115_PINS):
        for pin_name in pin_dict:
            current_state[pin_name] = None
    current_state['i_airtemp'] = None
    current_state['i_humidity']    = None
    current_state['i_ambient_pressure'] = None
    current_state['error_count'] = {
        "ads1115": 0,
        "mcp23017": 0,
        "bme280": 0
    }
    previous_state = copy.deepcopy(current_state)

def configure_gpio():
    global current_state, debounce, last_stable_state
    """Configure Raspberry Pi GPIO pins."""
    GPIO.setmode(GPIO.BCM)
    try:
        for name in RPi_INPUT_PINS:
            GPIO.setup(RPi_INPUT_PINS[name], GPIO.IN, pull_up_down=GPIO.PUD_UP)
            log.debug(f"RPi GPIO input pin {name} configured on BCM pin {RPi_INPUT_PINS[name]}.")
        log.debug(f"{RPi_OUTPUT_PINS}")
        for name in RPi_OUTPUT_PINS:
            GPIO.setup(RPi_OUTPUT_PINS[name], GPIO.OUT, initial=GPIO.LOW)
            current_state[name] = GPIO.input(RPi_OUTPUT_PINS[name])
            log.debug(f"RPi GPIO output pin {name} configured on BCM pin {RPi_OUTPUT_PINS[name]}.")
        log.debug("RPi GPIOs configured.")

        # initialize debounce tracking
        for name in list(RPi_INPUT_PINS.keys()):
            debounce[name] = deque(maxlen=DEBOUNCE_LEN)
            last_stable_state[name] = None

        gpio_configured.set()  # Signal that GPIO configuration is complete
    except KeyError as e:
        log.error(f"Configuration error: RPi_GPIO_PINS dictionary is missing key {e}.")

def configure_ads1115():
    global ads1115_device
    try:
        if ADS1115_ADDRESS not in missing_devices:
            return
        ads = ADS1115(i2c_bus1, address=ADS1115_ADDRESS)
        ads1115_device = {
            "ads": ads,
            "channels": {
                name: AnalogIn(ads, getattr(ADS1115, f'P{chan}'))
                for name, chan in pinmap.ADS1115_CHANNELS.items()
            }
        }
        missing_devices.remove(ADS1115_ADDRESS)
        log.info(f"ADS1115 detected at address 0x{ADS1115_ADDRESS:02X}.")
        
    except Exception as e:
        log.warning(f"Failed to configure ADS1115 at 0x{ADS1115_ADDRESS:02X}: {e}")

def configure_mcp23017():
    """Detect and configure MCP23017 devices with pin directions from MCP23017_PINS."""
    if MCP23017_ADDRESSES not in missing_devices: # TBD is this okay - can we search for a list not in a list?
        return
    devices = i2c_bus1.scan()
    for address in MCP23017_ADDRESSES:
        if address in missing_devices and address in devices:
            try:
                mcp = MCP23017(i2c_bus1, address=address)
                mcp23017_devices[address] = mcp
                missing_devices.remove(address)
                log.info(f"MCP23017 configured at address 0x{address:02X}.")
            except Exception as e:
                log.error(f"Error configuring MCP23017 at address 0x{address:02X}: {e}")

            for name, (pin_owner, pin) in MCP23017_PINS.items():
                if pin_owner == address: 
                    try:
                        if name in MCP23017_OUTPUT_PINS:
                            mcp.setup(pin, mcp.OUT)
                            mcp.output(pin, 0)  # default LOW
                        else:
                            mcp.setup(pin, mcp.IN)
                            mcp.pullups |= (1 << pin)
                    except Exception as e:
                        log.error(f"Failed to configure MCP23017 pin {name} on address 0x{address:02X}: {e}")

                    if name not in debounce:
                        debounce[name] = deque(maxlen=DEBOUNCE_LEN)
                        last_stable_state[name] = None

def configure_encoder():
    if ENCODER_ADDRESS not in missing_devices:
        return
    devices = i2c_bus1.scan()
    if ENCODER_ADDRESS in devices:
        missing_devices.remove(ENCODER_ADDRESS)

    # TBD do any setup we need to put it in delta mode if any
    
def configure_bme280():
    if BME280_ADDRESS not in missing_devices:
        return
    devices = i2c_bus1.scan()
    if BME280_ADDRESS in devices:
        missing_devices.remove(BME280_ADDRESS)

def configure():
    global debounce, last_stable_state
    configure_gpio()

    while not stop_event.is_set():
        try:
            configure_mcp23017()                    
            configure_ads1115()
            configure_encoder()
            configure_bme280()

            if len(missing_devices) == 0:
                log.info("All devices configured successfully.")
                break

        except Exception as e:
            pass
        stop_event.wait(wait_config)

def read_gpio():
    global debounce
    for name, pin in RPi_INPUT_PINS.items():
        try:
            val = GPIO.input(pin)
            debounce[name].append(val)
        except Exception as e:
            raise RuntimeError(f"GPIO read error on pin {name}: {e}")

def read_expanders():
    global debounce, current_state
    for address, mcp in mcp23017_devices.items():
        if address in missing_devices:
            continue
        for name, (device_address, pin) in MCP23017_PINS.items():
            if device_address == address:
                try:
                    val = mcp.input(pin)
                    debounce[name].append(val)
                    if current_state['error_count']['mcp'] > 0:
                        log.info(f"MCP23017 read error recovery at address 0x{address:02X}")
                        current_state['error_count']['mcp'] = 0
                except Exception as e:
                    current_state['error_count']['mcp'] += 1
                    if current_state['error_count']['mcp'] == error_threshold:
                        log.error(f"MCP23017 read error at address 0x{address:02X}: {e}")


def read_ads1115():
    global current_state
    if ADS1115_ADDRESS in missing_devices:
        return
    for name, channel in ads1115_device["channels"].items():
        try:
            current_state[name] = channel.value
            if current_state['error_count']['ads1115'] > 0:
                log.info(f"ADC read error recovered")
                current_state['error_count']['ads1115'] = 0
        except Exception as e:
            current_state['error_count']['ads1115'] += 1
            if current_state['error_count']['ads1115'] == error_threshold:
                log.error(f"ADC read error: {e}")

# TBD - figure out how to read the Adafruit encoder
def read_encoder_deltas():
    if ENCODER_ADDRESS in missing_devices:
        return
    i2c_bus1.readfrom_mem(ENCODER_ADDRESS, 0x88, 24)  # This is garbage

def read_bme280():
    import struct
    import time

    if BME280_ADDRESS in missing_devices:
        return

    # Read calibration data
    calib = i2c_bus1.readfrom_mem(BME280_ADDRESS, 0x88, 24)
    h_calib = i2c_bus1.readfrom_mem(BME280_ADDRESS, 0xA1, 1) + i2c_bus1.readfrom_mem(BME280_ADDRESS, 0xE1, 7)

    dig_T1 = struct.unpack('<H', calib[0:2])[0]
    dig_T2 = struct.unpack('<h', calib[2:4])[0]
    dig_T3 = struct.unpack('<h', calib[4:6])[0]

    dig_P1 = struct.unpack('<H', calib[6:8])[0]
    dig_P2 = struct.unpack('<h', calib[8:10])[0]
    dig_P3 = struct.unpack('<h', calib[10:12])[0]
    dig_P4 = struct.unpack('<h', calib[12:14])[0]
    dig_P5 = struct.unpack('<h', calib[14:16])[0]
    dig_P6 = struct.unpack('<h', calib[16:18])[0]
    dig_P7 = struct.unpack('<h', calib[18:20])[0]
    dig_P8 = struct.unpack('<h', calib[20:22])[0]
    dig_P9 = struct.unpack('<h', calib[22:24])[0]

    dig_H1 = h_calib[0]
    dig_H2 = struct.unpack('<h', h_calib[1:3])[0]
    dig_H3 = h_calib[3]
    e4, e5, e6 = h_calib[4:7]
    dig_H4 = (e4 << 4) | (e5 & 0x0F)
    dig_H5 = (e6 << 4) | (e5 >> 4)
    dig_H6 = struct.unpack('b', h_calib[7:8])[0]

    # Trigger measurement
    i2c_bus1.writeto_mem(BME280_ADDRESS, 0xF2, b'\x01')  # ctrl_hum: 1x
    i2c_bus1.writeto_mem(BME280_ADDRESS, 0xF4, b'\x27')  # ctrl_meas: temp/press 1x
    time.sleep_ms(100)

    # Read raw data
    raw = i2c_bus1.readfrom_mem(BME280_ADDRESS, 0xF7, 8)
    adc_P = ((raw[0] << 16) | (raw[1] << 8) | raw[2]) >> 4
    adc_T = ((raw[3] << 16) | (raw[4] << 8) | raw[5]) >> 4
    adc_H = (raw[6] << 8) | raw[7]

    # Temperature compensation
    var1 = (((adc_T >> 3) - (dig_T1 << 1)) * dig_T2) >> 11
    var2 = (((((adc_T >> 4) - dig_T1) * ((adc_T >> 4) - dig_T1)) >> 12) * dig_T3) >> 14
    t_fine = var1 + var2
    T = (t_fine * 5 + 128) >> 8

    # Pressure compensation
    var1 = t_fine - 128000
    var2 = var1 * var1 * dig_P6
    var2 += (var1 * dig_P5) << 17
    var2 += dig_P4 << 35
    var1 = ((var1 * var1 * dig_P3) >> 8) + ((var1 * dig_P2) << 12)
    var1 = (((1 << 47) + var1) * dig_P1) >> 33
    if var1 == 0:
        P = 0
    else:
        p = 1048576 - adc_P
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (dig_P9 * (p >> 13) * (p >> 13)) >> 25
        var2 = (dig_P8 * p) >> 19
        P = ((p + var1 + var2) >> 8) + (dig_P7 << 4)

    # Humidity compensation
    h = t_fine - 76800
    h = (((((adc_H << 14) - (dig_H4 << 20) - (dig_H5 * h)) + 16384) >> 15) *
         (((((((h * dig_H6) >> 10) * (((h * dig_H3) >> 11) + 32768)) >> 10) + 2097152) * dig_H2 + 8192) >> 14))
    h -= (((((h >> 15) * (h >> 15)) >> 7) * dig_H1) >> 4)
    h = max(0, min(h, 419430400))
    H = h >> 12

    return T / 100.0, P / 25600.0, H / 1024.0  # °C, hPa, %RH


def read_environment():
    global current_state
    if bme280_device:
        try:
            temp, pressure, humidity = read_bme280()
            current_state['i_airtemp'] = temp
            current_state['i_humidity'] = humidity
            current_state['i_ambient_pressure'] = pressure
            current_state['error_count']['bme280'] = 0
            if current_state['error_count']['bme280'] > 0:
                log.info("Recovered from BME280 error.")
                current_state['error_count']['bme280'] = 0
        except Exception as e:
            current_state['error_count']['bme280'] += 1
            if current_state['error_count']['bme280'] == error_threshold:
                log.warning(f"BME280 read error: {e}")
        except RuntimeError as e:
            current_state['error_count']['bme280'] += 1
            if current_state['error_count']['bme280'] == error_threshold:
                log.warning(f"BME280 read error: {e}")

def monitor_40Hz():
    global current_state, previous_state, last_40Hz_poll
    while not stop_event.is_set():
        with state_lock:
            with heartbeat_lock:
                last_40Hz_poll = time.time()

            read_gpio()
            read_expanders()
            read_ads1115()
            read_encoder_deltas()

            for pin, history in debounce.items():
                if len(history) == DEBOUNCE_LEN and all(v == history[0] for v in history):
                    if last_stable_state[pin] != history[0]:
                        last_stable_state[pin] = history[0]
                        current_state[pin] = history[0]

            # if previous_state != current_state:
            publish_state()
            previous_state = copy.deepcopy(current_state)

        stop_event.wait(wait_40Hz)


def monitor_60s():
    """Monitor environment and publish state changes."""
    global last_60s_poll, current_state, pub
    while not stop_event.is_set():
        with state_lock:
            with heartbeat_lock:
                last_60s_poll = time.time()
                if time.time() - last_60s_poll > TIMEOUT_60s:
                    raise RuntimeError("Input read timeout")
            read_environment()
            # publish_state()
            log.debug(f"Current state: {json.dumps(current_state, indent=2)}")
        stop_event.wait(wait_60s)

def publish_state():
    global current_state
    pub.send_multipart([b'hal', json.dumps({"state": current_state}).encode()])

def set_output(pin, value):
    """Set the state of an output pin."""
    global current_state, mcp23017_devices, rep
    try:
        with state_lock:
            if pin in RPi_OUTPUT_PINS:
                GPIO.output(RPi_OUTPUT_PINS[pin], GPIO.HIGH if value else GPIO.LOW)
                current_state[pin] = GPIO.input(RPi_OUTPUT_PINS[pin])
                if current_state[pin] != (1 if value else 0):
                    raise RuntimeError(f"Failed to set RPi output pin {pin} to {value}")
            elif pin in MCP23017_PINS:
                addr, mcp_pin = MCP23017_PINS[pin]
                mcp = mcp23017_devices[addr]
                if mcp:
                    try:
                        mcp.setup(mcp_pin, mcp.OUT)
                        mcp.output(mcp_pin, 1 if value else 0)
                        current_state[pin] = 1 if value else 0
                    except Exception as e:
                        raise RuntimeError(f"Error setting MCP23017 pin {pin}: {e}")
                else:
                    raise RuntimeError(f"MCP23017 device at address 0x{addr:02X} not found")
            else:
                raise ValueError(f"Unknown pin: {pin}")

            msg = {
                "status": "ok",
                "pin": pin,
                "value": value
            }

    except Exception as e:
        msg = {
            "status": "error",
            "error": str(e),
            "pin": pin,
            "value": value
        }
    rep.send(msg)

def handle_commands():
    global current_state, mcp23017_devices, bme280_device, rep
    while not stop_event.is_set():
        try:
            msg = rep.recv_json()
            cmd = msg.get("cmd")
            if cmd == "set":
                set_output(msg["pin"], msg["state"])
            else:
                rep.send_json({"status": "unsupported command"})
        except Exception as e:
            try:
                rep.send_json({"status": "error", "error": str(e)})
            except Exception:
                log.error(f"Error handling ZMQ command and replying: {e}")

def thread_wrapper(func):
    try:
        log.info(f"Thread {func.__name__} started")
        func()
    except Exception as e:
        log.exception(f"Thread {func.__name__} crashed: {e}")
        stop_event.set()


def main(argv=None):
    global pub, rep, ctx, log
    if argv is None:
        argv = sys.argv[1:]  # exclude script name
    parser = argparse.ArgumentParser(description="HAL Watcher")
    parser.add_argument(
        "--log", "-l",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR", "CRITICAL"],
        help="Set the logging level (default: INFO)"
    )
    args = parser.parse_args()

    logging.basicConfig(
        level=getattr(logging, args.log),
        format="%(asctime)s [%(levelname)s] %(message)s",
        handlers=[logging.StreamHandler(sys.stdout)]
    )
    log = logging.getLogger("HAL_Watcher")

    log.info("Starting HAL ...")
    # ZMQ setup
    ctx = zmq.Context()
    pub = ctx.socket(zmq.PUB)
    pub.bind("tcp://*:5556")
    log.info("ZeroMQ publisher bound to tcp://*:5556")

    rep = ctx.socket(zmq.REP)
    rep.bind("tcp://*:5557")
    log.info("ZeroMQ replier bound to tcp://*:5557")

    threads = []
    threads.append(threading.Thread(target=thread_wrapper, args=(configure,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, args=(handle_commands,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, args=(monitor_40Hz,), daemon=True))
    threads.append(threading.Thread(target=thread_wrapper, args=(monitor_60s,), daemon=True))

    threads[0].start()
    gpio_configured.wait()  # Wait for GPIO configuration to complete
    for thread in threads[1:]:
        thread.start()
    log.info("HAL threads started.")

    try:
        while not stop_event.is_set():
            # these are only monitoring the threads, not the devices
            with heartbeat_lock:
                if time.time() - last_40Hz_poll > TIMEOUT_40Hz:
                    raise RuntimeError("Heartbeat timeout: No input read in the last 0.5 seconds")
                if time.time() - last_60s_poll > TIMEOUT_60s:
                    raise RuntimeError("Heartbeat timeout: No environment read in the last 60 seconds")
                notifier.notify("WATCHDOG=1")
            stop_event.wait(TIMEOUT_40Hz)
        log.info("HAL shutting down")

    except KeyboardInterrupt:
        log.info("Terminated by user")

    except Exception as e:
        log.exception("Unhandled exception in main loop")

    stop_event.set()  # Signal threads to stop
    time.sleep(0.1)  # Give threads a moment to exit
    GPIO.cleanup()
    for thread in threads:
        thread.join()

    rep.close()
    pub.close()
    ctx.term()
    log.info("HAL shutdown complete.")

if __name__ == "__main__":
    main(sys.argv)