import zmq
import RPi.GPIO as GPIO
import time
import json
from threading import Thread
import sys

# Import libraries for DHT11 and MCP23017, handle potential absence
try:
    import Adafruit_DHT
except ImportError:
    Adafruit_DHT = None
    print("Adafruit_DHT not found. DHT11 readings will be skipped.", file=sys.stderr)

try:
    import smbus
except ImportError:
    smbus = None
    print("smbus not found. MCP23017 readings will be skipped.", file=sys.stderr)

# Define DHT11 sensor
DHT_TYPE = Adafruit_DHT.DHT11 if Adafruit_DHT else None

# Setup
GPIO.setmode(GPIO.BCM)

# Define RPi GPIO pins
RPi_INPUT_PINS = {
    "i_m7": 17,
    "i_m8": 18,
    "i_lid": 27,
    "i_dht11": 4, # DHT11 sensor data pin
}

RPi_OUTPUT_PINS = {
    "o_k1_laser": 22, # laser
    "o_k2_hpa": 23, # high pressure air assist
    "o_k3_fire": 24, # CO2 Extinguisher
    "o_k4_light": 25, # Lights
    "o_k5_lpa": 5,  # Low Pressure Air Assist
    "o_k6_dry_fan": 6,  # Dehumidifier Fan
    "o_k7_dry_heat": 12, # Dehumidifier Heat
    "o_k8_exhaust": 13, # Exhaust Fan
}

# Define MCP23017 expanders and their pins
# Mapping of logical names to (I2C device address, GPIO pin on the device)
MCP23017_PINS = {
    "i_fp1"       : (0x20, 0),
    "o_fp1"       : (0x20, 1),
    "i_fp2"       : (0x20, 2),
    "o_fp2"       : (0x20, 3),
    "i_fp3"       : (0x20, 4),
    "o_fp3"       : (0x20, 5),
    "i_fp4"       : (0x20, 6),
    "o_fp4"       : (0x20, 7),
    "i_btn_estop" : (0x20, 8),
    "i_btn_fire"  : (0x20, 9),
    "i_enc_a"     : (0x21, 0),
    "i_enc_b"     : (0x21, 1),
    "i_ignore_enc": (0x21, 2),
    "i_axis_x"    : (0x21, 3),
    "i_axis_z"    : (0x21, 4),
    "i_coarse"    : (0x21, 5),
    "i_fine"      : (0x21, 6),
}

# Identify unique MCP23017 addresses
MCP23017_ADDRESSES = list(set([addr for addr, pin in MCP23017_PINS.values()]))
I2C_BUS_NUM = 1 # Use I2C bus 1

try:
    # initial state -verify intention
    for pin in RPi_INPUT_PINS:
        GPIO.setup(RPi_INPUT_PINS[pin], GPIO.IN, pull_up_down=GPIO.PUD_UP)
    for pin in RPi_OUTPUT_PINS:
        GPIO.setup(RPi_OUTPUT_PINS[pin], GPIO.OUT, initial=GPIO.LOW) # Set outputs to LOW initially

    print("RPi GPIOs configured.")
except KeyError as e:
    print(f"Configuration error: RPi_GPIO_PINS dictionary is missing key {e}. Please update RPi_GPIO_PINS.", file=sys.stderr)
    # Decide how to handle this error: exit, continue with partial config, etc.
    # For now, we'll print and continue, but the missing pins won't be monitored.


# ZMQ setup
ctx = zmq.Context()

# PUB socket (status updates)
pub = ctx.socket(zmq.PUB)
pub.bind("tcp://*:5556")
print("ZeroMQ publisher bound to tcp://*:5556")

# REP socket (commands from GUI)
rep = ctx.socket(zmq.REP)
rep.bind("tcp://*:5557")
print("ZeroMQ replier bound to tcp://*:5557")

# State tracking
# Initialize RPi GPIO states
rpi_gpio_states = {}
for name, pin in RPi_GPIO_PINS.items():
     try:
         rpi_gpio_states[name] = GPIO.input(pin)
     except Exception as e:
         print(f"Could not read initial state for RPi GPIO {name} (pin {pin}): {e}", file=sys.stderr)
         rpi_gpio_states[name] = None # Indicate unknown state

# Initialize MCP23017 states (assuming all pins are inputs initially)
mcp23017_states = {name: 0 for name in MCP23017_PINS.keys()}

# Initialize DHT11 state
dht11_state = {'temperature': None, 'humidity': None}

# I2C Bus and MCP23017 initialization
i2c_bus = None
mcp23017_devices = {}

if smbus:
    try:
        i2c_bus = smbus.SMBus(I2C_BUS_NUM)
        print(f"I2C bus {I2C_BUS_NUM} initialized.")

        for address in MCP23017_ADDRESSES:
            try:
                # Try to read a known register to check if the device is present
                i2c_bus.read_byte_data(address, 0x00) # IODIRA register
                mcp23017_devices[address] = True
                print(f"MCP23017 detected at address 0x{address:02X}.")
                # Configure all pins as inputs with pull-ups (example configuration)
                try:
                    i2c_bus.write_byte_data(address, 0x00, 0xFF) # IODIRA all input
                    i2c_bus.write_byte_data(address, 0x01, 0xFF) # IODIRB all input
                    i2c_bus.write_byte_data(address, 0x0C, 0xFF) # GPPUA all pull-up
                    i2c_bus.write_byte_data(address, 0x0D, 0xFF) # GPPUB all pull-up
                    print(f"MCP23017 at 0x{address:02X} configured for input with pull-ups.")
                except Exception as e:
                     print(f"Error configuring MCP23017 at 0x{address:02X}: {e}", file=sys.stderr)

            except Exception as e:
                mcp23017_devices[address] = False
                print(f"No MCP23017 detected at address 0x{address:02X} or communication error: {e}", file=sys.stderr)

    except Exception as e:
        print(f"Error initializing I2C bus {I2C_BUS_NUM}: {e}", file=sys.stderr)
        i2c_bus = None # Ensure bus is None if initialization fails

# --- Reading Functions ---

def read_mcp23017_pin(device_address, pin):
    """Read the state of a specific pin on an MCP23017."""
    if i2c_bus and mcp23017_devices.get(device_address, False):
        try:
            # Determine which register to read (GPIOA or GPIOB)
            if pin < 8:
                register = 0x12 # GPIOA
            elif pin < 16:
                register = 0x13 # GPIOB
                pin -= 8 # Adjust pin number for GPIOB
            else:
                print(f"Invalid pin number {pin} for MCP23017.", file=sys.stderr)
                return None

            data = i2c_bus.read_byte_data(device_address, register)
            return (data >> pin) & 1  # Extract the state of the specific pin
        except Exception as e:
            print(f"Error reading MCP23017 at 0x{device_address:02X}, pin {pin}: {e}", file=sys.stderr)
            return None
    return None # Return None if bus not initialized or device not detected

def monitor_inputs():
    """Monitor all inputs (RPi GPIO, DHT11, MCP23017) and publish state changes."""
    global rpi_gpio_states, mcp23017_states, dht11_state

    while True:
        # Monitor RPi GPIOs
        for name, pin in RPi_GPIO_PINS.items():
            try:
                current_state = GPIO.input(pin)
                if current_state != rpi_gpio_states.get(name): # Use .get for safety
                    rpi_gpio_states[name] = current_state
                    msg = {
                        "topic": f"input/rpi_gpio/{name}",
                        "state": "active" if current_state == 0 else "inactive", # Assuming active low
                    }
                    pub.send_json(msg)
            except Exception as e:
                 # Handle cases where a pin might not have been configured correctly
                 if rpi_gpio_states.get(name) is not None: # Only report if it was previously working
                     print(f"Error reading RPi GPIO {name} (pin {pin}): {e}", file=sys.stderr)
                     rpi_gpio_states[name] = None # Mark as unknown state
                     msg = {
                         "topic": f"input/rpi_gpio/{name}",
                         "state": "error",
                         "error": str(e)
                     }
                     pub.send_json(msg)


        # Monitor MCP23017 pins
        if smbus and i2c_bus and mcp23017_devices: # Check if I2C is available and devices were detected
            for name, (device_address, pin) in MCP23017_PINS.items():
                 if mcp23017_devices.get(device_address, False): # Check if this specific device was detected
                    current_state = read_mcp23017_pin(device_address, pin)
                    if current_state is not None and current_state != mcp23017_states.get(name): # Use .get for safety
                        mcp23017_states[name] = current_state
                        msg = {
                            "topic": f"input/mcp23017/{name}",
                            "state": "active" if current_state == 0 else "inactive", # Assuming active low
                        }
                        pub.send_json(msg)
                 elif mcp23017_states.get(name) is not None: # Report error only if it was previously working
                      print(f"MCP23017 device at 0x{device_address:02X} not available for pin {name}.", file=sys.stderr)
                      mcp23017_states[name] = None # Mark as unknown state
                      msg = {
                          "topic": f"input/mcp23017/{name}",
                          "state": "error",
                          "error": f"Device 0x{device_address:02X} not available"
                      }
                      pub.send_json(msg)


        # Monitor DHT11
        if Adafruit_DHT and DHT_TYPE is not None: # Check if DHT library is available
            try:
                humidity, temperature = Adafruit_DHT.read_retry(DHT_TYPE, RPi_INPUT_PINS['i_dht11'])
                temp_changed = temperature is not None and temperature != dht11_state['temperature']
                hum_changed = humidity is not None and humidity != dht11_state['humidity']

                if temp_changed or hum_changed:
                    dht11_state['temperature'] = temperature
                    dht11_state['humidity'] = humidity
                    msg = {
                        "topic": "sensor/dht11",
                        "temperature": temperature,
                        "humidity": humidity,
                        "timestamp": time.time()
                    }
                    pub.send_json(msg)
                    if temperature is None or humidity is None:
                         print("Failed to get DHT11 reading.", file=sys.stderr)
                         # Optionally publish an error state
                         # msg = {"topic": "sensor/dht11", "state": "error", "message": "Failed to read"}
                         # pub.send_json(msg)

            except Exception as e:
                # Only report error if it was previously working
                if dht11_state['temperature'] is not None or dht11_state['humidity'] is not None:
                    print(f"Error reading DHT11: {e}", file=sys.stderr)
                    dht11_state = {'temperature': None, 'humidity': None} # Mark as unknown state
                    msg = {
                        "topic": "sensor/dht11",
                        "state": "error",
                        "error": str(e),
                        "timestamp": time.time()
                    }
                    pub.send_json(msg)
        elif dht11_state['temperature'] is not None or dht11_state['humidity'] is not None:
             # Report error if DHT was previously working but now library is gone or type is None
             print("DHT11 reading skipped (library not available or type not set).", file=sys.stderr)
             dht11_state = {'temperature': None, 'humidity': None} # Mark as unknown state
             msg = {
                 "topic": "sensor/dht11",
                 "state": "error",
                 "error": "DHT11 library not available",
                 "timestamp": time.time()
             }
             pub.send_json(msg)


        time.sleep(0.1) # Polling interval

def handle_commands():
    """Handle incoming commands via ZMQ REP socket."""
    while True:
        try:
            message = rep.recv_json()
            print(f"Received command: {message}")
            if message.get("command") == "relay":
                # Assuming "relay" command controls an RPi GPIO pin
                relay_pin_name = message.get("target") # e.g., "air_assist"
                action = message.get("action") # "on" or "off"

                # You need to map the target name (e.g., "air_assist") to an RPi GPIO pin name in RPi_GPIO_PINS
                # For example, if "air_assist" corresponds to the pin named "relay" in RPi_GPIO_PINS:
                target_rpi_pin_name = None # Find the corresponding RPi GPIO pin name
                # Example mapping (you need to define this based on your setup)
                if relay_pin_name == "air_assist":
                    # Assuming "air_assist" is controlled by the pin named "relay" in RPi_GPIO_PINS
                    target_rpi_pin_name = "relay" # This key needs to exist in RPi_GPIO_PINS

                if target_rpi_pin_name and target_rpi_pin_name in RPi_GPIO_PINS:
                    pin = RPi_GPIO_PINS[target_rpi_pin_name]
                    if action == "on":
                        GPIO.output(pin, GPIO.HIGH) # Assuming HIGH turns the relay on
                        rep.send_json({"status": "ok", "action": "set_high", "pin": target_rpi_pin_name})
                        print(f"Set RPi GPIO {target_rpi_pin_name} (pin {pin}) HIGH.")
                    elif action == "off":
                        GPIO.output(pin, GPIO.LOW) # Assuming LOW turns the relay off
                        rep.send_json({"status": "ok", "action": "set_low", "pin": target_rpi_pin_name})
                        print(f"Set RPi GPIO {target_rpi_pin_name} (pin {pin}) LOW.")
                    else:
                        rep.send_json({"status": "error", "message": "unknown action"})
                        print(f"Received unknown action for relay command: {action}", file=sys.stderr)
                else:
                    rep.send_json({"status": "error", "message": "unknown target"})
                    print(f"Received unknown target for relay command: {relay_pin_name}", file=sys.stderr)

            # Add handling for other commands here (e.g., controlling LEDs on MCP23017)
            # elif message.get("command") == "set_led":
            #     led_name = message.get("target")
            #     state = message.get("state") # "on" or "off"
            #     # Find the MCP23017 address and pin for the LED
            #     if led_name in MCP23017_PINS:
            #          address, pin = MCP23017_PINS[led_name]
            #          if i2c_bus and mcp23017_devices.get(address, False):
            #              try:
            #                  # Need to implement write logic for MCP23017
