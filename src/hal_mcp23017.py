import busio
import board
import sys
from adafruit_mcp230xx.mcp23017 import MCP23017

class MCP23017Handler:
    def __init__(self, addresses):
        """
        Initialize the MCP23017 handler with the given I2C addresses.
        """
        self.i2c_bus = busio.I2C(board.SCL, board.SDA)
        self.devices = {}
        self.addresses = addresses
        self._initialize_devices()

    def _initialize_devices(self):
        """
        Detect and configure MCP23017 expanders if available.
        """
        for address in self.addresses:
            try:
                # Attempt to initialize the MCP23017 device
                mcp = MCP23017(self.i2c_bus, address=address)

                # Test device presence by reading a known register
                mcp.read_u8(0x00)  # Read from IODIRA register

                # Configure all pins as inputs with pull-ups enabled
                for pin in range(16):
                    mcp.get_pin(pin).switch_to_input(pull=True)

                # Add the device to the list of active devices
                self.devices[address] = mcp
                print(f"MCP23017 configured at address 0x{address:02X}.")
            except Exception as e:
                print(f"MCP23017 at 0x{address:02X} not present or failed to initialize: {e}", file=sys.stderr)

    def _validate_pin(self, pin):
        """
        Validate the pin number (0-15).
        """
        if not (0 <= pin < 16):
            raise ValueError(f"Invalid pin number {pin}. Must be between 0 and 15.")

    def _get_register(self, pin, is_output=False):
        """
        Get the appropriate register for the given pin.
        """
        if pin < 8:
            return 0x12 if not is_output else 0x14  # GPIOA or OLATA
        else:
            return 0x13 if not is_output else 0x15  # GPIOB or OLATB

    def read_pin(self, device_address, pin):
        """
        Read the state of a specific pin on an MCP23017.
        """
        try:
            self._validate_pin(pin)
            if device_address not in self.devices:
                raise ValueError(f"Device at address 0x{device_address:02X} not found.")

            register = self._get_register(pin)
            pin = pin % 8  # Adjust pin number for GPIOB if necessary
            data = self.devices[device_address].read_u8(register)
            return (data >> pin) & 1  # Extract the state of the specific pin
        except Exception as e:
            print(f"Error reading MCP23017 at 0x{device_address:02X}, pin {pin}: {e}", file=sys.stderr)
            return None

    def write_pin(self, device_address, pin, state):
        """
        Write to a specific pin on an MCP23017.
        """
        try:
            self._validate_pin(pin)
            if device_address not in self.devices:
                raise ValueError(f"Device at address 0x{device_address:02X} not found.")

            register = self._get_register(pin, is_output=True)
            pin = pin % 8  # Adjust pin number for GPIOB if necessary

            # Read current register value
            current_value = self.devices[device_address].read_u8(register)

            # Update the specific pin state
            if state:
                new_value = current_value | (1 << pin)  # Set pin HIGH
            else:
                new_value = current_value & ~(1 << pin)  # Set pin LOW

            # Write updated value back to the register
            self.devices[device_address].write_u8(register, new_value)
            return True
        except Exception as e:
            print(f"Error writing MCP23017 at 0x{device_address:02X}, pin {pin}: {e}", file=sys.stderr)
            return False
