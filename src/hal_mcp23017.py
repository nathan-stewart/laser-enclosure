import busio
import board
import sys
from adafruit_mcp230xx.mcp23017 import MCP23017


# no need to lock i2c -  only referenced from the  40hz thread and config 
def configure_expanders():
    """Configure MCP23017 expanders if available."""
    global mcp23017_devices
    mcp23017_devices = {}
    for address in MCP23017_ADDRESSES:
        try:
            mcp = MCP23017(busio.I2C(board.SCL, board.SDA), address=address)
            # Read a known register to check if device is present
            if mcp.read_i2c_block_data(0x00, 0x00):
                mcp.read_i2c_block_data(0x00, 0x00)
                mcp23017_devices[address] = mcp
                print(f"MCP23017 configured at address 0x{address:02X}.")
                
                
        except Exception as e:
            print(f"Error configuring MCP23017: {e}", file=sys.stderr)

    return mcp23017_devices



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

def write_mcp23017_pin(device_address, pin, state):
    """Write to a specific pin on an MCP23017."""
    if i2c_bus and mcp23017_devices.get(device_address, False):
        try:
            # Determine which register to write (OLATA or OLATB)
            if pin < 8:
                register = 0x14  # OLATA
            elif pin < 16:
                register = 0x15  # OLATB
                pin -= 8  # Adjust pin number for GPIOB
            else:
                print(f"Invalid pin number {pin} for MCP23017.", file=sys.stderr)
                return False

            # Read current register value
            current_value = i2c_bus.read_byte_data(device_address, register)

            # Update the specific pin state
            if state:
                new_value = current_value | (1 << pin)  # Set pin HIGH
            else:
                new_value = current_value & ~(1 << pin)  # Set pin LOW

            # Write updated value back to the register
            i2c_bus.write_byte_data(device_address, register, new_value)
            return True
        except Exception as e:
            print(f"Error writing MCP23017 at 0x{device_address:02X}, pin {pin}: {e}", file=sys.stderr)
            return False
    return False  # Return False if bus not initialized or device not detected
