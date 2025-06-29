# RPi GPIO pins
import board
import RPi.GPIO as GPIO

RPi_INPUT_PINS = {
    "i_m7": 22,
    "i_m8": 27,
    "i_lid": 17,
}

RPi_OUTPUT_PINS = {
    "o_k1_laser": 7,    # laser 24v power
    "o_k2_hpa": 8,      # high pressure air assist
    "o_k3_fire": 25,     # CO2 Extinguisher
    "o_k4_light": 24,    # Lights
    "o_k5_lpa": 23,       # Low Pressure Air Assist - 12V
    "o_k6_dry_fan": 18,   # Dehumidifier Fan - 12V
    "o_k7_exhaust": 12,  # Exhaust Fan - AC
    "o_k8_dry_heat": 16, # Dehumidifier Heat - AC
}

# MCP23017 expanders and their pins
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
    "i_ignore_enc": (0x21, 0),
    "i_axis_x"    : (0x21, 1),
    "i_axis_z"    : (0x21, 2),
    "i_coarse"    : (0x21, 3),
    "i_fine"      : (0x21, 4),
}

MCP23017_OUTPUT_PINS = [ x for x in MCP23017_PINS if x.startswith("o_") ]

ADS1115_PINS = {
    "i_air" : (0x48, 0),  # Example pin on ASD1115 1
    "i_co2" : (0x49, 1),  # Example pin on ASD1115 2
}

DHT11_PINS = {
    "i_dht11" : board.D4,  # DHT11 sensor pin
}
