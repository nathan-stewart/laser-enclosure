# RPi GPIO pins
import RPi.GPIO as GPIO

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

ADS1115_PINS = {
    "i_air" : (0x48, 0),  # Example pin on ASD1115 1
    "i_co2" : (0x49, 1),  # Example pin on ASD1115 2
}