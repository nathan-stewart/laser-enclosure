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

BME280_ADDRESS = 0x76
SEESAW_ADDRESS = 0x36
