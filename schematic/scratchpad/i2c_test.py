from machine import Pin, I2C
import time
import struct

i2c = I2C(0, scl=Pin(1), sda=Pin(0), freq=100_000)

print("Scanning I2C...")
devices = i2c.scan()
print("Found:", [hex(addr) for addr in devices])

def read_bme280_all(addr):
    import struct
    import time

    # Read calibration data
    calib = i2c.readfrom_mem(addr, 0x88, 24)
    h_calib = i2c.readfrom_mem(addr, 0xA1, 1) + i2c.readfrom_mem(addr, 0xE1, 7)

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
    i2c.writeto_mem(addr, 0xF2, b'\x01')  # ctrl_hum: 1x
    i2c.writeto_mem(addr, 0xF4, b'\x27')  # ctrl_meas: temp/press 1x
    time.sleep_ms(100)

    # Read raw data
    raw = i2c.readfrom_mem(addr, 0xF7, 8)
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



# --- ADS1115 (read one channel, no lib) ---
def read_ads1115(addr):
    try:
        # Write config: AIN0 single-ended, FSR = ±4.096V, continuous mode
        config = 0x4000 | 0x0200 | 0x0003  # OS=1, MUX=100, PGA=±4.096V, DR=128SPS
        buf = struct.pack('>H', config)
        i2c.writeto_mem(addr, 0x01, buf)  # Config register
        time.sleep_ms(10)
        raw = i2c.readfrom_mem(addr, 0x00, 2)
        val = struct.unpack('>h', raw)[0]
        return val * 4.096 / 32768  # Convert to voltage
    except:
        return None

# --- Device check ---
bme_addr = next((a for a in devices if a in (0x76, 0x77)), None)
ads_addr = next((a for a in devices if 0x48 <= a <= 0x4B), None)

print("Devices:")
if bme_addr:
    print(" - BME280 at", hex(bme_addr))
else:
    print(" - BME280 not found")

if ads_addr:
    print(" - ADS1115 at", hex(ads_addr))
else:
    print(" - ADS1115 not found")

# --- Main loop ---
while True:
    if bme_addr:
        temp, hum = read_bme280_raw(bme_addr)
        print(f"BME280: {temp:.2f}°C, {hum:.2f}% RH")

    if ads_addr:
        val = read_ads1115(ads_addr)
        if val is not None:
            print("ADS1115 voltage:", round(val, 3), "V")

    time.sleep(2)
a