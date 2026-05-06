# PCA9685 over hardware I2C
# Pins: SDA = GPIO 2 (pin 3), SCL = GPIO 3 (pin 5)
# Enable with: sudo raspi-config → Interface Options → I2C → Enable

import smbus2
import time

I2C_BUS  = 1
PCA_ADDR = 0x40

CH_MIDJE   = 0
CH_SKULDER = 1
CH_ALBUE   = 2
CH_WRIST   = 3
CH_PUMP    = 4

PIN_PUMP_FWD = 21   # IN1 on L298N — HIGH = forward direction

CENTER_US    = 1500
US_PER_DEG   = 1000 / 90
SERVO_MIN_US = 800
SERVO_MAX_US = 2200
MIDJE_MIN_US = 500
MIDJE_MAX_US = 2500
PUMP_MIN_US  = 500
PUMP_MAX_US  = 2500

bus = smbus2.SMBus(I2C_BUS)
bus.write_byte_data(PCA_ADDR, 0x00, 0x10)   # sleep
bus.write_byte_data(PCA_ADDR, 0xFE, 0x79)   # ~50 Hz
bus.write_byte_data(PCA_ADDR, 0x00, 0x00)   # wake
time.sleep(0.01)

def set_pwm(channel, pulse_us):
    ticks  = round(pulse_us / 20000 * 4096)
    off_at = channel * 256           # stagger start of LOW pulse
    on_at  = (off_at + ticks) % 4096 # end of LOW pulse
    reg    = 0x06 + channel * 4
    bus.write_byte_data(PCA_ADDR, reg + 0, on_at  & 0xFF)
    bus.write_byte_data(PCA_ADDR, reg + 1, (on_at  >> 8) & 0x0F)
    bus.write_byte_data(PCA_ADDR, reg + 2, off_at & 0xFF)
    bus.write_byte_data(PCA_ADDR, reg + 3, (off_at >> 8) & 0x0F)

def angle_to_us(deg):
    return max(SERVO_MIN_US, min(SERVO_MAX_US, int(CENTER_US + deg * US_PER_DEG)))

def midje_to_us(deg):
    return max(MIDJE_MIN_US, min(MIDJE_MAX_US, int(CENTER_US + deg * US_PER_DEG)))

def pump_to_us(pct):
    return int(PUMP_MIN_US + pct * (PUMP_MAX_US - PUMP_MIN_US) / 100)

def set_duty(channel, pct):
    reg = 0x06 + channel * 4
    if pct <= 0:
        bus.write_i2c_block_data(PCA_ADDR, reg, [0x00, 0x00, 0x00, 0x10])  # FULL_OFF
    elif pct >= 100:
        bus.write_i2c_block_data(PCA_ADDR, reg, [0x00, 0x10, 0x00, 0x00])  # FULL_ON
    else:
        ticks = round(pct / 100 * 4096)
        bus.write_i2c_block_data(PCA_ADDR, reg, [
            0x00, 0x00, ticks & 0xFF, (ticks >> 8) & 0x0F,
        ])

def drive(midje, skulder, albue, wrist, pump):
    set_pwm(CH_MIDJE,   midje_to_us(midje))
    set_pwm(CH_SKULDER, angle_to_us(skulder))
    set_pwm(CH_ALBUE,   angle_to_us(albue))
    set_pwm(CH_WRIST,   angle_to_us(wrist))
    set_duty(CH_PUMP, pump)


if __name__ == "__main__":
    print("Scanning I2C bus...")
    found = []
    for addr in range(0x03, 0x78):
        try:
            bus.read_byte(addr)
            found.append(hex(addr))
        except OSError:
            pass
    print("Devices found:", found if found else "Nothing")
    print("PCA9685 MODE1 =", hex(bus.read_byte_data(PCA_ADDR, 0x00)))

    print("Sweeping servo on channel 0...")
    for us in [600, 1500, 2000, 2400, 1500]:
        set_pwm(CH_MIDJE, us)
        time.sleep(0.8)

    bus.close()
    print("Done.")
