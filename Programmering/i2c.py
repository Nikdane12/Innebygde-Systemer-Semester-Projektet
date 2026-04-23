# PCA9685 over software I2C
# Pins: SDA = GPIO 4 (pin 7), SCL = GPIO 3 (pin 5)
# Requires in /boot/firmware/config.txt:
#   dtoverlay=i2c-gpio,bus=4,i2c_gpio_sda=4,i2c_gpio_scl=3

import smbus2
import time

I2C_BUS  = 4
PCA_ADDR = 0x40

CH_MIDJE   = 0
CH_SKULDER = 1
CH_ALBUE   = 2
CH_WRIST   = 3
CH_PUMP    = 4

PIN_PUMP_FWD = 19   # IN1 on L298N — HIGH = forward direction

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
    ticks = round(pulse_us / 20000 * 4096)
    reg   = 0x06 + channel * 4
    bus.write_byte_data(PCA_ADDR, reg + 0, 0x00)
    bus.write_byte_data(PCA_ADDR, reg + 1, 0x00)
    bus.write_byte_data(PCA_ADDR, reg + 2, ticks & 0xFF)
    bus.write_byte_data(PCA_ADDR, reg + 3, (ticks >> 8) & 0x0F)

def angle_to_us(deg):
    return max(SERVO_MIN_US, min(SERVO_MAX_US, int(CENTER_US + deg * US_PER_DEG)))

def midje_to_us(deg):
    return max(MIDJE_MIN_US, min(MIDJE_MAX_US, int(CENTER_US + deg * US_PER_DEG)))

def pump_to_us(pct):
    return int(PUMP_MIN_US + pct * (PUMP_MAX_US - PUMP_MIN_US) / 100)

def set_duty(channel, pct):
    #Set a channel to a raw duty cycle (0–100%) for motor control
    ticks = round(min(max(pct, 0), 100) / 100 * 4095)
    reg   = 0x06 + channel * 4
    bus.write_byte_data(PCA_ADDR, reg + 0, 0x00)
    bus.write_byte_data(PCA_ADDR, reg + 1, 0x00)
    bus.write_byte_data(PCA_ADDR, reg + 2, ticks & 0xFF)
    bus.write_byte_data(PCA_ADDR, reg + 3, (ticks >> 8) & 0x0F)

def drive(midje, skulder, albue, wrist, pump):
    set_pwm(CH_MIDJE,   midje_to_us(midje))
    set_pwm(CH_SKULDER, angle_to_us(skulder))
    set_pwm(CH_ALBUE,   angle_to_us(albue))
    set_pwm(CH_WRIST,   angle_to_us(wrist))
    set_duty(CH_PUMP,   pump)


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
