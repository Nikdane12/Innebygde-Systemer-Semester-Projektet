# Pins: SDA = GPIO 4 (pin 7), SCL = GPIO 3 (pin 5)
# Requires in /boot/firmware/config.txt:
#   dtoverlay=i2c-gpio,bus=4,i2c_gpio_sda=4,i2c_gpio_scl=3
# Install: pip install smbus2

import smbus2
import time

I2C_BUS  = 4       # software I2C bus on GPIO 4 (SDA) and GPIO 3 (SCL)
PCA_ADDR = 0x40    # default PCA9685 address

bus = smbus2.SMBus(I2C_BUS)

# Scan
print("Scanning I2C bus...")
found = []
for addr in range(0x03, 0x78):
    try:
        bus.read_byte(addr)
        found.append(hex(addr))
    except OSError:
        pass
print("Devices found:", found if found else "Nothing")

# Sleep -> set prescaler -> wake up
bus.write_byte_data(PCA_ADDR, 0x00, 0x10)   # sleep (required before prescaler)
bus.write_byte_data(PCA_ADDR, 0xFE, 0x79)   # prescaler 0x79 = 121 ≈ 50 Hz
bus.write_byte_data(PCA_ADDR, 0x00, 0x00)   # wake up
time.sleep(0.01)
print("PCA9685 MODE1 =", hex(bus.read_byte_data(PCA_ADDR, 0x00)))  # should be 0x00

def set_servo(channel, pulse_us):
    ticks = round(pulse_us / 20000 * 4096)
    off_l = ticks & 0xFF
    off_h = (ticks >> 8) & 0x0F
    reg   = 0x06 + channel * 4
    bus.write_byte_data(PCA_ADDR, reg + 0, 0x00)   # ON_L
    bus.write_byte_data(PCA_ADDR, reg + 1, 0x00)   # ON_H
    bus.write_byte_data(PCA_ADDR, reg + 2, off_l)  # OFF_L
    bus.write_byte_data(PCA_ADDR, reg + 3, off_h)  # OFF_H

    # Verify write
    rb_l = bus.read_byte_data(PCA_ADDR, reg + 2)
    rb_h = bus.read_byte_data(PCA_ADDR, reg + 3)
    rb_ticks = rb_l | (rb_h << 8)
    print(f"  ch{channel}: {pulse_us} µs -> {ticks} ticks | readback: {rb_ticks} ticks")

print("Sweeping servo on channel 0...")
for us in [600, 1500, 2000, 2400, 1500]:
    set_servo(0, us)
    time.sleep(0.8)

bus.close()
print("Done.")
