# PCA9685 over hardware I2C
# Pins: SDA = GPIO 2 (pin 3), SCL = GPIO 3 (pin 5)
# Enable with: sudo raspi-config -> Interface Options -> I2C -> Enable
#
# Speed tip: set I2C bus to 400 kHz in /boot/firmware/config.txt:
#   dtparam=i2c_arm=on,i2c_arm_baudrate=400000

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

bus = smbus2.SMBus(I2C_BUS)
bus.write_byte_data(PCA_ADDR, 0x00, 0x10)   # sleep
bus.write_byte_data(PCA_ADDR, 0xFE, 0x79)   # ~50 Hz
bus.write_byte_data(PCA_ADDR, 0x00, 0x20)   # wake + auto-increment (AI=1)
time.sleep(0.01)

# Inverted-pulse encoding: ON=ticks, OFF=0.
#   - count 0 hits LED_OFF -> output LOW
#   - count `ticks` hits LED_ON -> output HIGH
# Init writes ON=0, OFF=0 once. After this set_pwm only touches ON, so the
# OFF=0 invariant is preserved without sending those bytes every update.
_last_ticks = {}
for _ch in (CH_MIDJE, CH_SKULDER, CH_ALBUE, CH_WRIST):
    bus.write_i2c_block_data(PCA_ADDR, 0x06 + _ch * 4, [0, 0, 0, 0])
    _last_ticks[_ch] = 0

# ---------- helpers ----------

def angle_to_us(deg):
    return max(SERVO_MIN_US, min(SERVO_MAX_US, int(CENTER_US + deg * US_PER_DEG)))

def midje_to_us(deg):
    return max(MIDJE_MIN_US, min(MIDJE_MAX_US, int(CENTER_US + deg * US_PER_DEG)))

# ---------- servo write with glitch-minimized byte ordering ----------
#
# The PCA9685 doesn't double-buffer LED registers — every byte takes effect
# the instant it's written. With inverted polarity (ON=ticks variable), a
# new pulse width updates ON across two registers (ON_L, ON_H). Between
# those two byte writes the chip holds an intermediate ON value made of
# one new byte and one old byte. On a high-byte transition the intermediate
# can sit OUTSIDE the [old, new] range, producing a single bad cycle that
# shows up on a scope as "the LOW pulse suddenly got wider."
#
# Mitigation: choose the write order so the intermediate is the SMALLER of
# the two possibilities — the brief glitch becomes a narrower pulse instead
# of a wider one (much less visible). When only the low byte changes, one
# byte write is atomic — no glitch at all.

def set_pwm(channel, pulse_us):
    """Inverted servo write: LOW for `pulse_us`, HIGH for the rest of the cycle."""
    ticks = round(pulse_us / 20000 * 4096)
    new_h = (ticks >> 8) & 0x0F
    new_l = ticks & 0xFF
    reg   = 0x06 + channel * 4
    last  = _last_ticks.get(channel, 0)
    old_h = (last >> 8) & 0x0F
    old_l = last & 0xFF

    if new_h == old_h:
        # Same high byte — single atomic byte write (no glitch possible).
        if new_l != old_l:
            bus.write_byte_data(PCA_ADDR, reg, new_l)
    elif new_h < old_h:
        # Decreasing across high-byte boundary. Write high first:
        # intermediate = (old_l | new_h) < both old and new -> narrower glitch.
        bus.write_byte_data(PCA_ADDR, reg + 1, new_h)
        bus.write_byte_data(PCA_ADDR, reg,     new_l)
    else:
        # Increasing across high-byte boundary. Write low first:
        # intermediate = (new_l | old_h) < new -> narrower glitch.
        bus.write_byte_data(PCA_ADDR, reg,     new_l)
        bus.write_byte_data(PCA_ADDR, reg + 1, new_h)

    _last_ticks[channel] = ticks

def set_duty(channel, pct):
    """Pump duty (non-inverted): 0% = always LOW, 100% = always HIGH."""
    reg = 0x06 + channel * 4
    if pct <= 0:
        payload = [0x00, 0x00, 0x00, 0x10]   # FULL_OFF
    elif pct >= 100:
        payload = [0x00, 0x10, 0x00, 0x00]   # FULL_ON
    else:
        ticks = round(pct / 100 * 4096)
        payload = [0x00, 0x00, ticks & 0xFF, (ticks >> 8) & 0x0F]
    bus.write_i2c_block_data(PCA_ADDR, reg, payload)

# ---------- top-level drive with change detection ----------

_last_pct = None

def drive(midje, skulder, albue, wrist, pump):
    global _last_pct
    set_pwm(CH_MIDJE,   midje_to_us(midje))     # set_pwm carries its own
    set_pwm(CH_SKULDER, angle_to_us(skulder))   # change-detection cache, so
    set_pwm(CH_ALBUE,   angle_to_us(albue))     # unchanged channels do no I2C.
    set_pwm(CH_WRIST,   angle_to_us(wrist))
    if _last_pct != pump:
        set_duty(CH_PUMP, pump)
        _last_pct = pump

def invalidate_cache():
    _last_ticks.clear()
    for _ch in (CH_MIDJE, CH_SKULDER, CH_ALBUE, CH_WRIST):
        _last_ticks[_ch] = 0
    global _last_pct
    _last_pct = None


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
