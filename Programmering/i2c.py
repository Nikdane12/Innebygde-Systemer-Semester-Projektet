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

# ---------- low-level helpers ----------
#
# Inverted servo signal: output sits HIGH and pulses LOW for `pulse_us`
# at the start of each 20 ms cycle. Encoded with ON=ticks, OFF=0 so that:
#   - count 0 hits LED_OFF -> output goes LOW
#   - count `ticks` hits LED_ON -> output goes HIGH
# All four bytes are sent in one block write so the chip never observes a
# half-updated ON value (which on inverted polarity briefly pinned the line
# to 5 V whenever any servo moved).

def _servo_payload(pulse_us):
    """4-byte LED_ON/OFF payload for an inverted servo pulse."""
    ticks = round(pulse_us / 20000 * 4096)
    return [ticks & 0xFF, (ticks >> 8) & 0x0F, 0x00, 0x00]

def _duty_payload(pct):
    """4-byte payload for pump duty (non-inverted)."""
    if pct <= 0:
        return [0x00, 0x00, 0x00, 0x10]   # FULL_OFF -> always LOW
    elif pct >= 100:
        return [0x00, 0x10, 0x00, 0x00]   # FULL_ON  -> always HIGH
    else:
        ticks = round(pct / 100 * 4096)
        return [0x00, 0x00, ticks & 0xFF, (ticks >> 8) & 0x0F]

def angle_to_us(deg):
    return max(SERVO_MIN_US, min(SERVO_MAX_US, int(CENTER_US + deg * US_PER_DEG)))

def midje_to_us(deg):
    return max(MIDJE_MIN_US, min(MIDJE_MAX_US, int(CENTER_US + deg * US_PER_DEG)))

def set_pwm(channel, pulse_us):
    reg = 0x06 + channel * 4
    bus.write_i2c_block_data(PCA_ADDR, reg, _servo_payload(pulse_us))

def set_duty(channel, pct):
    reg = 0x06 + channel * 4
    bus.write_i2c_block_data(PCA_ADDR, reg, _duty_payload(pct))

# ---------- batched drive with change detection ----------
#
# Cache the last 4-byte payload sent to each channel. On the next drive():
#   1. Recompute each channel's payload.
#   2. Skip channels whose payload is unchanged — no I2C write, no servo
#      twitch, no current spike.
#   3. Group adjacent changed channels into a single block write so we
#      pay one I2C transaction instead of one per channel.
#
# The PCA9685's LED registers are contiguous (CH n starts at 0x06 + 4n),
# so a run of consecutive changed channels can be written in one shot.

_last_payload = {}   # channel -> list[4 bytes] last sent

def _flush_runs(payloads):
    if not payloads:
        return
    channels = sorted(payloads.keys())
    run_start = channels[0]
    run_bytes = list(payloads[run_start])
    prev = run_start

    for ch in channels[1:]:
        if ch == prev + 1:
            run_bytes.extend(payloads[ch])
            prev = ch
        else:
            bus.write_i2c_block_data(PCA_ADDR, 0x06 + run_start * 4, run_bytes)
            run_start = ch
            run_bytes = list(payloads[ch])
            prev = ch

    bus.write_i2c_block_data(PCA_ADDR, 0x06 + run_start * 4, run_bytes)

def drive(midje, skulder, albue, wrist, pump):
    new_payloads = {
        CH_MIDJE:   _servo_payload(midje_to_us(midje)),
        CH_SKULDER: _servo_payload(angle_to_us(skulder)),
        CH_ALBUE:   _servo_payload(angle_to_us(albue)),
        CH_WRIST:   _servo_payload(angle_to_us(wrist)),
        CH_PUMP:    _duty_payload(pump),
    }

    changed = {ch: p for ch, p in new_payloads.items()
               if _last_payload.get(ch) != p}

    _flush_runs(changed)

    for ch, p in changed.items():
        _last_payload[ch] = p

def invalidate_cache():
    _last_payload.clear()


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
