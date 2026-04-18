#
#Simple HX711 load cell test — Raspberry Pi 5
#Wiring:
#    HX711 DT   → GPIO 5  (pin 29)
#    HX711 SCK  → GPIO 6  (pin 31)
#    HX711 VCC  → 3.3V or 5V
#    HX711 GND  → GND
#
#Install: pip install RPi.GPIO
#

import RPi.GPIO as GPIO
import time

#CONFIG
PIN_DT  = 5    # Data out
PIN_SCK = 6    # Clock
GAIN_PULSES = 25   # 25 = Channel A, gain 128 (default)
                   # 26 = Channel B, gain 32
                   # 27 = Channel A, gain 64
#

GPIO.setmode(GPIO.BCM)
GPIO.setup(PIN_DT,  GPIO.IN)
GPIO.setup(PIN_SCK, GPIO.OUT, initial=GPIO.LOW)


def read_raw() -> int:
    """Read one 24-bit sample from HX711. Returns signed integer."""
    # Wait for DOUT to go LOW (conversion ready, up to ~100 ms at 10 Hz)
    timeout = time.time() + 1.0
    while GPIO.input(PIN_DT) == GPIO.HIGH:
        if time.time() > timeout:
            raise TimeoutError("HX711 not responding — check wiring")

    # Clock in 24 data bits (MSB first)
    raw = 0
    for _ in range(24):
        GPIO.output(PIN_SCK, GPIO.HIGH)
        raw = (raw << 1) | GPIO.input(PIN_DT)
        GPIO.output(PIN_SCK, GPIO.LOW)

    # Extra pulses set gain for next reading
    for _ in range(GAIN_PULSES - 24):
        GPIO.output(PIN_SCK, GPIO.HIGH)
        GPIO.output(PIN_SCK, GPIO.LOW)

    # Convert to signed 24-bit (two's complement)
    if raw & 0x800000:
        raw -= 0x1000000
    return raw


def tare(samples: int = 10) -> float:
    """Average several readings to get the zero offset."""
    total = sum(read_raw() for _ in range(samples))
    return total / samples


#Run test
try:
    print("HX711 load cell test — reading raw values")
    print("Taring (keep load cell unloaded)...")
    offset = tare()
    print(f"  Offset = {offset:.0f}")
    print("\nReading... (Ctrl+C to stop)\n")

    while True:
        raw   = read_raw()
        value = raw - offset
        print(f"  raw={raw:10.0f}   tared={value:10.0f}")
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    GPIO.cleanup()
