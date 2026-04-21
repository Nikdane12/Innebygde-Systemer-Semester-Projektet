#Wiring:
#    HX711 DT   → GPIO 6
#    HX711 SCK  → GPIO 5
#    HX711 VCC  → 3.3V or 5V
#    HX711 GND  → GND

from gpiozero import InputDevice, OutputDevice
from collections import deque
import statistics
import time

#CONFIG
PIN_DT  = 6    # Data out
PIN_SCK = 5    # Clock
GAIN_PULSES = 25   # 25 = Channel A, gain 128 (default)
                   # 26 = Channel B, gain 32
                   # 27 = Channel A, gain 64

dt  = InputDevice(PIN_DT)
sck = OutputDevice(PIN_SCK, initial_value=False)

buffer = deque(maxlen=10)

def read_raw() -> int:
    #Read one 24-bit sample from HX711. Returns signed integer.
    #Wait for DOUT to go LOW (conversion ready, up to ~100 ms at 10 Hz)
    timeout = time.time() + 1.0
    while dt.value == 1:
        if time.time() > timeout:
            raise TimeoutError("HX711 not responding — check wiring")

    #Clock in 24 data bits (MSB first)
    raw = 0
    for _ in range(24):
        sck.on()
        raw = (raw << 1) | dt.value
        sck.off()

    #Extra pulses set gain for next reading
    for _ in range(GAIN_PULSES - 24):
        sck.on()
        sck.off()

    #Convert to signed 24-bit (two's complement)
    if raw & 0x800000:
        raw -= 0x1000000
    return raw


def read_stable() -> float:
    #Add latest reading to rolling buffer, return median of last 10.
    buffer.append(read_raw())
    return statistics.median(buffer)


def tare(samples: int = 10) -> float:
    #Average several readings to get the zero offset.
    total = sum(read_raw() for _ in range(samples))
    return total / samples


def calibrate(offset: float, samples: int = 10) -> float:
    known = float(input("Enter the weight of your calibration object in grams: "))
    input(f"Place {known}g on the scale and press Enter...")
    tared = sum(read_raw() - offset for _ in range(samples)) / samples
    scale_factor = tared / known
    print(f"  Scale factor = {scale_factor:.2f} raw/g")
    return scale_factor


#Run test
try:
    print("HX711 loadcell test")
    print("Taring (keep load cell unloaded)...")
    offset = tare()
    print(f"  Offset = {offset:.0f}")

    scale_factor = calibrate(offset)

    input("\nRemove calibration weight and press Enter to start reading...")
    print("\nReading... (Ctrl+C to stop)\n")

    while True:
        stable = read_stable()
        weight = (stable - offset) / scale_factor
        print(f"  {weight:8.1f} g")
        time.sleep(0.2)

except KeyboardInterrupt:
    print("\nStopped.")
finally:
    sck.close()
    dt.close()
