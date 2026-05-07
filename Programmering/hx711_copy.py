# HX711 load cell amplifier — multi-sensor
# Wiring: DT1 → GPIO 6, DT2 → GPIO 13, DT3 → GPIO 19
#         SCK (shared) → GPIO 5

from gpiozero import InputDevice, OutputDevice
from collections import deque
import statistics
import lgpio
import time

PIN_DT1     = 6
PIN_DT2     = 13
PIN_DT3     = 19
PIN_SCK     = 5
GAIN_PULSES = 25   # 25 = Channel A, gain 128 (default)
                   # 26 = Channel B, gain 32
                   # 27 = Channel A, gain 64

def _open_device(pin):
    try:
        return InputDevice(pin)
    except Exception:
        h = lgpio.gpiochip_open(0)
        lgpio.gpio_free(h, pin)
        lgpio.gpiochip_close(h)
        return InputDevice(pin)

def _open_output(pin):
    try:
        return OutputDevice(pin, initial_value=False)
    except Exception:
        h = lgpio.gpiochip_open(0)
        lgpio.gpio_free(h, pin)
        lgpio.gpiochip_close(h)
        return OutputDevice(pin, initial_value=False)

# Shared clock
sck = _open_output(PIN_SCK)

class HX711:
    def __init__(self, pin_dt):
        self.dt          = _open_device(pin_dt)
        self._buf        = deque(maxlen=10)
        self.hx_offset   = 0.0
        self.scale_factor = 1.0

    def read_raw(self) -> int:
        timeout = time.time() + 1.0
        while self.dt.value == 1:
            if time.time() > timeout:
                raise TimeoutError("HX711 not responding — check wiring")
        raw = 0
        for _ in range(24):
            sck.on()
            raw = (raw << 1) | self.dt.value
            sck.off()
        for _ in range(GAIN_PULSES - 24):
            sck.on()
            sck.off()
        if raw & 0x800000:
            raw -= 0x1000000
        return raw

    def read_stable(self) -> float:
        self._buf.append(self.read_raw())
        return statistics.median(self._buf)

    def read_median(self, samples: int = 10) -> float:
        #Takes N fresh readingsand returns their median
        readings = [self.read_raw() for _ in range(samples)]
        return statistics.median(readings)

    def tare(self, samples: int = 10) -> float:
        self.hx_offset = sum(self.read_raw() for _ in range(samples)) / samples
        return self.hx_offset

    def calibrate(self, known_grams: float, samples: int = 10) -> float:
        tared = sum(self.read_raw() - self.hx_offset for _ in range(samples)) / samples
        self.scale_factor = tared / known_grams
        return self.scale_factor

    def read_grams(self) -> float:
        return (self.read_stable() - self.hx_offset) / self.scale_factor

    def close(self):
        self.dt.close()


# Sensor instances
sensor1 = HX711(PIN_DT1)
sensor2 = HX711(PIN_DT2)
sensor3 = HX711(PIN_DT3)


if __name__ == "__main__":
    try:
        print("HX711 test — sensor 1")
        sensor1.tare()
        print(f"  Offset = {sensor1.hx_offset:.0f}")
        known = float(input("Weight of calibration object in grams: "))
        input(f"Place {known}g on the scale and press Enter...")
        sensor1.calibrate(known)
        input("Remove calibration weight and press Enter...")
        print("\nReading... (Ctrl+C to stop)\n")
        while True:
            print(f"  {sensor1.read_grams():8.1f} g")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sensor1.close()
        sensor2.close()
        sensor3.close()
        sck.close()
