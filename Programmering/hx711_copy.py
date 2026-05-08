# HX711 load cell amplifier — multi-sensor with background reader threads
# Wiring: DT1 → GPIO 6, DT2 → GPIO 13, DT3 → GPIO 19
#         SCK (shared) → GPIO 5

from gpiozero import InputDevice, OutputDevice
from collections import deque
import statistics
import threading
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

# Shared clock — only one reader thread may pulse it at a time
sck = _open_output(PIN_SCK)
_sck_lock = threading.Lock()


class HX711:
    def __init__(self, pin_dt):
        self.dt           = _open_device(pin_dt)
        self._buf         = deque(maxlen=10)
        self.hx_offset    = 0.0
        self.scale_factor = 1.0

        # _cond protects _buf, _latest_raw, _sample_count
        self._cond         = threading.Condition()
        self._latest_raw   = None
        self._sample_count = 0
        self._stop         = threading.Event()
        self._thread = threading.Thread(target=self._reader_loop, daemon=True)
        self._thread.start()

    def _read_raw_blocking(self) -> int:
        # Wait for DT to go low (data ready). Sleep 1 ms between checks so
        # three reader threads don't peg the CPU spinning.
        timeout = time.time() + 1.0
        while self.dt.value == 1:
            if time.time() > timeout:
                raise TimeoutError("HX711 not responding — check wiring")
            time.sleep(0.001)

        # Bit-bang under the SCK lock so the three sensors can't clash on the
        # shared clock line.
        with _sck_lock:
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

    def _reader_loop(self):
        while not self._stop.is_set():
            try:
                raw = self._read_raw_blocking()
            except Exception:
                time.sleep(0.05)
                continue
            with self._cond:
                self._buf.append(raw)
                self._latest_raw = raw
                self._sample_count += 1
                self._cond.notify_all()

    def _wait_for_samples(self, n: int, timeout: float = 5.0):
        samples = []
        deadline = time.time() + timeout
        with self._cond:
            start = self._sample_count
            while len(samples) < n:
                target = start + len(samples) + 1
                while self._sample_count < target:
                    remaining = deadline - time.time()
                    if remaining <= 0:
                        raise TimeoutError("HX711 reader thread not producing samples")
                    self._cond.wait(timeout=remaining)
                samples.append(self._latest_raw)
        return samples

    def read_raw(self) -> int:
        with self._cond:
            if self._latest_raw is None:
                raise RuntimeError("no sample yet")
            return self._latest_raw

    def read_stable(self) -> float:
        with self._cond:
            if not self._buf:
                raise RuntimeError("no sample yet")
            return statistics.median(self._buf)

    def read_median(self, samples: int = 10) -> float:
        readings = self._wait_for_samples(samples)
        return statistics.median(readings)

    def tare(self, samples: int = 10) -> float:
        readings = self._wait_for_samples(samples)
        self.hx_offset = sum(readings) / len(readings)
        return self.hx_offset

    def calibrate(self, known_grams: float, samples: int = 10) -> float:
        readings = self._wait_for_samples(samples)
        tared = sum(r - self.hx_offset for r in readings) / len(readings)
        self.scale_factor = tared / known_grams
        return self.scale_factor

    def read_grams(self) -> float:
        return (self.read_stable() - self.hx_offset) / self.scale_factor

    def close(self):
        self._stop.set()
        self._thread.join(timeout=1.0)
        self.dt.close()


# Sensor instances
sensor1 = HX711(PIN_DT1)
sensor2 = HX711(PIN_DT2)
sensor3 = HX711(PIN_DT3)


if __name__ == "__main__":
    try:
        print("HX711 test — sensor 1")
        time.sleep(0.3)  # let reader thread collect a few samples
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
