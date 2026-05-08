# HX711 load cell amplifier — multi-sensor
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

# Shared clock + lock to prevent multiple sensors from interleaving clock pulses
sck = _open_output(PIN_SCK)
sck_lock = threading.Lock()

class HX711:
    def __init__(self, pin_dt):
        self.dt           = _open_device(pin_dt)
        self._buf         = deque(maxlen=10)
        self.hx_offset    = 0.0
        self.scale_factor = 1.0

        # Background polling state
        self._latest_grams = 0.0
        self._latest_raw   = 0
        self._data_lock    = threading.Lock()
        self._running      = False
        self._thread       = None

    def read_raw(self) -> int:
        timeout = time.time() + 1.0
        while self.dt.value == 1:
            if time.time() > timeout:
                raise TimeoutError("HX711 not responding — check wiring")
        raw = 0
        # Lock the shared clock so concurrent sensors don't corrupt each other's reads
        with sck_lock:
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
        # Takes N fresh readings and returns their median
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
        # Blocking read — used during tare/calibrate or if you want a fresh value
        return (self.read_stable() - self.hx_offset) / self.scale_factor

    # ── Background polling ──────────────────────────────────────────────

    def start_background(self, interval: float = 0.1) -> None:
        """Start polling the sensor in a background thread.
        Main loop can then call get_grams() without blocking."""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(
            target=self._poll_loop,
            args=(interval,),
            daemon=True,
        )
        self._thread.start()

    def stop_background(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=1.5)
            self._thread = None

    def _poll_loop(self, interval: float) -> None:
        while self._running:
            try:
                raw = self.read_raw()
                self._buf.append(raw)
                stable = statistics.median(self._buf)
                grams = (stable - self.hx_offset) / self.scale_factor
                with self._data_lock:
                    self._latest_raw   = raw
                    self._latest_grams = grams
            except TimeoutError:
                # Sensor not ready this cycle — skip and try again
                pass
            except Exception:
                # Don't let the thread die on transient errors
                pass
            time.sleep(interval)

    def get_grams(self) -> float:
        """Non-blocking — returns the most recent cached reading in grams."""
        with self._data_lock:
            return self._latest_grams

    def get_raw(self) -> int:
        """Non-blocking — returns the most recent cached raw value."""
        with self._data_lock:
            return self._latest_raw

    # ────────────────────────────────────────────────────────────────────

    def close(self):
        self.stop_background()
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

        # Start background polling — main loop now never blocks on the HX711
        sensor1.start_background(interval=0.1)

        print("\nReading... (Ctrl+C to stop)\n")
        while True:
            print(f"  {sensor1.get_grams():8.1f} g")
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        sensor1.close()
        sensor2.close()
        sensor3.close()
        sck.close()