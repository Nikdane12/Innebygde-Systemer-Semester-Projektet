#!/usr/bin/env python3
"""
Read 3 HX711 load cell ADC modules from a Raspberry Pi 5, sampling roughly simultaneously.

Wiring assumption (recommended):
- Each HX711 has its own DOUT pin and its own SCK pin.
- All share Pi GND, and share 3V3 (or 5V if your HX711 board supports it + level shifting).
"""

import time
import csv
import threading
from dataclasses import dataclass
from typing import Optional, List

from gpiozero import DigitalInputDevice, DigitalOutputDevice


# ----------------------------
# HX711 driver (bit-banged)
# ----------------------------

@dataclass
class HX711Config:
    dout_gpio: int
    sck_gpio: int
    gain_pulses: int = 1
    # gain_pulses: number of extra pulses after 24 bits to set next conversion:
    #   1 pulse  -> Channel A, Gain 128
    #   2 pulses -> Channel B, Gain 32
    #   3 pulses -> Channel A, Gain 64


class HX711:
    def __init__(self, cfg: HX711Config):
        self.cfg = cfg
        self.dout = DigitalInputDevice(cfg.dout_gpio, pull_up=False)
        self.sck = DigitalOutputDevice(cfg.sck_gpio, initial_value=False)

        # Timing: HX711 is fairly forgiving; keep pulses short but not extreme.
        # We'll use a small sleep to avoid CPU jitter causing odd edges.
        self._half_period = 0.000002  # 2 µs

    def is_ready(self) -> bool:
        # DOUT goes LOW when data is ready
        return self.dout.value == 0

    def wait_ready(self, timeout_s: float = 0.2) -> bool:
        t0 = time.perf_counter()
        while (time.perf_counter() - t0) < timeout_s:
            if self.is_ready():
                return True
            time.sleep(0.0005)
        return False

    def _pulse(self):
        self.sck.on()
        time.sleep(self._half_period)
        self.sck.off()
        time.sleep(self._half_period)

    def read_raw(self, timeout_s: float = 0.2) -> int:
        """
        Read one 24-bit sample and return signed integer.
        """
        if not self.wait_ready(timeout_s=timeout_s):
            raise TimeoutError(f"HX711 not ready (DOUT stayed high) on GPIO{self.cfg.dout_gpio}")

        # Read 24 bits MSB first
        value = 0
        for _ in range(24):
            self.sck.on()
            time.sleep(self._half_period)

            bit = 1 if self.dout.value else 0
            value = (value << 1) | bit

            self.sck.off()
            time.sleep(self._half_period)

        # Set gain/channel for NEXT conversion by extra pulses
        for _ in range(self.cfg.gain_pulses):
            self._pulse()

        # Convert from 24-bit two's complement to signed int
        if value & 0x800000:
            value -= 1 << 24
        return value


# ----------------------------
# Parallel sampler
# ----------------------------

@dataclass
class Sample:
    t: float
    raw1: int
    raw2: int
    raw3: int


def read_three_hx711(
    hx_list: List[HX711],
    rate_hz: float = 20.0,
    duration_s: float = 10.0,
    outfile: Optional[str] = "hx711_3ch.csv",
):
    """
    Quasi-synchronous sampling using threads + a barrier.
    Each sample: all 3 threads are released together, each reads one HX711.
    """
    assert len(hx_list) == 3, "Provide exactly 3 HX711 instances"

    period = 1.0 / rate_hz
    n_samples = int(duration_s * rate_hz)

    barrier = threading.Barrier(4)  # 3 workers + main thread
    stop_flag = threading.Event()

    latest = {"v": [0, 0, 0], "err": [None, None, None]}
    lock = threading.Lock()

    def worker(i: int):
        while not stop_flag.is_set():
            # Wait for main thread to trigger a read
            try:
                barrier.wait()
            except threading.BrokenBarrierError:
                return

            # Attempt a read
            try:
                val = hx_list[i].read_raw(timeout_s=0.25)
                err = None
            except Exception as e:
                val = 0
                err = e

            with lock:
                latest["v"][i] = val
                latest["err"][i] = err

            # Signal completion
            try:
                barrier.wait()
            except threading.BrokenBarrierError:
                return

    threads = [threading.Thread(target=worker, args=(i,), daemon=True) for i in range(3)]
    for th in threads:
        th.start()

    rows: List[Sample] = []

    t_start = time.perf_counter()
    next_t = t_start

    for _ in range(n_samples):
        # wait for next tick
        now = time.perf_counter()
        if now < next_t:
            time.sleep(next_t - now)

        # Release all workers to read "together"
        try:
            barrier.wait()
            barrier.wait()
        except threading.BrokenBarrierError:
            break

        # Collect results
        with lock:
            v1, v2, v3 = latest["v"]
            e1, e2, e3 = latest["err"]

        # Optional: if you want, print errors
        if e1 or e2 or e3:
            print("Read error(s):",
                  f"1={repr(e1)}" if e1 else "1=OK",
                  f"2={repr(e2)}" if e2 else "2=OK",
                  f"3={repr(e3)}" if e3 else "3=OK")

        t_rel = time.perf_counter() - t_start
        rows.append(Sample(t=t_rel, raw1=v1, raw2=v2, raw3=v3))

        next_t += period

    stop_flag.set()
    try:
        barrier.abort()
    except Exception:
        pass

    # Save CSV
    if outfile:
        with open(outfile, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_s", "raw_1", "raw_2", "raw_3"])
            for r in rows:
                w.writerow([f"{r.t:.6f}", r.raw1, r.raw2, r.raw3])

        print(f"Saved {len(rows)} samples to {outfile}")

    return rows


# ----------------------------
# Main: fill in pins later
# ----------------------------

def main():
    # TODO: replace these placeholders with your real GPIO numbers
    HX1_DOUT = 0   # e.g. 5
    HX1_SCK  = 0   # e.g. 6
    HX2_DOUT = 0
    HX2_SCK  = 0
    HX3_DOUT = 0
    HX3_SCK  = 0

    # Gain/channel: default 1 pulse = Channel A, Gain 128
    hx1 = HX711(HX711Config(dout_gpio=HX1_DOUT, sck_gpio=HX1_SCK, gain_pulses=1))
    hx2 = HX711(HX711Config(dout_gpio=HX2_DOUT, sck_gpio=HX2_SCK, gain_pulses=1))
    hx3 = HX711(HX711Config(dout_gpio=HX3_DOUT, sck_gpio=HX3_SCK, gain_pulses=1))

    # Example: 20 Hz for 10 seconds
    read_three_hx711([hx1, hx2, hx3], rate_hz=20.0, duration_s=10.0, outfile="hx711_3ch.csv")


if __name__ == "__main__":
    main()