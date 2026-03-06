#!/usr/bin/env python3
"""
io_card.py  –  Oppgave 8: Kommunikasjon med IO-kort
Raspberry Pi side

Usage examples
--------------
python3 io_card.py ping
python3 io_card.py adc_tmp
python3 io_card.py adc_pot
python3 io_card.py led on
python3 io_card.py led off
python3 io_card.py monitor        # stream temperature every second
"""

import serial
import sys
import time

# ── Serial port configuration ─────────────────────────────────────
PORT     = "/dev/ttyS0"   # or /dev/ttyAMA0, /dev/ttyUSB0 – adjust as needed
BAUDRATE = 9600
TIMEOUT  = 2.0            # seconds to wait for a reply


def open_port() -> serial.Serial:
    """Open and return the serial port."""
    return serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)


# ── Low-level send / receive ──────────────────────────────────────
def send_command(ser: serial.Serial, cmd: str) -> str:
    """
    Send a newline-terminated command and return the stripped reply line.
    Raises TimeoutError if no reply arrives within TIMEOUT seconds.
    """
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    reply = ser.readline().decode(errors="replace").strip()
    if not reply:
        raise TimeoutError(f"No reply to '{cmd}' within {TIMEOUT}s")
    return reply


# ── High-level helpers ────────────────────────────────────────────
def ping(ser: serial.Serial) -> bool:
    reply = send_command(ser, "PING")
    print(f"Ping reply: {reply}")
    return reply == "PONG"


def read_temperature(ser: serial.Serial) -> tuple[int, int]:
    """
    Returns (millivolts, degrees_celsius) from the TMP235 sensor.
    """
    reply = send_command(ser, "ADC_TMP")
    # Expected format: "TMP:<mv>:<degC>"
    parts = reply.split(":")
    if len(parts) != 3 or parts[0] != "TMP":
        raise ValueError(f"Unexpected TMP reply: {reply}")
    mv   = int(parts[1])
    degC = int(parts[2])
    return mv, degC


def read_potentiometer(ser: serial.Serial) -> int:
    """
    Returns the raw 12-bit ADC value (0–4095) from the potentiometer.
    """
    reply = send_command(ser, "ADC_POT")
    parts = reply.split(":")
    if len(parts) != 2 or parts[0] != "POT":
        raise ValueError(f"Unexpected POT reply: {reply}")
    return int(parts[1])


def set_led(ser: serial.Serial, state: bool) -> str:
    """Turn the debug LED on or off. Returns the reply from the MCU."""
    cmd   = "LED_ON" if state else "LED_OFF"
    reply = send_command(ser, cmd)
    print(f"LED reply: {reply}")
    return reply


def monitor_temperature(ser: serial.Serial, interval: float = 1.0):
    """Continuously print temperature until Ctrl-C."""
    print("Monitoring temperature (Ctrl-C to stop) …")
    print(f"{'Time':>8}  {'mV':>6}  {'°C':>6}")
    print("-" * 28)
    start = time.time()
    try:
        while True:
            mv, degC = read_temperature(ser)
            elapsed  = time.time() - start
            print(f"{elapsed:8.1f}  {mv:6}  {degC:6}")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped.")


# ── Command-line interface ────────────────────────────────────────
def main():
    args = sys.argv[1:]

    if not args:
        print(__doc__)
        sys.exit(0)

    with open_port() as ser:
        cmd = args[0].lower()

        if cmd == "ping":
            ping(ser)

        elif cmd == "adc_tmp":
            mv, degC = read_temperature(ser)
            print(f"Temperature: {degC} °C  ({mv} mV)")

        elif cmd == "adc_pot":
            raw = read_potentiometer(ser)
            percent = round(raw / 4095 * 100, 1)
            print(f"Potentiometer: {raw} / 4095  ({percent}%)")

        elif cmd == "led":
            if len(args) < 2 or args[1].lower() not in ("on", "off"):
                print("Usage: python3 io_card.py led on|off")
                sys.exit(1)
            set_led(ser, args[1].lower() == "on")

        elif cmd == "monitor":
            interval = float(args[1]) if len(args) > 1 else 1.0
            monitor_temperature(ser, interval)

        else:
            print(f"Unknown command: {cmd}")
            print(__doc__)
            sys.exit(1)


if __name__ == "__main__":
    main()