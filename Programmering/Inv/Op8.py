#!/usr/bin/env python3
import serial, time

PORT     = "/dev/ttyAMA10"
BAUDRATE = 9600
TIMEOUT  = 2.0

ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)

def send(cmd):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    reply = ser.readline().decode(errors="replace").strip()
    print(f"  [DEBUG] sent={cmd!r}  reply={reply!r}")   # remove once working
    return reply

def ping():
    r = send("PING")
    print("Ping:", r)

def read_temperature():
    r = send("ADC_TMP")
    parts = r.split(":")
    if len(parts) != 3 or parts[0] != "TMP":
        raise ValueError(f"Bad TMP reply: {r!r}")
    return int(parts[1]), int(parts[2])

def read_pot():
    r = send("ADC_POT")
    parts = r.split(":")
    if len(parts) != 2 or parts[0] != "POT":
        raise ValueError(f"Bad POT reply: {r!r}")
    return int(parts[1])

def led(state):
    r = send("LED_ON" if state else "LED_OFF")
    print(r)

def monitor(interval=1.0):
    print(f"{'Time':>8}  {'mV':>6}  {'degC':>6}")
    print("-" * 28)
    t0 = time.time()
    try:
        while True:
            mv, degC = read_temperature()
            print(f"{time.time()-t0:8.1f}  {mv:6}  {degC:6}")
            time.sleep(interval)
    except KeyboardInterrupt:
        print("\nStopped.")

# ── Run ───────────────────────────────────────────────────────────
ping()
monitor()