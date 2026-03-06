#!/usr/bin/env python3
import serial, time

PORT = "/dev/ttyAMA10"   # change to /dev/ttyAMA0 or /dev/ttyUSB0 if needed
BAUDRATE = 9600
TIMEOUT  = 2.0

ser = serial.Serial(PORT, BAUDRATE, timeout=TIMEOUT)

def send(cmd):
    ser.reset_input_buffer()
    ser.write((cmd + "\n").encode())
    return ser.readline().decode(errors="replace").strip()

def ping():
    r = send("PING")
    print("Ping:", r)

def read_temperature():
    r = send("ADC_TMP")
    _, mv, degC = r.split(":")
    return int(mv), int(degC)

def read_pot():
    r = send("ADC_POT")
    return int(r.split(":")[1])

def led(state):
    r = send("LED_ON" if state else "LED_OFF")
    print(r)

def monitor(interval=1.0):
    print(f"{'Time':>8}  {'mV':>6}  {'°C':>6}")
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