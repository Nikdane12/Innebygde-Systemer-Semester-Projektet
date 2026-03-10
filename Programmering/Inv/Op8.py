#!/usr/bin/env python3
import serial
import time

ser = serial.Serial('/dev/ttyAMA0', 38400, timeout=2)
time.sleep(0.5)

print("Connected. Type a command (ADC, TMP, LED:1, LED:0, SERVO:90, BUZZ:500, BUZZ:0)")
print("Press Ctrl+C to quit\n")

while True:
    cmd = input(">>> ")
    ser.write((cmd + '\n').encode())
    response = ser.readline().decode().strip()
    print(f"Response: {response}\n")
