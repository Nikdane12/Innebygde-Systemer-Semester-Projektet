#!/usr/bin/env python3
import serial, time

ser = serial.Serial("/dev/ttyAMA10", 9600, timeout=3)

print("Listening for 5 seconds - reset the AVR now...")
for _ in range(10):
    line = ser.readline().decode(errors="replace").strip()
    if line:
        print("Received:", repr(line))

print("Sending PING...")
ser.write(b"PING\n")
time.sleep(1)
reply = ser.readline().decode(errors="replace").strip()
print("Reply:", repr(reply))