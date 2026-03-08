#!/usr/bin/env python3
import serial, time

ser = serial.Serial("/dev/ttyAMA10", 38400, timeout=3)
time.sleep(0.5)

# Read whatever the AVR sends on startup
startup = ser.readline().decode(errors="replace").strip()
print("AVR startup message:", repr(startup))

# Send a single character and wait for echo
print("Sending 'A'...")
ser.write(b"A")
time.sleep(0.2)
echo = ser.read(1).decode(errors="replace")
print("Echo back:", repr(echo))

if echo == "A":
    print("SUCCESS - communication is working!")
else:
    print("FAIL - no echo received. Check wiring.")

ser.close()