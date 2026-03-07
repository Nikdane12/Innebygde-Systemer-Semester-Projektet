#!/usr/bin/env python3
import serial, time

ser = serial.Serial("/dev/ttyAMA10", 9600, timeout=2)
time.sleep(0.5)

# Read READY message from AVR
startup = ser.readline().decode(errors="replace").strip()
print("AVR says:", startup)

def send(cmd):
    """Send one byte, return reply line."""
    ser.write(cmd.encode())
    reply = ser.readline().decode(errors="replace").strip()
    print(f"  sent={cmd!r}  got={reply!r}")
    return reply

# Test ping
send('0')
time.sleep(0.5)

# Test temperature
send('1')
time.sleep(0.5)

# Test potentiometer
send('2')
time.sleep(0.5)

# Test LED on
send('3')
time.sleep(1)

# Test LED off
send('4')

ser.close()
print("Done.")