#!/usr/bin/env python3
# pi_controller.py - Controls IO-kort over USART

import serial
import time

# RPi UART port - check with 'ls /dev/serial*'
PORT = '/dev/ttyAMA0'
BAUD = 38400

ser = serial.Serial(PORT, BAUD, timeout=2)
time.sleep(0.5)  # let AVR settle

def send_cmd(cmd):
    ser.write((cmd + '\n').encode())
    response = ser.readline().decode().strip()
    return response

# Task 2 - ADC measurement
print("=== ADC Measurement ===")
response = send_cmd("ADC")
print(f"Response: {response}")  # e.g. ADC:2048

# Task 3 - Full control examples
print("\n=== IO Control ===")

print(send_cmd("LED:1"))        # LED on
time.sleep(1)
print(send_cmd("LED:0"))        # LED off

print(send_cmd("SERVO:90"))     # servo to 90 degrees
time.sleep(1)
print(send_cmd("SERVO:0"))      # servo to 0 degrees

print(send_cmd("BUZZ:500"))     # buzzer at 500 Hz
time.sleep(0.5)
print(send_cmd("BUZZ:0"))       # buzzer off

print(send_cmd("TMP"))          # temperature reading

ser.close()