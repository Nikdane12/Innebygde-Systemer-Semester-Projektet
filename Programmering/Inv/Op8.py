import time
import serial

ser = serial.Serial('/dev/ttyAMA0', 38400, timeout=2)

while True:
    # Read message from AVR
    response = ser.readline().decode('utf-8').strip()
    if response:
        print(f"AVR: {response}")
    else:
        print("No response from AVR (not connected?)")

    # Send message back to AVR
    ser.write("Hello AVR\r\n".encode('utf-8'))
    time.sleep(1)
