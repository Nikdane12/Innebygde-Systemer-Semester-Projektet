import serial

ser = serial.Serial('/dev/ttyAMA0', 38400, timeout=None)

while True:
    msg = input("Send to AVR: ")
    ser.write((msg + '\r\n').encode('utf-8'))
