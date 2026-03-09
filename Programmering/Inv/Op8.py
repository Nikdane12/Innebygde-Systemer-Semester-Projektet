import serial

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=2)

message = "Hello AVR\r\n"
ser.write(message.encode('utf-8'))
print(f"Sent: {message.strip()}")

response = ser.readline().decode('utf-8').strip()
print(f"Received: {response}")

ser.close()
