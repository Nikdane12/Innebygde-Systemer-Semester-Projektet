import time
import serial

ser = serial.Serial('/dev/ttyAMA10', 9600, timeout=1)

while True:
    signal = ser.read()
    if signal == b'1':
        # Dummy processing
        dummy_value = 1.75  # Replace this with actual processing if needed

        print(f"Sending dummy value: {dummy_value}")
        ser.write(f'{dummy_value:.2f}\n'.encode('utf-8'))
    else:
        time.sleep(1)  # Sleep for 1 second before checking again 