import time
import serial
import threading

ser = serial.Serial('/dev/ttyAMA0', 38400, timeout=1)

def receive_loop():
    while True:
        response = ser.readline().decode('utf-8').strip()
        if response:
            print(f"AVR: {response}")

def send_loop():
    while True:
        msg = input("RPi: ")
        ser.write((msg + '\r\n').encode('utf-8'))
        time.sleep(1)

recv_thread = threading.Thread(target=receive_loop, daemon=True)
recv_thread.start()

send_loop()
