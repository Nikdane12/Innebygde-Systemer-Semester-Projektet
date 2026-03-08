import time
import serial

# Loopback test for Raspberry Pi 5 UART
# Hardware: connect GPIO14 (TX) directly to GPIO15 (RX) with a jumper wire
ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)

test_messages = [b'Hello\n', b'Loopback\n', b'Test123\n']

print("Starting UART loopback test...")
print("Make sure GPIO14 (TX) is connected to GPIO15 (RX)\n")

for msg in test_messages:
    ser.write(msg)
    time.sleep(0.1)  # Give time for data to loop back

    received = ser.readline()
    if received == msg:
        print(f"OK  Sent: {msg!r}  Received: {received!r}")
    else:
        print(f"FAIL  Sent: {msg!r}  Received: {received!r}")

ser.close()
print("\nDone.")
