import time
import serial

ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=2)

def send_command(cmd):
    ser.write((cmd + '\r\n').encode('utf-8'))
    response = ser.readline().decode('utf-8').strip()
    return response

# Wait for AVR to send READY on startup
print("Waiting for AVR...")
ready = ser.readline().decode('utf-8').strip()
print(f"AVR: {ready}")

# PING test
resp = send_command("PING")
print(f"PING -> {resp}")

# Read temperature
resp = send_command("ADC_TMP")
print(f"ADC_TMP -> {resp}")
if resp.startswith("TMP:"):
    parts = resp.split(":")
    print(f"  Temperature: {parts[2]} C  ({parts[1]} mV)")

# Read potentiometer
resp = send_command("ADC_POT")
print(f"ADC_POT -> {resp}")
if resp.startswith("POT:"):
    raw = int(resp.split(":")[1])
    print(f"  Potentiometer raw: {raw}  ({raw / 4095 * 100:.1f}%)")

# LED test
print(send_command("LED_ON"))
time.sleep(1)
print(send_command("LED_OFF"))

ser.close()
