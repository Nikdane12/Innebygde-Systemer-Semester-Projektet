import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
PIN = 33
GPIO.setup(PIN, GPIO.OUT)

try:
    while True:
        GPIO.output(PIN, 1)
        time.sleep(0.5)
        GPIO.output(PIN, 0)
        time.sleep(0.5)
finally:
    GPIO.cleanup()
