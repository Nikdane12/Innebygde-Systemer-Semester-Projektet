# L298N motor driver
# IN1 (forward)  → GPIO 21
# IN2 (backward) → GND (hardwired, always forward)
# ENA (speed)    → PCA9685 CH_PUMP (duty cycle via I2C)

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import i2c
from gpiozero import OutputDevice
from time import sleep

forward_pin = OutputDevice(21)
forward_pin.on()   # IN1 HIGH — forward direction always on

print("Full speed (100%) for 5s")
i2c.set_duty(i2c.CH_PUMP, 100)
sleep(5)

print("Half speed (50%) for 5s")
i2c.set_duty(i2c.CH_PUMP, 50)
sleep(5)

print("Stop")
i2c.set_duty(i2c.CH_PUMP, 0)
forward_pin.close()
i2c.bus.close()
