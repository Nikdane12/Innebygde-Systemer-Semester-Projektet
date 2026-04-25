# L298N motor driver
# IN1 (forward)  → GPIO 21
# IN2 (backward) → GPIO 13
# ENA (speed)    → PCA9685 CH_PUMP (duty cycle via I2C)

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import i2c
from gpiozero import Motor
from time import sleep

motor = Motor(forward=21, backward=13)

print("Full speed forward for 3s")
motor.forward()
i2c.set_pwm(i2c.CH_PUMP, i2c.pump_to_us(100))
sleep(3)

print("Half speed forward for 2s")
i2c.set_pwm(i2c.CH_PUMP, i2c.pump_to_us(50))
sleep(2)

print("Full speed backward for 3s")
motor.backward()
i2c.set_pwm(i2c.CH_PUMP, i2c.pump_to_us(100))
sleep(3)

print("Stop")
i2c.set_pwm(i2c.CH_PUMP, i2c.pump_to_us(0))
motor.stop()
motor.close()
i2c.bus.close()
