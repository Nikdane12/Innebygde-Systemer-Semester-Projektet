# L298N motor driver via PCA9685 (I2C) + GPIO
# IN1 → GPIO 19  (forward enable, set HIGH)
# IN2 → GND      (hardwired LOW = backward disabled)
# ENA → PCA9685 CH_PUMP (PWM speed via duty cycle)

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import i2c
from gpiozero import Motor
from time import sleep

motor = Motor(forward=19, backward=None, enable=None, pwm=True)
motor.forward()   # set IN1 HIGH — direction always forward

print("Full speed (100%) for 30s")
i2c.set_duty(i2c.CH_PUMP, 100)
sleep(30)

print("Half speed (50%) for 2s")
i2c.set_duty(i2c.CH_PUMP, 50)
sleep(2)

print("Stop")
i2c.set_duty(i2c.CH_PUMP, 0)
motor.close()
i2c.bus.close()
