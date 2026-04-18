#!/usr/bin/env python3
#L298N motordriver – test
#  IN1  → GPIO 17  (fremover)
#  IN2  → GPIO 27  (bakover)
#  ENA  → GPIO 19  (PWM hastighet)


from gpiozero import Motor
from time import sleep

# enable=19 knytter ENA-pinnen til GPIO 19 (PWM)
motor = Motor(forward=17, backward=27, enable=19, pwm=True)

print("Full hastighet fremover i 2s")
motor.forward(1.0)
sleep(30)

print("Halv hastighet fremover i 2s")
motor.forward(0.5)
sleep(2)

print("Stopp")
motor.stop()
