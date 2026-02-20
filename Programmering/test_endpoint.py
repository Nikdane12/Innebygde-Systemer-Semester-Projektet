from gpiozero import Servo
from time import sleep

s = Servo(18, min_pulse_width=0.8/1000, max_pulse_width=2.2/1000)

for v in [-1, 0, 1, 0, -1]:
    s.value = v
    print("value:", v)
    sleep(2)
