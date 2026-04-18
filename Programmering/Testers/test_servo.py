from gpiozero import Servo
from time import sleep

servo1 = Servo(18)  # GPIO 18 (pin 12)
servo2 = Servo(19)  # GPIO 19 (pin 35)

while True:
    servo1.min()
    servo2.min()
    sleep(1)

    servo1.mid()
    servo2.mid()
    sleep(1)

    servo1.max()
    servo2.max()
    sleep(1)
