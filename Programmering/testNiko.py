from adafruit_servokit import ServoKit

kit = ServoKit(channels = 16)

max = 6554
min = 3277

angle = 25 #25%

pwm = int(((max-min)*(angle/360)) + min)

kit._pca.channels[0].duty_cycle = 65535 - pwm
