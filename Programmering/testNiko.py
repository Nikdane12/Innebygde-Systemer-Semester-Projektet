from adafruit_servokit import ServoKit

kit = ServoKit(channels = 16)

max = 6554
min = 3277

prosent = 25 #25%

pwm = int(((max-min)*(prosent/100)) + min)

kit._pca.channel[0].duty_cycle = 65535 - pwm
