from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
max = 8000
low = 3277

desired_duty = max
kit._pca.channels[3].duty_cycle = 65535 - desired_duty




