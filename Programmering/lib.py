from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)
max = 6554
low = 3277

desired_duty = max
kit._pca.channels[0].duty_cycle = 65535 - desired_duty
kit._pca.channels[1].duty_cycle = 65535 - desired_duty
kit._pca.channels[2].duty_cycle = 65535 - desired_duty
kit._pca.channels[3].duty_cycle = 65535 - desired_duty
kit._pca.channels[4].duty_cycle = 65535 - desired_duty



