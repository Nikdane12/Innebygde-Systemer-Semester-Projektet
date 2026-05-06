from adafruit_servokit import ServoKit

# Set channels to 16
kit = ServoKit(channels=16)

# Set servo at channel 0 to 90 degrees
kit.servo[0].angle = 90

# Set servo at channel 1 to 180 degrees
kit.servo[1].angle = 180
