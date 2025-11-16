import time

from pi5RC import pi5RC

# Create servo instance on GPIO18
servo_a = pi5RC(18)
servo_b = pi5RC(19)

try:
    while True:
        angle = int(input("Enter angle between 0 and 180:\n"))
        servo_a.set(angle)
        servo_b.set(angle)
except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    del servo_a
    del servo_b  # clean up PWM and pin state
