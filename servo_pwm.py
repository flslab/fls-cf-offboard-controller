import time

from pi5RC import pi5RC

# Create servo instance on GPIO18
servo_a = pi5RC(18)
servo_b = pi5RC(19)

try:
    i = 0
    while True:
        angle = int(input(f"Enter angle between 0 and 180 for servo {i % 2 + 1}:\n"))
        i += 1
        if i % 2:
            servo_a.set(angle)
        else:
            servo_b.set(angle)
except KeyboardInterrupt:
    print("Program stopped by user")

finally:
    del servo_a
    del servo_b  # clean up PWM and pin state
