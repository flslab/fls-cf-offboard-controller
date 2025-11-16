from pi5RC import pi5RC


class Servo:
    def __init__(self):
        self.servo_a = pi5RC(18)
        self.servo_b = pi5RC(19)

    def set_a(self, a):
        self.servo_a.set(a)

    def set_b(self, b):
        self.servo_b.set(b)

    def set_a_b(self, a, b):
        self.set_a(a)
        self.set_b(b)

    def __del__(self):
        del self.servo_a
        del self.servo_b


if __name__ == '__main__':
    servo = Servo()
    try:
        i = 0
        while True:
            angle = int(input(f"Enter angle between 0 and 180 for servo {i % 2 + 1}:\n"))
            i += 1
            if i % 2:
                servo.set_a(angle)
            else:
                servo.set_b(angle)
    except KeyboardInterrupt:
        print("Program stopped by user")

    finally:
        del servo
