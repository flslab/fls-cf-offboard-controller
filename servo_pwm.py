import argparse
import time

from pi5RC import pi5RC


pwm_pins = [19, 18, 12, 13]


def sign(n):
    if n < 0:
        return -1
    elif n > 0:
        return 1
    else:
        return 0


class Servo:
    def __init__(self, num=2, offsets=None):
        if offsets is None:
            offsets = [0, 0]
        self.servos = [pi5RC(pwm_pins[i]) for i in range(num)]
        self.offsets = offsets
        self.values = [None, None]
        self.tick_angle = 1
        self.tick_duration = 1/180

    def set(self, index, a):
        angle = a + self.offsets[index]
        if 0 <= angle <= 180:
            self.servos[index].set(angle)
            self.values[index] = angle

    def set_smooth(self, index, a):
        if self.values[index] is None:
            return self.set(index, a)

        angle = a + self.offsets[index]
        if 0 <= angle <= 180:
            start_angle = self.values[index]
            delta_angle = angle - start_angle
            n = abs(delta_angle) // self.tick_angle

            for i in range(n):
                angle_i = start_angle + (i + 1) * self.tick_angle * sign(delta_angle)
                self.servos[index].set(angle_i)
                self.values[index] = angle_i
                time.sleep(self.tick_duration)

            self.servos[index].set(angle)
            self.values[index] = angle

    def set_all_smooth(self, target_angles):
        if None in self.values:
            return self.set_all(target_angles)

        angles = [a + offset for a, offset in zip(target_angles, self.offsets)]
        if all(0 <= x <= 180 for x in angles):
            start_angles = [v for v in self.values]
            delta_angles = [angle - start_angle for angle, start_angle in zip(angles, start_angles)]
            ns = [abs(delta_angle) // self.tick_angle for delta_angle in delta_angles]

            for i in range(max(ns)):
                for index in range(len(self.servos)):
                    if i < ns[index]:
                        angle_i = start_angles[index] + (i + 1) * self.tick_angle * sign(delta_angles[index])
                        self.servos[index].set(angle_i)
                        self.values[index] = angle_i

                time.sleep(self.tick_duration)

            for index in range(len(self.servos)):
                self.servos[index].set(angles[index])
                self.values[index] = angles[index]

    def set_all(self, target_angles):
        for i in range(len(self.servos)):
            self.set(i, target_angles[i])

    def __del__(self):
        for servo in self.servos:
            del servo
        self.servos.clear()

    def __len__(self):
        return len(self.servos)

    def __getitem__(self, index):
        return self.servos[index]

    def __setitem__(self, index, value):
        self.set(index, value)


def range_test(servos, a, b, n=5):
    for i in range(n):
        for j in range(len(servos)):
            servos[j] = a

        time.sleep(1)

        for j in range(len(servos)):
            servos[j] = b

        time.sleep(1)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-n", default=2, type=int, help="number of servos [1, 4]")
    ap.add_argument("-i", action="store_true")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--smooth", action="store_true")
    ap.add_argument("--servo-type", type=str, help="type of light bender servo setting")
    ap.add_argument("--range-test", type=int, nargs=3, default=[], help="range test angle1 angle2 repetitions")
    ap.add_argument("--set-all", type=int, nargs="+", default=[], help="set angles")
    args = ap.parse_args()

    offsets = [0, -180] if args.servo_type == 'a' else [-90, -270]
    servos = Servo(args.n, offsets)

    try:
        if args.i:
            while True:
                user_input = input(f"Enter servo index and angle, e.g., 0 90:\n")
                if user_input:
                    j, angle = user_input.strip().split(" ")
                    j, angle = int(j), int(angle)

                    if 0 <= j < args.n:
                        if args.smooth:
                            servos.set_smooth(j, angle)
                        else:
                            servos.set(j, angle)
        elif args.all:
            while True:
                user_input = input(f"Enter angles for {args.n} servos, e.g., 0 180:\n")
                if user_input:
                    angles = user_input.strip().split(" ")
                    angles = list(map(int, angles))

                    if args.smooth:
                        servos.set_all_smooth(angles)
                    else:
                        servos.set_all(angles)
        elif len(args.range_test):
            range_test(servos, *args.range_test)
        elif len(args.set_all):
            servos.set_all(args.set_all)
            time.sleep(1)
            servos.set_all(args.set_all)

    except KeyboardInterrupt:
        print("Program stopped by user")

    finally:
        del servos
