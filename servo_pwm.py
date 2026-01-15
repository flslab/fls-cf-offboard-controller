import argparse
import time

# Assuming pi5RC is available in the environment as per original code
from pi5RC import pi5RC
from smooth_controller import SmoothController

pwm_pins = [19, 18, 12, 13]


def sign(n):
    if n < 0:
        return -1
    elif n > 0:
        return 1
    else:
        return 0


class Servo:
    """
    handles hardware offsets and direct control.
    """

    def __init__(self, num=2, offsets=None):
        if offsets is None:
            offsets = [0, 0]
        self.servos = [pi5RC(pwm_pins[i]) for i in range(num)]
        self.offsets = offsets

    def set(self, index, a):
        """Sets servo angle immediately."""
        angle = a + self.offsets[index]
        if 0 <= angle <= 180:
            self.servos[index].set(angle)

    def set_all(self, target_angles):
        """Sets all servos immediately."""
        for i in range(len(self.servos)):
            if i < len(target_angles):
                self.set(i, target_angles[i])

    def __del__(self):
        # for servo in self.servos:
        #     if servo:
        #         del servo
        self.servos.clear()

    def __len__(self):
        return len(self.servos)

    def __getitem__(self, index):
        return self.servos[index]

    def __setitem__(self, index, value):
        self.set(index, value)


def range_test(servos, a, b, n=5):
    # This test function uses explicit sleeps and direct set() calls.
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
    ranges = [(0, 180), (180, 360)] if args.servo_type == 'a' else [(90, 270), (270, 450)]
    initial_values = (1, 181) if args.servo_type == 'a' else (181, 361)
    servos = Servo(args.n, offsets)

    # ---------------------------------------------------------
    # Demonstration of using SmoothController with Servo class
    # ---------------------------------------------------------

    # 1. Initialize the generic smooth controller
    controller = SmoothController(rate=100)

    # 2. Register callbacks to glue the Controller to the Servo instance
    #    We register each servo as an independent group "servo_{i}"
    #    so we can control them individually or together.
    for i in range(args.n):
        # Define a closure to capture the specific servo index for the callback
        def make_servo_callback(idx):
            return lambda values: servos.set(idx, values[0])


        controller.register_group(
            name=f"servo_{i}",
            initial_values=[initial_values[i]],  # Controller assumes 0 start; hardware will snap on first update
            callback=make_servo_callback(i),
            ranges=[ranges[i]]
        )

    try:
        if args.i:
            while True:
                user_input = input(f"Enter servo index and angle, e.g., 0 90:\n")
                if user_input:
                    try:
                        parts = user_input.strip().split(" ")
                        if len(parts) >= 2:
                            j, angle = int(parts[0]), int(parts[1])
                            if 0 <= j < args.n:
                                # Determine duration based on user flag
                                duration = 1.0 if args.smooth else 0.0

                                # Send command to controller
                                # Even for instant movement (duration=0), we go through the controller
                                # to keep its internal state synchronized with the hardware.
                                controller.set_group_values(f"servo_{j}", [angle], duration=duration)

                                mode = "smooth" if args.smooth else "instant"
                                print(f"Command sent: Servo {j} -> {angle} ({mode})")
                    except ValueError:
                        print("Invalid input format.")

        elif args.all:
            while True:
                user_input = input(f"Enter angles for {args.n} servos, e.g., 0 180:\n")
                if user_input:
                    try:
                        angles = list(map(int, user_input.strip().split(" ")))
                        # Update all controller groups
                        if len(angles) <= args.n:
                            for idx, angle in enumerate(angles):
                                duration = 1.0 if args.smooth else 0.0
                                controller.set_group_values(f"servo_{idx}", [angle], duration=duration)

                            mode = "smooth" if args.smooth else "instant"
                            print(f"Command sent: All -> {angles} ({mode})")
                    except ValueError:
                        print("Invalid input format.")

        elif len(args.range_test):
            # Range test calls servo.set() directly, bypassing the controller.
            # Note: This will desynchronize the controller's internal state from the hardware.
            range_test(servos, *args.range_test)

        elif len(args.set_all):
            servos.set_all(args.set_all)
            time.sleep(1)
            servos.set_all(args.set_all)

    except KeyboardInterrupt:
        print("\nProgram stopped by user")

    finally:
        # Stop the controller thread and cleanup
        controller.stop()
        del servos