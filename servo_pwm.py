import argparse
import time
import threading

# Assuming pi5RC is available in the environment as per original code
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

        # self.values stores the current hardware angle (float)
        self.values = [None] * num
        # self.target_values stores the desired destination angle
        self.target_values = [None] * num

        # Interpolation state
        self.start_values = [None] * num
        self.start_times = [0.0] * num
        self.move_durations = [1.0] * num

        # Threading setup for non-blocking smooth movement
        self.running = True
        self.lock = threading.RLock()  # Re-entrant lock to allow methods to call each other
        self.worker_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.worker_thread.start()

    def _update_loop(self):
        """
        Background thread that moves servos smoothly based on time duration.
        """
        while self.running:
            with self.lock:
                now = time.time()
                for i in range(len(self.servos)):
                    # Skip if not initialized or no target set
                    if self.values[i] is None or self.target_values[i] is None:
                        continue

                    # Check if movement is needed
                    if self.values[i] != self.target_values[i]:
                        elapsed = now - self.start_times[i]
                        duration = self.move_durations[i]

                        if duration <= 0:
                            # Instant move if duration is 0
                            self.values[i] = self.target_values[i]
                            self.servos[i].set(self.values[i])
                        elif elapsed >= duration:
                            # Movement finished
                            self.values[i] = self.target_values[i]
                            self.servos[i].set(self.values[i])
                        else:
                            # Interpolate position
                            progress = elapsed / duration
                            start = self.start_values[i]
                            target = self.target_values[i]

                            # Linear interpolation (Lerp)
                            current_angle = start + (target - start) * progress

                            self.values[i] = current_angle
                            self.servos[i].set(current_angle)

            # Update frequency ~100Hz
            time.sleep(0.01)

    def set(self, index, a):
        """Sets servo angle immediately (snaps to position)."""
        angle = a + self.offsets[index]
        if 0 <= angle <= 180:
            with self.lock:
                self.servos[index].set(angle)
                self.values[index] = angle
                self.target_values[index] = angle  # Stop any ongoing smooth move
                self.start_values[index] = angle

    def set_smooth(self, index, a, duration=1.0):
        """
        Sets the target angle and returns immediately.
        The servo will reach the target in 'duration' seconds.
        """
        angle = a + self.offsets[index]
        if 0 <= angle <= 180:
            with self.lock:
                # If first time (uninitialized), snap to position immediately
                if self.values[index] is None:
                    self.set(index, a)
                    return

                # Capture current state as start state for smooth transition
                self.start_values[index] = self.values[index]
                self.target_values[index] = angle
                self.start_times[index] = time.time()
                self.move_durations[index] = duration

    def set_all_smooth(self, target_angles, duration=1.0):
        """
        Sets targets for multiple servos and returns immediately.
        All servos will reach their targets in 'duration' seconds.
        """
        with self.lock:
            # If any servo is uninitialized, snap all to start positions
            if any(v is None for v in self.values):
                self.set_all(target_angles)
                return

            now = time.time()
            for i, a in enumerate(target_angles):
                if i < len(self.servos):
                    angle = a + self.offsets[i]
                    if 0 <= angle <= 180:
                        # Capture current state for each servo
                        self.start_values[i] = self.values[i]
                        self.target_values[i] = angle
                        self.start_times[i] = now
                        self.move_durations[i] = duration

    def set_all(self, target_angles):
        """Sets all servos immediately."""
        for i in range(len(self.servos)):
            if i < len(target_angles):
                self.set(i, target_angles[i])

    def __del__(self):
        self.running = False
        # Attempt to join thread if it's alive, but don't block forever
        if hasattr(self, 'worker_thread') and self.worker_thread.is_alive():
            self.worker_thread.join(0.1)

        for servo in self.servos:
            if servo:
                del servo
        self.servos.clear()

    def __len__(self):
        return len(self.servos)

    def __getitem__(self, index):
        return self.servos[index]

    def __setitem__(self, index, value):
        self.set(index, value)


def range_test(servos, a, b, n=5):
    # This test function uses explicit sleeps, so we don't need to change it.
    # It relies on set(), which is immediate.
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
                    try:
                        parts = user_input.strip().split(" ")
                        if len(parts) >= 2:
                            j, angle = int(parts[0]), int(parts[1])
                            if 0 <= j < args.n:
                                if args.smooth:
                                    servos.set_smooth(j, angle, duration=1.0)
                                    print(f"Command sent: Servo {j} -> {angle} (smooth, 1.0s)")
                                else:
                                    servos.set(j, angle)
                                    print(f"Command sent: Servo {j} -> {angle} (instant)")
                    except ValueError:
                        print("Invalid input format.")

        elif args.all:
            while True:
                user_input = input(f"Enter angles for {args.n} servos, e.g., 0 180:\n")
                if user_input:
                    try:
                        angles = list(map(int, user_input.strip().split(" ")))
                        if args.smooth:
                            servos.set_all_smooth(angles, duration=1.0)
                            print(f"Command sent: All -> {angles} (smooth, 1.0s)")
                        else:
                            servos.set_all(angles)
                            print(f"Command sent: All -> {angles} (instant)")
                    except ValueError:
                        print("Invalid input format.")

        elif len(args.range_test):
            range_test(servos, *args.range_test)
        elif len(args.set_all):
            servos.set_all(args.set_all)
            time.sleep(1)
            servos.set_all(args.set_all)

    except KeyboardInterrupt:
        print("\nProgram stopped by user")

    finally:
        # Explicit cleanup to stop thread
        servos.__del__()