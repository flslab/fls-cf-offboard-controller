import threading
import time


class SmoothController:
    def __init__(self, rate=30):
        """
        initializes the controller with a specific update rate (Hz).
        """
        self.rate = rate
        self.groups = {}
        self.update_callbacks = []  # List of generic callbacks to run every loop
        self.running = True
        self.lock = threading.RLock()
        self.worker_thread = threading.Thread(target=self._update_loop, daemon=True)
        self.worker_thread.start()

    def register_group(self, name, initial_values, callback, ranges=None, offsets=None):
        """
        Registers a group of variables.
        name: Unique identifier for the group.
        initial_values: List of starting values.
        callback: Function to call when values change (receives list of new values).
        ranges: List of (min, max) tuples for each variable, or None.
        """
        with self.lock:
            # Deep copy values to ensure thread safety
            values = list(initial_values)
            n = len(values)
            if offsets is None:
                offsets = [0.0] * n

            self.groups[name] = {
                'values': values,  # Current interpolated values
                'targets': list(values),  # Final target values
                'starts': list(values),  # Values at start of current movement
                'start_times': [0.0] * n,
                'durations': [0.0] * n,
                'callback': callback,
                'ranges': ranges,
                'offsets': offsets
            }

    def set_group_values(self, name, target_values, duration=0.0):
        """
        Updates the target values for a group.
        name: Group identifier.
        target_values: List of new target values.
        duration: Time in seconds to reach the new targets.
        """
        with self.lock:
            group = self.groups.get(name)
            if not group:
                return

            now = time.time()
            # Update state for interpolation for each variable in the group
            for i, target in enumerate(target_values):
                if i < len(group['values']):
                    # Start from wherever we are currently
                    group['starts'][i] = group['values'][i]
                    group['targets'][i] = target + group['offsets'][i]
                    group['start_times'][i] = now
                    group['durations'][i] = duration

    def add_update_callback(self, callback):
        """
        Adds a generic callback that is executed on every update loop iteration.
        Useful for logging, tick hooks, or synchronization.
        """
        with self.lock:
            if callback not in self.update_callbacks:
                self.update_callbacks.append(callback)

    def remove_update_callback(self, callback):
        """
        Removes a previously added generic update callback.
        """
        with self.lock:
            if callback in self.update_callbacks:
                self.update_callbacks.remove(callback)

    def _update_loop(self):
        """
        Background thread loop.
        Calculates interpolated values and calls callbacks if changes occur.
        """
        while self.running:
            loop_start = time.time()

            with self.lock:
                # 1. Run generic update callbacks
                for cb in self.update_callbacks:
                    try:
                        cb()
                    except Exception as e:
                        print(f"Error in update callback: {e}")

                # 2. Process groups (interpolation)
                now = time.time()
                for name, group in self.groups.items():
                    changed = False
                    current_vals = group['values']
                    targets = group['targets']
                    starts = group['starts']
                    ranges = group['ranges']

                    for i in range(len(current_vals)):
                        # If we haven't reached target, calculate new value
                        if current_vals[i] != targets[i]:
                            elapsed = now - group['start_times'][i]
                            dur = group['durations'][i]

                            val = current_vals[i]

                            if dur <= 0 or elapsed >= dur:
                                val = targets[i]
                            else:
                                # Linear Interpolation
                                progress = elapsed / dur
                                val = starts[i] + (targets[i] - starts[i]) * progress

                            # Apply range constraints if they exist
                            if ranges and i < len(ranges) and ranges[i] is not None:
                                min_v, max_v = ranges[i]
                                val = max(min_v, min(val, max_v))

                            # Only update and flag change if value actually differs
                            if val != current_vals[i]:
                                current_vals[i] = val
                                changed = True

                    # If any value in the group changed, trigger the group callback
                    if changed:
                        try:
                            group['callback'](list(current_vals))
                        except Exception as e:
                            print(f"Error in callback for group {name}: {e}")

            # Rate limiting
            elapsed_work = time.time() - loop_start
            delay = (1.0 / self.rate) - elapsed_work
            if delay > 0:
                time.sleep(delay)

    def stop(self):
        self.running = False
        if self.worker_thread.is_alive():
            self.worker_thread.join(0.5)
