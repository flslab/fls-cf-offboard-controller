import datetime
import functools
import time


class CommandLogger:
    def __init__(self, target, log_function, execute=True):
        # Setup background logging
        self.log_function = log_function
        self.start_time = time.time()
        self._wrapped_instance = target
        self.class_name = target.__class__.__name__
        self.execution = execute

    def set_start_time(self, timestamp=None):
        if not timestamp:
            timestamp = time.time()
        self.start_time = timestamp

    def __getattr__(self, name):
        # Get the attribute from the base class safely
        attr = getattr(self._wrapped_instance, name)

        # Intercept methods, but ignore private/internal ones
        if callable(attr) and not name.startswith('_'):
            @functools.wraps(attr)  # Keeps the original function's name and docstring
            def wrapper(*args, **kwargs):
                timestamp = time.time() - self.start_time
                log_entry = {'time': timestamp, "args": args,  "kwargs": kwargs}
                self.log_function(group_name='commands', entry=log_entry, name=f"{self.class_name}.{name}")

                if self.execution:
                    return attr(*args, **kwargs)

            return wrapper

        return attr
