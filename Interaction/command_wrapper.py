import datetime
import copy
import functools
import logging
import time

import numpy as np

from Interaction.live_logger import LiveLoggerError


logger = logging.getLogger(__name__)


class CommandWrapper:
    def __init__(self, target, log_function, execute=True, offset=np.zeros(3), start_time=None):
        # Setup background logging
        self.log_function = log_function
        self.start_time = start_time if start_time is not None else time.time()
        self._wrapped_instance = target
        self.class_name = target.__class__.__name__
        self.execution = execute
        self.offset = offset

    def set_start_time(self, timestamp=None):
        if not timestamp:
            timestamp = time.time()
        self.start_time = timestamp

    def for_safety_cleanup(self):
        """Clone for shutdown commands that must survive a failed flight log.

        This is exclusively for the landing/stop cleanup path, never normal
        flight control. The original wrapper remains fail-fast. Only the
        explicit storage/queue failure type is bypassed; programming mistakes
        in logging and errors from the actual command still propagate.
        """
        original_log = self.log_function

        def safety_log(*args, **kwargs):
            try:
                return original_log(*args, **kwargs)
            except LiveLoggerError:
                logger.error(
                    "Flight logging failed during safety cleanup; continuing "
                    "with the landing/stop command without a flight-log record",
                    exc_info=True,
                )

        return CommandWrapper(
            self._wrapped_instance,
            safety_log if original_log else None,
            execute=self.execution,
            offset=copy.copy(self.offset),
            start_time=self.start_time,
        )

    def __getattr__(self, name):
        # Get the attribute from the base class safely
        attr = getattr(self._wrapped_instance, name)

        # Intercept methods, but ignore private/internal ones
        if callable(attr) and not name.startswith('_'):
            @functools.wraps(attr)  # Keeps the original function's name and docstring
            def wrapper(*args, **kwargs):
                if name == 'send_position_setpoint' or name == 'go_to':
                    # args are likely (x, y, z, yaw)
                    # We convert to list to mutate them
                    modified_args = list(args)
                    modified_args[0] = float(modified_args[0]) + self.offset[0]
                    modified_args[1] = float(modified_args[1]) + self.offset[1]
                    modified_args[2] = float(modified_args[2]) + self.offset[2]

                    args = tuple(modified_args)

                if self.log_function:
                    timestamp = time.time() - self.start_time
                    log_entry = {'time': timestamp, 'args': args, 'kwargs': kwargs}
                    self.log_function(group_name='commands', entry=log_entry, name=f"{self.class_name}.{name}")

                if self.execution:
                    return attr(*args, **kwargs)

            return wrapper

        return attr
