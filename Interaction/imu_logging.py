"""Log periods for the FLS firmware's one-millisecond CRTP period byte.

cflib encodes ``period_in_ms / 10``. Both the deployed FLS firmware and the
frozen FLS SITL used for IMU validation consume that byte as milliseconds.
This is the same compensation used by the normal offboard log manager.
"""


def fls_log_config(name, period_ms):
    from cflib.crazyflie.log import LogConfig
    if type(period_ms) is not int or not 1 <= period_ms <= 255:
        raise ValueError('FLS telemetry period must be 1..255 milliseconds')
    return LogConfig(name=name, period_in_ms=period_ms * 10)
