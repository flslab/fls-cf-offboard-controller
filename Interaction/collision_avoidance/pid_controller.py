"""
Cascaded PID controller emulating Crazyflie drone dynamics.

Cascade structure used in this simulation:
  Velocity PID  -> desired attitude / vertical acceleration
  Attitude PID  -> desired angular rate
  Rate PID      -> angular acceleration
  Attitude integration -> translational acceleration -> velocity integration

This adds the missing attitude dynamics so the drone does not jump from hover
to maximum horizontal acceleration in a single timestep. The default gains are
loaded from the repository's root ``config.py``.
"""

import numpy as np

from config import PID_VALUES as PID_PARAMS_DEFAULT


class PIDAxis:
    """Single-axis PID controller with integral anti-windup."""

    def __init__(
        self,
        kp: float,
        ki: float,
        kd: float,
        output_limit: float = None,
        i_limit: float = None,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limit = output_limit
        self.i_limit = i_limit or (output_limit * 2.0 if output_limit else 100.0)
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialised = False

    def reset(self):
        self._integral = 0.0
        self._prev_error = 0.0
        self._initialised = False

    def update(self, error: float, dt: float) -> float:
        p = self.kp * error

        self._integral += error * dt
        self._integral = np.clip(self._integral, -self.i_limit, self.i_limit)
        i = self.ki * self._integral

        if not self._initialised:
            d = 0.0
            self._initialised = True
        else:
            d = self.kd * (error - self._prev_error) / dt if dt > 0 else 0.0
        self._prev_error = error

        out = p + i + d
        if self.output_limit is not None:
            out = float(np.clip(out, -self.output_limit, self.output_limit))
        return out


_G = 9.81
_DEG2RAD = np.pi / 180.0
_RAD2DEG = 180.0 / np.pi


class CascadedPIDController:
    """
    3-axis velocity-tracking controller with attitude dynamics.

    Horizontal motion:
      velocity error -> desired pitch/roll angle (deg)
      angle error    -> desired pitch/roll rate (deg/s)
      rate error     -> angular acceleration (deg/s^2)
      integrated attitude -> horizontal acceleration via gravity

    Vertical motion:
      velocity PID directly commands vertical acceleration.
    """

    def __init__(
        self,
        pid_params: dict = None,
        v_max: float = 1.0,
        a_max: float = 1.0,
        tilt_limit_deg: float = 20.0,
        rate_limit_deg_s: float = 720.0,
        angular_accel_limit_deg_s2: float = 4000.0,
    ):
        if pid_params is None:
            pid_params = PID_PARAMS_DEFAULT

        self.v_max = v_max
        self.a_max = a_max
        self.tilt_limit_deg = tilt_limit_deg
        self.rate_limit_deg_s = rate_limit_deg_s
        self.angular_accel_limit_deg_s2 = angular_accel_limit_deg_s2

        self.vel_pid_xy = [
            PIDAxis(
                float(pid_params["velCtlPid.vxKp"]),
                float(pid_params["velCtlPid.vxKi"]),
                float(pid_params["velCtlPid.vxKd"]),
                output_limit=tilt_limit_deg,
            ),
            PIDAxis(
                float(pid_params["velCtlPid.vyKp"]),
                float(pid_params["velCtlPid.vyKi"]),
                float(pid_params["velCtlPid.vyKd"]),
                output_limit=tilt_limit_deg,
            ),
        ]
        self.vel_pid_z = PIDAxis(
            float(pid_params["velCtlPid.vzKp"]),
            float(pid_params["velCtlPid.vzKi"]),
            float(pid_params["velCtlPid.vzKd"]),
            output_limit=a_max,
        )

        self.att_pid = {
            "roll": PIDAxis(
                float(pid_params["pid_attitude.roll_kp"]),
                float(pid_params["pid_attitude.roll_ki"]),
                float(pid_params["pid_attitude.roll_kd"]),
                output_limit=rate_limit_deg_s,
            ),
            "pitch": PIDAxis(
                float(pid_params["pid_attitude.pitch_kp"]),
                float(pid_params["pid_attitude.pitch_ki"]),
                float(pid_params["pid_attitude.pitch_kd"]),
                output_limit=rate_limit_deg_s,
            ),
        }

        self.rate_pid = {
            "roll": PIDAxis(
                float(pid_params["pid_rate.roll_kp"]),
                float(pid_params["pid_rate.roll_ki"]),
                float(pid_params["pid_rate.roll_kd"]),
                output_limit=angular_accel_limit_deg_s2,
            ),
            "pitch": PIDAxis(
                float(pid_params["pid_rate.pitch_kp"]),
                float(pid_params["pid_rate.pitch_ki"]),
                float(pid_params["pid_rate.pitch_kd"]),
                output_limit=angular_accel_limit_deg_s2,
            ),
        }

        self.velocity = np.zeros(3)
        self.attitude_deg = {"roll": 0.0, "pitch": 0.0}
        self.attitude_rate_deg_s = {"roll": 0.0, "pitch": 0.0}

    def reset(self, initial_velocity: np.ndarray = None):
        for pid in self.vel_pid_xy:
            pid.reset()
        self.vel_pid_z.reset()
        for pid in self.att_pid.values():
            pid.reset()
        for pid in self.rate_pid.values():
            pid.reset()

        self.velocity = (
            np.zeros(3) if initial_velocity is None else initial_velocity.copy()
        )
        self.attitude_deg = {"roll": 0.0, "pitch": 0.0}
        self.attitude_rate_deg_s = {"roll": 0.0, "pitch": 0.0}

    def _step_attitude_axis(self, axis: str, desired_angle_deg: float, dt: float) -> float:
        desired_rate_deg_s = self.att_pid[axis].update(
            desired_angle_deg - self.attitude_deg[axis], dt
        )
        desired_rate_deg_s = float(
            np.clip(desired_rate_deg_s, -self.rate_limit_deg_s, self.rate_limit_deg_s)
        )

        angular_accel_deg_s2 = self.rate_pid[axis].update(
            desired_rate_deg_s - self.attitude_rate_deg_s[axis], dt
        )
        angular_accel_deg_s2 = float(
            np.clip(
                angular_accel_deg_s2,
                -self.angular_accel_limit_deg_s2,
                self.angular_accel_limit_deg_s2,
            )
        )

        self.attitude_rate_deg_s[axis] += angular_accel_deg_s2 * dt
        self.attitude_rate_deg_s[axis] = float(
            np.clip(
                self.attitude_rate_deg_s[axis],
                -self.rate_limit_deg_s,
                self.rate_limit_deg_s,
            )
        )

        self.attitude_deg[axis] += self.attitude_rate_deg_s[axis] * dt
        self.attitude_deg[axis] = float(
            np.clip(
                self.attitude_deg[axis],
                -self.tilt_limit_deg,
                self.tilt_limit_deg,
            )
        )
        return self.attitude_deg[axis]

    def step_position(
        self,
        pos_setpoint: np.ndarray,
        current_pos: np.ndarray,
        dt: float,
        pos_kp: float = 2.0,
    ) -> np.ndarray:
        """
        Track *pos_setpoint* using an outer position P loop feeding the
        velocity/attitude/rate cascade.

        The desired velocity is:
            v_des = pos_kp * (pos_setpoint - current_pos)
        clipped to v_max before entering the inner cascade.

        Returns the position delta (Δp) for this timestep.
        """
        pos_error = np.asarray(pos_setpoint, dtype=float) - np.asarray(current_pos, dtype=float)
        v_des = pos_kp * pos_error
        speed = float(np.linalg.norm(v_des))
        if speed > self.v_max:
            v_des = v_des * (self.v_max / speed)
        return self.step_velocity(v_des, dt)

    def step_velocity(self, vel_setpoint: np.ndarray, dt: float) -> np.ndarray:
        """
        Track *vel_setpoint* with cascaded velocity/attitude/rate dynamics.

        Returns the position delta (Δp) for this timestep.
        """
        vel_setpoint = np.asarray(vel_setpoint, dtype=float)

        vel_error = vel_setpoint - self.velocity
        desired_pitch_deg = self.vel_pid_xy[0].update(float(vel_error[0]), dt)
        desired_roll_deg = self.vel_pid_xy[1].update(float(vel_error[1]), dt)
        desired_vz_acc = self.vel_pid_z.update(float(vel_error[2]), dt)

        desired_pitch_deg = float(
            np.clip(desired_pitch_deg, -self.tilt_limit_deg, self.tilt_limit_deg)
        )
        desired_roll_deg = float(
            np.clip(desired_roll_deg, -self.tilt_limit_deg, self.tilt_limit_deg)
        )

        pitch_deg = self._step_attitude_axis("pitch", desired_pitch_deg, dt)
        roll_deg = self._step_attitude_axis("roll", desired_roll_deg, dt)

        pitch_rad = pitch_deg * _DEG2RAD
        roll_rad = roll_deg * _DEG2RAD

        # Small-angle translational model around hover:
        # pitch drives x acceleration, roll drives y acceleration.
        acc = np.array([
            _G * np.sin(pitch_rad),
            _G * np.sin(roll_rad),
            desired_vz_acc,
        ])

        acc_mag = float(np.linalg.norm(acc))
        if acc_mag > self.a_max:
            acc = acc * self.a_max / acc_mag

        self.velocity = self.velocity + acc * dt

        speed = float(np.linalg.norm(self.velocity))
        if speed > self.v_max:
            self.velocity = self.velocity * self.v_max / speed

        return self.velocity * dt

    @property
    def debug_state(self) -> dict:
        """Expose internal state for debugging or future logging."""
        return {
            "velocity": self.velocity.copy(),
            "roll_deg": self.attitude_deg["roll"],
            "pitch_deg": self.attitude_deg["pitch"],
            "roll_rate_deg_s": self.attitude_rate_deg_s["roll"],
            "pitch_rate_deg_s": self.attitude_rate_deg_s["pitch"],
        }
