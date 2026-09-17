"""Read-only Pi Vicon-KF velocity candidate for release-state experiments.

This does not transmit a packet or claim firmware capture-time alignment.
In particular, a Pi receive timestamp is not the firmware release epoch.
"""

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ViconVelocitySeedCandidate:
    position_m: tuple[float, float, float]
    velocity_m_s: tuple[float, float, float]
    velocity_std_m_s: tuple[float, float, float]
    pi_frame_receive_monotonic_s: float
    pi_release_receive_monotonic_s: float


def vicon_velocity_seed_candidate(
        log_manager, group_name, release_receive_monotonic_s,
        release_position_m):
    """Use only the latest KF state and its *matching* Pi-received frame.

    A fixed-dt Vicon KF reports model uncertainty, not independently verified
    physical velocity uncertainty. Apply a conservative floor and require
    recent, reasonably spaced Pi receive events. This is a diagnostic
    candidate, never a captured-time 15-state EKF update by itself.
    """
    frames = log_manager.groups.get(group_name, ())
    filters = log_manager.group_kfs.get(group_name, {})
    if len(frames) < 10 or any(axis not in filters for axis in 'xyz'):
        raise ValueError('Vicon KF has too few frames or missing axes')
    frame = frames[-1]
    release_time = float(release_receive_monotonic_s)
    frame_time = float(frame['host_receive_monotonic_s'])
    age = release_time - frame_time
    if not math.isfinite(age) or not 0.0 <= age <= 0.035:
        raise ValueError('latest Vicon frame is not fresh at Pi release receive')
    last_times = [float(item['host_receive_monotonic_s'])
                  for item in frames[-10:]]
    gaps = np.diff(last_times)
    if (not np.all(np.isfinite(gaps)) or
            np.any(gaps < 0.003) or np.any(gaps > 0.025)):
        raise ValueError('Pi-received Vicon frames do not form a regular window')
    position = np.asarray(frame['tvec'], dtype=float)
    velocity = np.asarray(frame['vel'], dtype=float)
    release_position = np.asarray(release_position_m, dtype=float)
    if (position.shape != (3,) or velocity.shape != (3,) or
            release_position.shape != (3,) or
            not np.all(np.isfinite(position)) or
            not np.all(np.isfinite(velocity)) or
            not np.all(np.isfinite(release_position)) or
            np.linalg.norm(position - release_position) > 0.10 or
            np.linalg.norm(velocity[:2]) > 1.5 or
            abs(velocity[2]) > 0.10):
        raise ValueError('Vicon velocity or position is invalid at release')
    variance = np.asarray([
        filters[axis].P[1, 1] for axis in 'xyz'
    ], dtype=float)
    latest_filter_velocity = np.asarray([
        filters[axis].x[1, 0] for axis in 'xyz'
    ], dtype=float)
    if (not np.all(np.isfinite(latest_filter_velocity)) or
            np.max(np.abs(latest_filter_velocity - velocity)) > 1e-5):
        raise ValueError('Vicon frame and KF covariance are not the same update')
    if (not np.all(np.isfinite(variance)) or
            np.any(variance < 0) or np.any(variance > 0.15 ** 2)):
        raise ValueError('Vicon velocity KF covariance is not usable')
    std = np.maximum(np.sqrt(variance), 0.08)
    return ViconVelocitySeedCandidate(
        position_m=tuple(position),
        velocity_m_s=tuple(velocity),
        velocity_std_m_s=tuple(std),
        pi_frame_receive_monotonic_s=frame_time,
        pi_release_receive_monotonic_s=release_time,
    )
