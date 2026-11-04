"""Offline handoff sensitivity diagnostic; no hardware access or flight claim.

Run as ``python -m Interaction.simulate_position_handoff``.
The fixed-target comparison is an idealized handoff baseline, not an entire
PB/OB/S-curve release implementation or a replay of recorded flight motion.
"""
import argparse
import itertools
import json
from pathlib import Path

import numpy as np

from Interaction.position_follow import (
    POSITION_KP, VELOCITY_KP, VELOCITY_LIMIT, SUPPRESSED_GAINS,
)
from Interaction.position_handoff import PositionHandoff


def simulate(tau, delay, speed, *, compensated):
    parameters = {**dict(zip(POSITION_KP, (1.9, 2.1))),
        **dict.fromkeys(VELOCITY_KP, 30.),
        **dict.fromkeys(VELOCITY_LIMIT, 2.),
        **dict.fromkeys(SUPPRESSED_GAINS, 0.)}
    handoff = PositionHandoff(parameters, {}, {})
    p = np.array([0., 0., 1.])
    v = np.array([speed, 0., 0.])
    acceleration = np.zeros(2)
    previous_rp = np.zeros(2)
    commands, positions, velocities = [], [0.], [speed]
    captured_at = error = None
    dt = .01
    for k in range(400):
        t = k * dt
        bias = np.array([.055 if t >= .1 else 0., 0., 0.])
        # The small-angle plant's attitude follows its actual acceleration,
        # allowing the controller's tilt/rate gates to operate independently.
        rp = -acceleration[::-1] / 9.81
        rates = (rp - previous_rp) / dt
        previous_rp = rp.copy()
        try:
            if compensated:
                target, status = handoff.update(position=p, velocity=v+bias,
                    attitude_rpy=[*rp, 0.], angular_velocity=[*rates, 0.],
                    motion_position=p, motion_velocity=v, timestamp=t,
                    motion_time=t, height=1., incoming_rp_deg=[0., 0.])
                if handoff.complete and captured_at is None:
                    captured_at = t
            else:
                target = np.array([0., 0., 1.])
        except (ValueError, RuntimeError) as exc:
            error = str(exc)
            break
        # Independent planar position/velocity P cascade, followed by command
        # transport delay and a first-order acceleration response.
        velocity_error = np.array([1.9, 2.1]) * (target[:2]-p[:2]) - (v+bias)[:2]
        commands.append(9.81 * np.radians(30.) * velocity_error)
        requested = commands[max(0, k-round(delay/dt))]
        acceleration += (requested-acceleration) * dt/tau
        v[:2] += acceleration*dt
        p[:2] += v[:2]*dt
        positions.append(float(p[0]))
        velocities.append(float(v[0]))
    positions = np.asarray(positions)
    return dict(response_tau_s=tau, command_delay_s=delay, initial_speed_m_s=speed,
        maximum_flyback_mm=float(np.max(np.maximum.accumulate(positions)-positions)*1000),
        peak_reverse_speed_mm_s=max(0., -min(velocities)*1000),
        terminal_speed_mm_s=float(np.linalg.norm(v[:2])*1000),
        terminal_tilt_deg=float(np.linalg.norm(acceleration)/9.81*180/np.pi),
        capture_s=captured_at, error=error)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path,
        default=Path('outputs/position_handoff_20261028/simulation.json'))
    args = parser.parse_args()
    cases = []
    for tau, delay, speed in itertools.product((.04, .08, .15), (0., .02, .05), (.01, .03, .1)):
        cases.append({'compensated': simulate(tau, delay, speed, compensated=True),
                      'fixed_target': simulate(tau, delay, speed, compensated=False)})
    report = dict(simulation_only=True, flight_validated=False,
        model='P-only FC cascade, first-order acceleration response, command delay, '
              '+55 mm/s FC velocity-bias step at 0.1 s; instantaneous state telemetry '
              'and ideal Vicon motion. No attitude-estimator jumps, contact, wind, or radio faults.',
        duration_s=4., dt_s=.01, cases=cases)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2)+'\n')
    print(json.dumps({'cases': len(cases), 'output': str(args.output),
        'capture_failures': sum(c['compensated']['error'] is not None or
            c['compensated']['capture_s'] is None for c in cases),
        'maximum_flyback_mm': max(c['compensated']['maximum_flyback_mm'] for c in cases),
        'maximum_terminal_speed_mm_s': max(c['compensated']['terminal_speed_mm_s'] for c in cases),
        'simulation_only': True, 'flight_validated': False}, indent=2))


if __name__ == '__main__':
    main()
