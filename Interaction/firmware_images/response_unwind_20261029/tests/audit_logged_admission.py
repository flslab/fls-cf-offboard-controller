"""Approximate initial admission replay; no hardware access or calibration writes.

The C probe uses the verified Oct 7 lb11 response model. Input is the saved
flyback diagnostic source.json and summary.json, with FC clock timestamps.
50 Hz commands cannot reproduce the full on-FC 500 Hz command queue.
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

import numpy as np
from Interaction.compressed_state_logs import decode_kinematic_packet


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--diagnostic', type=Path, required=True)
    parser.add_argument('--probe', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    source = json.loads((args.diagnostic / 'source.json').read_text())
    path = Path(source['source'])
    assert hashlib.sha256(path.read_bytes()).hexdigest() == source['sha256']
    model = json.loads((args.diagnostic / 'pi_attitude_response.json').read_text())
    axes = model['drones']['lb11']['accepted']['report']['axes']
    names = ['delay_s', 'wn_rad_s', 'zeta', 'gain', 'bias_deg']
    expected = {'pitch': [.013257, 11.878785, .448143, .923565, -.018168],
                'roll': [.044451, 21.922344, 1.114356, 1.006446, .000689]}
    for name in expected:
        assert np.allclose([axes[name][key] for key in names], expected[name], atol=1e-7)
    records = json.loads(path.read_text())
    overview = json.loads((args.diagnostic / 'summary.json').read_text())
    def rows(group):
        return [r['data'] for r in records if r.get('group') == group]
    kin = rows('FIRMWARE_KIN')
    kt = np.array([r['cf_timestamp_ms'] * .001 for r in kin])
    decoded = [decode_kinematic_packet(r) for r in kin]
    angles = np.rad2deg([r[2] for r in decoded])
    rates = np.rad2deg([r[3] for r in decoded])
    velocity = np.array([r[1] for r in decoded])
    phi, theta = np.deg2rad(angles[:, 0]), -np.deg2rad(angles[:, 1])
    p, q, r = rates[:, 0], -rates[:, 1], rates[:, 2]
    euler = np.c_[p + np.sin(phi)*np.tan(theta)*q + np.cos(phi)*np.tan(theta)*r,
                  -np.cos(phi)*q + np.sin(phi)*r]
    commands = rows('POSITION_HANDOFF')
    ct = np.array([r['cf_timestamp_ms'] * .001 for r in commands])
    command = np.array([[r['controller.pitch'], r['controller.roll']] for r in commands])
    auto = rows('AUTO_UNWIND')
    inputs, report = [], []
    for item in overview['interactions'][:4]:
        t = next(r['cf_timestamp_ms'] * .001 for r in auto
                 if item['start_s'] + source['origin'] < r['time'] < item['end_s'] + source['origin']
                 and r['hlCommander.scUnwPhase'] == 1)
        ix = max(0, np.searchsorted(kt, t, side='right') - 1)
        ci = max(0, np.searchsorted(ct, t, side='right') - 1)
        yaw = np.deg2rad(angles[ix, 2])
        co, si = np.cos(yaw), np.sin(yaw)
        rotation = np.array([[co, si], [-si, co]])
        v = rotation @ velocity[ix, :2]
        direction = rotation @ np.asarray(item['direction_xy'])
        angle, rate = angles[ix, [1, 0]], euler[ix, [1, 0]]
        times = t - .15 + np.arange(76)*.002
        indices = np.clip(np.searchsorted(ct, times, side='right') - 1, 0, len(ct)-1)
        history = command[indices].T
        inputs.append(' '.join(map(str, np.r_[v, angle, rate, command[ci], direction, angle, history.ravel()])))
        report.append(dict(interaction=item['interaction'], cf_s=t,
                           signed_velocity_m_s=float(direction @ v),
                           angle_pitch_roll_deg=angle.tolist(),
                           rate_pitch_roll_deg_s=rate.tolist(), sample_age_s=float(t-kt[ix])))
    args.output.with_suffix('.inputs.txt').write_text('\n'.join(inputs) + '\n')
    proc = subprocess.run([str(args.probe.resolve()), 'probe'], input='\n'.join(inputs)+'\n',
                          capture_output=True, text=True, check=True)
    for item, line in zip(report, proc.stdout.splitlines()):
        ok, reason, minimum = line.split()
        value = float(minimum)
        item.update(admitted=bool(int(ok)), reason=int(reason),
                    predicted_minimum_m_s=value if np.isfinite(value) else None)
    assert len(proc.stdout.splitlines()) == len(report)
    result = dict(source=source, interactions=report, limitations=[
        'Admission replay only, not a closed-loop flight counterfactual.',
        '50 Hz sampled commands; exact on-FC 500 Hz issued-command history unavailable.',
        'Uses nearest previous compressed FC state; online residual correction not reconstructed.'])
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps(report, indent=2, allow_nan=False))


if __name__ == '__main__':
    main()
