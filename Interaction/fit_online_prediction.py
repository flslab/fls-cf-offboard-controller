"""Replay calibration in time order through the online background fitter.

This is an offline diagnostic command: no radio, serial connection, flight
command or active calibration-file write. Output must be a new directory.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import time

from Interaction.braking_replay import extract
from Interaction.online_prediction_calibration import OnlinePredictionCalibration


def samples_from_log(records, max_sample_gap_s=.06):
    """Use executed phase commands, not the observer row's next command.

    Historical command and state clocks are aligned by the existing extractor;
    their approximate send anchors are recorded in provenance. Duplicate polls
    retain the first observation exactly as in the underlying extractor.
    """
    _fit, trials, samples, metadata = extract(records, max_gap_s=max_sample_gap_s)
    by_time = {}
    for record in records:
        if record.get('type') == 'wrench_observer':
            row = record['data']
            by_time.setdefault(float(row['state_time']), row)
    for sample in samples:
        row = by_time[sample['timestamp']]
        sample.update(
            actual_attitude_rpy_rad=row['orientation_rpy_rad'],
            angular_velocity_rad_s=row['angular_velocity_rad_s'],
            position_xy=row['position_m'][:2],
            velocity_xy=row['velocity_m_s'][:2],
            battery_voltage_V=row['battery_voltage_V'],
            state_group_skew_s=row['state_group_skew_s'],
        )
    return samples, [trial.segment for trial in trials], metadata


def summary_markdown(report):
    lines = ['# Sequential online-model replay', '',
             'Offline replay only; no flight commands or active calibration updates.', '',
             f"Status: {report['status']}; complete data: {report['data_complete']}; "
             f"held-out diagnostic gates passed: {report['validation_passed']}.", '',
             'The last all-data refit is NOT independently validated. '
             'Only the previous frozen model is scored on the final held-out pair.', '',
             '| Version | Training segments | Validation segments | Passed | +Y margin (m/s) | -Y margin (m/s) |',
             '|---|---|---|---|---|---|']
    for value in report.get('validation_history', []):
        margins = value.get('terminal_velocity_error_margins_m_s') or {}
        lines.append(f"| {value['candidate_version']} | {value['training_segment_ids']} "
                     f"| {value['validation_segment_ids']} | {value['validation_passed']} "
                     f"| {margins.get('positive_y', 'n/a')} "
                     f"| {margins.get('negative_y', 'n/a')} |")
    candidate = report.get('candidate') or {}
    if candidate:
        lines += [
            '', '## Next-pair control eligibility', '',
            f"Eligible: `{candidate.get('control_eligible', False)}`; reason: "
            f"`{candidate.get('control_eligibility_reason', 'not_reported')}`.",
        ]
    validated = report.get('validated_candidate') or {}
    model = validated.get('model') or {}
    if model:
        lines += ['', '## Last frozen model', '', '```json',
                  json.dumps({key: model.get(key) for key in (
                      'attitude_fit', 'motion_gain', 'identifiability',
                      'directional_models')}, indent=2), '```']
    metrics = validated.get('metrics') or {}
    if metrics.get('per_trial'):
        lines += ['', '## Final held-out trials', '',
                  '| Segment | Tilt RMSE (deg) | Velocity RMSE (m/s) | Terminal velocity error (m/s) | Endpoint error (m) | Actual / predicted reverse |',
                  '|---|---|---|---|---|---|']
        for row in metrics['per_trial']:
            lines.append(f"| {row['segment_id']} | {row['theta_rmse_deg']:.4f} | "
                         f"{row['velocity_rmse_m_s']:.4f} | {row['terminal_error_m_s']:.4f} | "
                         f"{row['end_position_error_m']:.4f} | "
                         f"{row['actual_reverse']} / {row['predicted_reverse']} |")
    lines += ['', '## Interpretation limits', '',
              '- Only the tested world-Y attitude-command response is identified.',
              '- Delay is effective host-clock delay, including telemetry/scheduling.',
              '- Evaluation is conditional on the executed command schedule; no future measured states initialize forecasts.',
              '- Position-controller capture, X/Z response, payload/battery extrapolation and reliable stopping are not validated.',
              '- Models are diagnostic only: runtime_enabled=false and flight_approved=false.', '',
              'Full provenance, raw samples, gate failures and all intermediate candidates are in report.json and report.samples.jsonl.', '']
    return '\n'.join(lines)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--output', type=Path, required=True, help='new directory only')
    parser.add_argument('--drone-id', default='lb11')
    parser.add_argument('--timeout-s', type=float, default=180.)
    args = parser.parse_args(argv)
    if not 0 < args.timeout_s <= 3600:
        parser.error('--timeout-s must be in (0, 3600]')
    source = args.log.resolve()
    source_bytes = source.read_bytes()
    samples, segments, metadata = samples_from_log(json.loads(source_bytes))
    if len(segments) < 4 or len(segments) % 2:
        parser.error('at least two complete opposed pairs are required')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    metadata.update(
        mode='offline_sequential_replay', source_path=str(source),
        source_sha256=hashlib.sha256(source_bytes).hexdigest(),
        clock_scope='host_receive_effective_delay_with_reconstructed_send_anchors',
        prediction_scope='attitude_command_only',
        completion_certification='logged_actual_recovery_commands',
        position_capture_model_identified=False,
    )
    session = OnlinePredictionCalibration({}, output / 'report.json', args.drone_id,
                                          segments, metadata=metadata)
    started = time.monotonic()
    report = None
    try:
        if not session.start():
            raise RuntimeError('background fitter could not start')
        for segment in segments:
            if not session.submit_trial([s for s in samples if s['segment_id'] == segment]):
                raise RuntimeError(f'failed to queue segment {segment}; output remains diagnostic')
        finish_requested = session.request_finish()
        while time.monotonic() - started < args.timeout_s:
            for event in session.poll():
                print(event['event'], json.dumps(event['data']), flush=True)
            report = session.latest_report
            if report and report['status'] in ('completed', 'partial', 'failed'):
                break
            if not finish_requested:
                finish_requested = session.request_finish()
            time.sleep(.05)  # Offline CLI only; flight loop never waits here.
        else:
            raise TimeoutError('offline fitting timed out; partial artifacts retained')
    finally:
        session.close()
    (output / 'summary.md').write_text(summary_markdown(report), encoding='utf-8')
    print(f"Report: {output / 'report.json'}\nSummary: {output / 'summary.md'}", flush=True)
    return 0 if report['status'] == 'completed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
