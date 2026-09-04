"""Offline arrival-clock and kinematic audit; never changes flight calibration.

python -m Interaction.analyze_mocap_timing LOG.json [OTHER.json ...] --output NEW_DIR

Old frame IDs are local wait-loop counters, not camera sequence IDs. Old frame
times are host times after waitForNextFrame returns, not capture timestamps.
"""
from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np


WRAP_MS = 1 << 24
WRAP_S = WRAP_MS / 1000.0
TIMING_FIELDS = (
    "wait_duration_s", "processing_duration_s", "callback_duration_s",
    "callback_count", "local_loop_interval_s",
    "processing_excluding_callbacks_s", "max_callback_duration_s",
    "between_processing_and_wait_s",
)
SEND_FIELDS = ("extpos_send_duration_s", "wait_return_to_send_s", "extpos_send_interval_s")
QUEUE_FIELDS = (
    "accepted_records", "written_records", "queued_records", "in_flight_records",
    "max_queue_depth", "queue_capacity", "overflow_errors", "writer_errors",
)


def number(value):
    return bool(not isinstance(value, bool) and isinstance(value, (int, float))
                and np.isfinite(value))


def load_records(path):
    """Recover only the complete JSON-array prefix after an interrupted write.

    Like orchestrator/analyze_interaction_logs.py, accept unterminated LiveLogger
    arrays. This standalone module does not require the other repository. Unlike
    line-by-line recovery, never skip a malformed middle record and hide a gap.
    """
    raw = Path(path).read_text(encoding="utf-8")
    try:
        parsed = json.loads(raw)
    except json.JSONDecodeError:
        parsed = None
    if parsed is not None:
        if not isinstance(parsed, list) or any(not isinstance(r, dict) for r in parsed):
            raise ValueError("expected a JSON array of log objects")
        return parsed, dict(valid_json=True, recovered=False, record_count=len(parsed))
    stripped = raw.lstrip()
    if not stripped.startswith("["):
        raise ValueError("expected a LiveLogger JSON array")
    decoder, records, cursor = json.JSONDecoder(), [], len(raw) - len(stripped) + 1
    while True:
        while cursor < len(raw) and raw[cursor].isspace():
            cursor += 1
        if cursor == len(raw):
            break
        try:
            record, end = decoder.raw_decode(raw, cursor)
        except json.JSONDecodeError:
            break
        if not isinstance(record, dict):
            break
        records.append(record)
        cursor = end
        while cursor < len(raw) and raw[cursor].isspace():
            cursor += 1
        if cursor < len(raw) and raw[cursor] == ",":
            cursor += 1
        else:
            break
    return records, dict(valid_json=False, recovered=True, record_count=len(records),
                         unparsed_suffix_characters=len(raw) - cursor,
                         note="Only the complete prefix was recovered; no completeness claim.")


def stats(values):
    values = np.asarray([float(v) for v in values if number(v)], dtype=float)
    if not len(values):
        return dict(available=False, count=0, minimum=None, mean=None,
                    p50=None, p95=None, p99=None, maximum=None)
    q = np.percentile(values, [0, 50, 95, 99, 100])
    return dict(available=True, count=len(values), minimum=float(q[0]),
                mean=float(np.mean(values)), p50=float(q[1]), p95=float(q[2]),
                p99=float(q[3]), maximum=float(q[4]))


def unwrap_cf(values):
    """Unwrap the raw 24-bit millisecond counter, never a host-time fit.

    Small backwards steps and duplicate timestamps remain visible rather than
    being relabeled as wraps. The caller must reject them for interpolation.
    """
    if any(not number(v) or int(v) != v or not 0 <= v < WRAP_MS for v in values):
        raise ValueError("missing or invalid raw 24-bit CF timestamp")
    raw = np.asarray(values, dtype=float)
    if not len(raw):
        return raw, 0
    wraps = np.r_[0, np.cumsum(np.diff(raw) < -WRAP_MS / 2)]
    return (raw + wraps * WRAP_MS) / 1000., int(wraps[-1])


def data_rows(records, record_type, group=None):
    return [r["data"] for r in records if r.get("type") == record_type
            and (group is None or r.get("group") == group)
            and isinstance(r.get("data"), dict)]


def timed_rows(rows):
    return sorted((r for r in rows if number(r.get("time"))), key=lambda r: r["time"])


def arrival_summary(rows, gap_threshold_s=.03, burst_threshold_s=.002):
    original_times = np.array([r["time"] for r in rows if number(r.get("time"))])
    original_nonpositive = int(np.sum(np.diff(original_times) <= 0))
    rows = timed_rows(rows)
    times = np.array([r["time"] for r in rows])
    dt = np.diff(times)
    gaps = [dict(start_time_s=float(times[i]), end_time_s=float(times[i+1]),
                 interval_s=float(dt[i]), previous_local_frame_id=rows[i].get("frame_id"),
                 next_local_frame_id=rows[i+1].get("frame_id"))
            for i in np.flatnonzero(dt > gap_threshold_s)]
    return dict(record_count=len(rows), interval_s=stats(dt),
                nonpositive_intervals=int(np.sum(dt <= 0)),
                original_record_order_nonpositive_intervals=original_nonpositive,
                sorting_note="Interval/gap distributions use host-time order; original record-order nonpositive steps are reported separately and can indicate clock jumps or record reordering.",
                gap_threshold_s=gap_threshold_s, gap_count=len(gaps),
                burst_threshold_s=burst_threshold_s,
                burst_count=int(np.sum((dt >= 0) & (dt < burst_threshold_s))), gaps=gaps)


def packet_summary(rows, gap_threshold_s, burst_threshold_s):
    result = arrival_summary(rows, gap_threshold_s, burst_threshold_s)
    rows = timed_rows(rows)
    result["cf_timestamp_present_count"] = sum(number(r.get("cf_timestamp_ms")) for r in rows)
    try:
        if not rows:
            raise ValueError("group unavailable")
        device, wraps = unwrap_cf([r.get("cf_timestamp_ms") for r in rows])
    except ValueError as error:
        result["device_clock"] = dict(available=False, reason=str(error))
    else:
        host = np.array([r["time"] for r in rows])
        result["device_clock"] = dict(available=True, wrap_count=wraps,
            interval_s=stats(np.diff(device)),
            nonpositive_intervals=int(np.sum(np.diff(device) <= 0)),
            host_minus_device_elapsed_s=stats((host-host[0])-(device-device[0])),
            note="Relative clock difference includes queueing and clock-rate drift; not absolute latency.")
    return result


def optional_timing_summary(rows, fields):
    return {field: dict(stats([r.get(field) for r in rows]),
                        missing_count=sum(not number(r.get(field)) for r in rows))
            for field in fields}


def brake_windows(records):
    """Extract complete brake/recovery pairs without requiring calibration save.

    Phase events precede actual commands in record order. Use their median clock
    difference when all endpoints are matched; otherwise explicitly use event
    times. Never fit clock alignment against position, velocity or attitude.
    """
    phases, offsets = {}, []
    for i, record in enumerate(records):
        if record.get("name") != "Planar Braking Calibration Phase":
            continue
        row = record.get("data", {})
        segment, phase = row.get("segment_id"), row.get("phase")
        if segment is None or phase not in ("brake", "recovery") or not number(row.get("time")):
            continue
        target = ("Commander.send_position_setpoint" if phase == "recovery"
                  else "Commander.send_zdistance_setpoint")
        command_time = None
        for candidate in records[i+1:i+101]:
            if candidate.get("type") == "wrench_observer" or candidate.get("name") == "Planar Braking Calibration Phase":
                break
            if candidate.get("name") == target:
                sent = candidate.get("data", {}).get("time")
                if number(sent):
                    command_time = float(sent)
                    offsets.append(float(row["time"])-command_time)
                break
        phases.setdefault(segment, {}).setdefault(phase, []).append((row, command_time))
    offset = float(np.median(offsets)) if offsets else None
    consistent = bool(offsets) and np.ptp(offsets) <= .01
    result = []
    for segment, pair in sorted(phases.items()):
        if any(len(pair.get(p, [])) != 1 for p in ("brake", "recovery")):
            result.append(dict(segment=segment, available=False, reason="missing or duplicate brake/recovery phase"))
            continue
        (brake, bc), (recovery, rc) = pair["brake"][0], pair["recovery"][0]
        direction = brake.get("direction_xy")
        if (not isinstance(direction, (list, tuple)) or len(direction) != 2
                or not all(number(v) for v in direction) or np.linalg.norm(direction) == 0):
            result.append(dict(segment=segment, available=False, reason="invalid trial direction"))
            continue
        use_command = consistent and bc is not None and rc is not None
        start = bc + offset if use_command else float(brake["time"])
        end = rc + offset if use_command else float(recovery["time"])
        if end <= start:
            result.append(dict(segment=segment, available=False, reason="nonpositive trial window"))
            continue
        result.append(dict(segment=segment, available=True, brake_time_s=start,
            recovery_time_s=end, direction_xy=(np.array(direction)/np.linalg.norm(direction)).tolist(),
            boundary_clock_method=("phase-event/next-command median offset" if use_command else "phase event host time (command alignment unavailable)")))
    return result


def interpolate_supported(targets, times, values, max_match_s):
    if len(times) < 2 or np.any(np.diff(times) < 0):
        raise ValueError("fewer than two monotonic position timestamps")
    if min(targets) < times[0] or max(targets) > times[-1]:
        raise ValueError("position group does not bracket velocity endpoints; no extrapolation")
    # A duplicate counter during unrelated startup is reported globally, but
    # must not invalidate a later strictly monotonic trial. Retain one support
    # packet on each side, and reject duplicates inside this actual window.
    first = max(0, int(np.searchsorted(times, targets[0], side="left"))-1)
    last = min(len(times), int(np.searchsorted(times, targets[-1], side="right"))+1)
    times, values = times[first:last], values[first:last]
    if np.any(np.diff(times) <= 0):
        raise ValueError("position timestamps in the supported window are not strictly increasing")
    right = np.searchsorted(times, targets, side="left")
    left = np.maximum(right-1, 0)
    nearest = np.minimum(abs(times[right]-targets), abs(times[left]-targets))
    if max(nearest) > max_match_s:
        raise ValueError("position/velocity packet matching exceeds maximum group skew")
    # Endpoint values cannot silently interpolate across an arbitrarily long gap.
    for j in (0, len(targets)-1):
        if times[right[j]] != targets[j] and times[right[j]]-times[left[j]] > 2*max_match_s:
            raise ValueError("position endpoint interpolation crosses a long packet gap")
    return np.interp(targets, times, values), nearest


def window_kinematics(window, positions, velocities, max_match_s=.03):
    result = dict(window)
    if not window["available"]:
        return result
    keys_p, keys_v = ("stateEstimate.x", "stateEstimate.y"), ("stateEstimate.vx", "stateEstimate.vy")
    pp = [r for r in timed_rows(positions) if all(number(r.get(k)) for k in keys_p)]
    vv = [r for r in timed_rows(velocities) if all(number(r.get(k)) for k in keys_v)]
    hv = np.array([r["time"] for r in vv])
    ids = np.flatnonzero((hv >= window["brake_time_s"]) & (hv < window["recovery_time_s"]))
    if len(ids) < 2 or not pp:
        result.update(available=False, reason="missing POS_ACC or fewer than two VEL_ORI packets in window")
        return result
    direction = np.array(window["direction_xy"])
    velocity_xy = np.array([[vv[i][k] for k in keys_v] for i in ids])
    velocity = velocity_xy[:, 0]*direction[0] + velocity_xy[:, 1]*direction[1]
    hp = np.array([r["time"] for r in pp])
    position_xy = np.array([[r[k] for k in keys_p] for r in pp])
    position = position_xy[:, 0]*direction[0] + position_xy[:, 1]*direction[1]
    host = hv[ids]
    result.update(sample_count=len(ids), first_packet_time_s=float(host[0]),
                  last_packet_time_s=float(host[-1]), host_interval_s=stats(np.diff(host)))

    def compare(times, position_times):
        if np.any(np.diff(times) <= 0):
            raise ValueError("velocity timestamps are not strictly increasing")
        projected, nearest = interpolate_supported(times, position_times, position, max_match_s)
        integral = float(np.sum(.5*(velocity[1:]+velocity[:-1])*np.diff(times)))
        displacement = float(projected[-1]-projected[0])
        return dict(available=True, elapsed_s=float(times[-1]-times[0]),
                    velocity_integral_m=integral, position_displacement_m=displacement,
                    integral_minus_displacement_m=integral-displacement,
                    nearest_position_packet_skew_s=stats(nearest))

    try:
        result["host_clock"] = compare(host, hp)
    except ValueError as error:
        result["host_clock"] = dict(available=False, reason=str(error))
    try:
        # Full streams preserve wrap continuity even if the selected trial is
        # shortly after a wrap. Shift only by a whole 24-bit epoch when one
        # group starts after the other crossed a wrap, never fit a fine lag.
        cv, _ = unwrap_cf([r.get("cf_timestamp_ms") for r in vv])
        cp, _ = unwrap_cf([r.get("cf_timestamp_ms") for r in pp])
        nearest = int(np.argmin(abs(hv-hp[0])))
        cp = cp + round((cv[nearest]-cp[0])/WRAP_S)*WRAP_S
        result["device_clock"] = compare(cv[ids], cp)
        result["device_clock"]["same_velocity_packet_endpoints_as_host"] = True
    except ValueError as error:
        result["device_clock"] = dict(available=False, reason=str(error))
    if result["host_clock"]["available"] and result["device_clock"]["available"]:
        result["device_minus_host_residual_m"] = (
            result["device_clock"]["integral_minus_displacement_m"]
            - result["host_clock"]["integral_minus_displacement_m"])
    return result


def analyze(records, *, gap_threshold_s=.03, burst_threshold_s=.002):
    frames = data_rows(records, "frames")
    groups = sorted({r.get("group") for r in records if r.get("type") == "state"
                     and isinstance(r.get("group"), str)})
    packets = {g: data_rows(records, "state", g) for g in groups}
    timing = data_rows(records, "mocap_timing")
    send = [f["mocap_timing"] for f in frames if isinstance(f.get("mocap_timing"), dict)]
    queues = [r["logger_queue"] for r in timing if isinstance(r.get("logger_queue"), dict)]
    queue_fields = optional_timing_summary(queues, QUEUE_FIELDS)
    for field, summary in queue_fields.items():
        summary["last_observed"] = next((r[field] for r in reversed(queues)
                                         if number(r.get(field))), None)
    windows = [window_kinematics(w, packets.get("POS_ACC", []), packets.get("VEL_ORI", []))
               for w in brake_windows(records)]
    for row in windows:
        if "brake_time_s" in row:
            selected = [f for f in frames if number(f.get("time"))
                        and row["brake_time_s"] <= f["time"] < row["recovery_time_s"]]
            row["external_frame_arrivals"] = arrival_summary(selected, gap_threshold_s, burst_threshold_s)
    return dict(offline_only=True, frame_arrivals=arrival_summary(frames, gap_threshold_s, burst_threshold_s),
        packet_groups={g: packet_summary(p, gap_threshold_s, burst_threshold_s) for g, p in packets.items()},
        brake_windows=windows,
        optional_instrumentation=dict(mocap_timing_record_count=len(timing),
            timing_fields=optional_timing_summary(timing, TIMING_FIELDS),
            frame_send_metadata_count=len(send), frames_without_send_metadata=len(frames)-len(send),
            send_fields=optional_timing_summary(send, SEND_FIELDS),
            logger_queue_snapshot_count=len(queues), logger_queue_fields=queue_fields,
            logger_queue_note="Snapshots are sampled, not every enqueue; last_observed need not be the final shutdown counter. Missing counters remain unavailable."),
        caveats=[
            "frame_id is a local wait-loop counter, not a native camera sequence ID; continuity does not prove no capture loss.",
            "frames.time is host time after waitForNextFrame returned, not a camera capture timestamp.",
            "Missing optional timing fields are unavailable, not zero latency.",
            "Clock comparison uses identical velocity packet endpoints, 24-bit unwrap and only integer-wrap epoch alignment; no motion-derived lag or clock correction.",
            "Integral-minus-position residual can reflect estimator corrections and velocity bias as well as timing; correlation is not proof of root cause.",
            "This report does not change flight control, fit a model, certify a flight, or overwrite calibration.",
        ])


def markdown(report):
    lines = ["# Mocap arrival timing and kinematic consistency", "",
             "Offline audit. Arrival gaps are not automatically camera frame losses or measured capture latency.", ""]
    for run in report["runs"]:
        lines += [f"## {Path(run['input_path']).stem}", "",
                  f"Input recovered: `{run['integrity']['recovered']}`; SHA-256: `{run['sha256']}`.", ""]
        arrival = run["frame_arrivals"]
        lines += [f"External frame arrivals: {arrival['record_count']} records; {arrival['gap_count']} gaps above {1000*arrival['gap_threshold_s']:.0f} ms; {arrival['burst_count']} intervals below {1000*arrival['burst_threshold_s']:.0f} ms.", "",
                  "| Trial (zero-based) | Host ∫v−Δp (cm) | CF ∫v−Δp (cm) | Note |",
                  "|---|---:|---:|---|"]
        for window in run["brake_windows"]:
            cells = []
            reasons = []
            for name in ("host_clock", "device_clock"):
                clock = window.get(name, {})
                cells.append(f"{100*clock['integral_minus_displacement_m']:.4f}" if clock.get("available") else "unavailable")
                if clock.get("reason"):
                    reasons.append(clock["reason"])
            if window.get("reason"):
                reasons.append(window["reason"])
            lines.append(f"| {window['segment']} | {cells[0]} | {cells[1]} | {'; '.join(dict.fromkeys(reasons)) or 'Same velocity packet endpoints; before recovery'} |")
        instrument = run["optional_instrumentation"]
        lines += ["", f"Optional mocap timing records: {instrument['mocap_timing_record_count']}; frames with send timing metadata: {instrument['frame_send_metadata_count']}.",
                  "Missing metadata remains unavailable. Full interval distributions, gap timestamps and optional duration statistics are in report.json.", ""]
    lines += ["## Interpretation limits", ""] + [f"- {c}" for c in report["runs"][0]["caveats"]]
    return "\n".join(lines) + "\n"


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", type=Path, nargs="+")
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("output directory already exists; choose a new directory")
    runs = []
    for path in args.logs:
        records, integrity = load_records(path)
        run = analyze(records)
        run.update(input_path=str(path.resolve()), integrity=integrity,
                   sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        runs.append(run)
    report = dict(runs=runs, offline_only=True)
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False)+"\n", encoding="utf-8")
    (args.output / "README.md").write_text(markdown(report), encoding="utf-8")
    print(args.output / "README.md")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
