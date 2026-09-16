"""Prop-off, read-only calibration of Bolt microsecond ticks per Pi microsecond.

Each probe brackets the *firmware receive* time between Pi send and Pi echo
receipt. A long-baseline pair bounds clock rate without assuming a symmetric
radio path or synchronizing clock epochs. Results are diagnostic only.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
import sys
import time
from pathlib import Path

from Interaction.calibrate_cf_radio_clock import (
    MICROS_MODULUS,
    collect_probes,
)


def _number(value, name):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be finite")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(f"{name} must be finite")
    return value


def session_rate_interval(document, *, warmup_probes=1,
                          minimum_baseline_s=1.0):
    """Causal [lower, upper] rate interval in Bolt us / Pi us."""
    if document.get("schema") != "bolt_radio_clock_brackets_v1":
        raise ValueError("unsupported Bolt clock-probe schema")
    rows = document.get("probes")
    if (not isinstance(rows, list) or len(rows) < warmup_probes + 2
            or type(warmup_probes) is not int or warmup_probes < 0):
        raise ValueError("insufficient ordered clock probes")
    raw = [row["bolt_receive_us_mod32"] for row in rows]
    if any(type(value) is not int or not 0 <= value < MICROS_MODULUS
           for value in raw):
        raise ValueError("invalid firmware receive counter")
    firmware_delta_us = 0
    for before, after in zip(raw[warmup_probes:-1],
                             raw[warmup_probes + 1:]):
        advance = (after - before) % MICROS_MODULUS
        if not 0 < advance < MICROS_MODULUS // 2:
            raise ValueError("firmware clock reset or reordered probes")
        firmware_delta_us += advance
    first, last = rows[warmup_probes], rows[-1]
    send_first = _number(first["host_send_start_monotonic_ns"], "host send") / 1000
    reply_first = _number(first["host_reply_end_monotonic_ns"], "host reply") / 1000
    send_last = _number(last["host_send_start_monotonic_ns"], "host send") / 1000
    reply_last = _number(last["host_reply_end_monotonic_ns"], "host reply") / 1000
    minimum_host_delta_us = send_last - reply_first
    maximum_host_delta_us = reply_last - send_first
    if (not send_first < reply_first < send_last < reply_last
            or minimum_host_delta_us < minimum_baseline_s * 1e6):
        raise ValueError("clock probe baseline too short or out of order")
    lower = firmware_delta_us / maximum_host_delta_us
    upper = firmware_delta_us / minimum_host_delta_us
    return {
        "ratio_lower": lower,
        "ratio_upper": upper,
        "ratio_midpoint": (lower + upper) / 2,
        "firmware_elapsed_us": firmware_delta_us,
        "host_baseline_min_ms": minimum_host_delta_us / 1000,
        "host_baseline_max_ms": maximum_host_delta_us / 1000,
        "warmup_probes_excluded": warmup_probes,
        "probe_count": len(rows),
    }


def fit_clock_rate(sessions, *, minimum_sessions=5):
    """Observed rate envelope plus an independent last-session check."""
    if type(minimum_sessions) is not int or minimum_sessions < 3:
        raise ValueError("minimum_sessions must be at least 3")
    bounds = [session_rate_interval(item) for item in sessions]
    if len(bounds) < minimum_sessions:
        raise ValueError(f"at least {minimum_sessions} probe sessions required")
    training = bounds[:-1]
    holdout = bounds[-1]
    training_lower = min(row["ratio_lower"] for row in training)
    training_upper = max(row["ratio_upper"] for row in training)
    holdout_compatible = (
        holdout["ratio_lower"] <= training_upper
        and holdout["ratio_upper"] >= training_lower
    )
    # Include the holdout when reporting the observed envelope, even when
    # it disproves a stable training calibration.
    lower = min(row["ratio_lower"] for row in bounds)
    upper = max(row["ratio_upper"] for row in bounds)
    return {
        "schema": "bolt_to_pi_clock_rate_calibration_v1",
        "time_basis": "firmware_microseconds_per_pi_monotonic_microsecond",
        "ratio_midpoint": (lower + upper) / 2,
        "ratio_lower": lower,
        "ratio_upper": upper,
        "ratio_empirical_uncertainty": (upper - lower) / 2,
        "session_count": len(bounds),
        "training_session_count": len(training),
        "holdout_compatible": holdout_compatible,
        "median_session_ratio": statistics.median(
            row["ratio_midpoint"] for row in bounds
        ),
        "sessions": bounds,
        "release_epoch_calibrated": False,
        "position_transport_calibrated": False,
        "command_authority": False,
    }


def release_delay_in_firmware_clock(release_fit, clock_fit):
    """Convert a Pi-time causal release-delay envelope to Bolt ticks."""
    if release_fit.get("schema") != "post_release_fixed_transport_delay_calibration_v1":
        raise ValueError("release delay calibration schema invalid")
    if clock_fit.get("schema") != "bolt_to_pi_clock_rate_calibration_v1":
        raise ValueError("firmware clock-rate calibration schema invalid")
    release_lower = _number(release_fit["observed_lower_ms"], "release lower")
    release_upper = _number(release_fit["observed_upper_ms"], "release upper")
    ratio_lower = _number(clock_fit["ratio_lower"], "clock lower")
    ratio_upper = _number(clock_fit["ratio_upper"], "clock upper")
    if not 0 <= release_lower <= release_upper or not 0 < ratio_lower <= ratio_upper:
        raise ValueError("calibration bounds invalid")
    lower = release_lower * ratio_lower
    upper = release_upper * ratio_upper
    return {
        "schema": "post_release_fixed_firmware_clock_delay_shadow_v1",
        "fixed_delay_firmware_ms": (lower + upper) / 2,
        "empirical_uncertainty_firmware_ms": (upper - lower) / 2,
        "observed_lower_firmware_ms": lower,
        "observed_upper_firmware_ms": upper,
        "release_holdout_covered": release_fit["holdout_covered"],
        "clock_holdout_compatible": clock_fit["holdout_compatible"],
        "position_transport_calibrated": False,
        "command_authority": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("documents", nargs="*", type=Path,
                        help="prior clock-probe JSON or batch JSON")
    parser.add_argument("--collect", type=int, default=0,
                        help="run N read-only probe sessions on this Pi")
    parser.add_argument("--uri", default="usb://0")
    parser.add_argument("--probes-per-session", type=int, default=32)
    parser.add_argument("--between-session-s", type=float, default=1.0)
    parser.add_argument("--release-batch", type=Path)
    args = parser.parse_args()
    if args.collect and (args.collect < 5 or args.collect > 20):
        parser.error("--collect must be 5..20 sessions")
    if args.collect and args.documents:
        parser.error("choose --collect or input documents")
    if args.between_session_s < 0:
        parser.error("--between-session-s must be nonnegative")
    sessions = []
    for path in args.documents:
        data = json.loads(path.read_text())
        if data.get("schema") == "bolt_clock_rate_prop_off_batch_v1":
            sessions.extend(data["probe_sessions"])
        else:
            sessions.append(data)
    for index in range(args.collect):
        print(f"Read-only clock probe {index + 1}/{args.collect}",
              file=sys.stderr, flush=True)
        try:
            sessions.append(collect_probes(
                args.uri, probes=args.probes_per_session,
            ))
        except (RuntimeError, TimeoutError, OSError) as error:
            print(json.dumps({
                "schema": "bolt_clock_rate_prop_off_batch_v1",
                "probe_sessions": sessions,
                "clock_fit": None,
                "incomplete_error": str(error),
                "command_authority": False,
            }, indent=2))
            raise SystemExit(1) from error
        if index + 1 < args.collect:
            time.sleep(args.between_session_s)
    result = {
        "schema": "bolt_clock_rate_prop_off_batch_v1",
        "probe_sessions": sessions,
        "clock_fit": fit_clock_rate(sessions),
        "command_authority": False,
    }
    if args.release_batch:
        from Interaction.calibrate_release_transport_delay import (
            fit_release_transport_delay,
        )
        release_data = json.loads(args.release_batch.read_text())
        release_records = (release_data["records"]
                           if release_data.get("schema") == "post_release_prop_off_calibration_batch_v1"
                           else [release_data])
        release_fit = fit_release_transport_delay(release_records)
        result["release_fit"] = release_fit
        result["firmware_delay_shadow"] = release_delay_in_firmware_clock(
            release_fit, result["clock_fit"]
        )
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
