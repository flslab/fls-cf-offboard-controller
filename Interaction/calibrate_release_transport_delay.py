"""Fit a bounded, fixed Pi-receive to Bolt-receive release delay.

Inputs are prop-off ``probe_post_release_event`` JSON files. The fitted
constant is diagnostic only: it is not a physical release timestamp, a
position-packet delay, or permission to use the post-release estimator.
No Pi clock epoch needs to be sent to firmware during operation.
"""

from __future__ import annotations

import argparse
import json
import math
import statistics
from pathlib import Path


def _finite_nonnegative(value, name):
    if isinstance(value, bool) or not isinstance(value, (float, int)):
        raise ValueError(f"{name} must be a nonnegative finite number")
    number = float(value)
    if not math.isfinite(number) or number < 0:
        raise ValueError(f"{name} must be a nonnegative finite number")
    return number


def release_delay_interval_ms(record):
    """Conservative delay bounds from one matched event and its echo.

    The radio uplink cannot take less than zero or more than the complete
    host send-to-echo round trip. We deliberately assume no path symmetry.
    """
    if record.get("schema") != "post_release_receive_event_prop_off_v1":
        raise ValueError("unsupported release diagnostic schema")
    sent, ack = record["sent"], record["ack"]
    identity = ("session_id", "sequence", "release_event_arduino_time_ms",
                "pi_receive_to_send_elapsed_us")
    if any(sent.get(key) != ack.get(key) for key in identity):
        raise ValueError("release event and firmware echo do not match")
    if not ack.get("radio_delivery_confirmed"):
        raise ValueError("firmware delivery was not confirmed")
    elapsed_ms = _finite_nonnegative(
        sent["pi_receive_to_send_elapsed_us"], "Pi receive-to-send delay"
    ) / 1000.0
    roundtrip_ms = _finite_nonnegative(
        record["host_radio_roundtrip_ms"], "host radio round trip"
    )
    if roundtrip_ms <= 0 or elapsed_ms > 1000:
        raise ValueError("release diagnostic timings are invalid")
    return elapsed_ms, elapsed_ms + roundtrip_ms


def fit_release_transport_delay(records, *, minimum_samples=20,
                                maximum_uncertainty_ms=None):
    """Return a fixed midpoint and a worst-observed causal error bound.

    This is an empirical envelope, not a probabilistic confidence interval
    or a future worst-case guarantee. Validation samples are held out in
    chronological input order to detect nonstationarity.
    """
    if type(minimum_samples) is not int or minimum_samples < 4:
        raise ValueError("minimum_samples must be an integer of at least 4")
    if maximum_uncertainty_ms is not None:
        maximum_uncertainty_ms = _finite_nonnegative(
            maximum_uncertainty_ms, "maximum uncertainty"
        )
    intervals = [release_delay_interval_ms(record) for record in records]
    if len(intervals) < minimum_samples:
        raise ValueError(
            f"at least {minimum_samples} matched prop-off releases required"
        )
    train_count = len(intervals) * 4 // 5
    if train_count == len(intervals):
        train_count -= 1
    training = intervals[:train_count]
    validation = intervals[train_count:]
    low = min(item[0] for item in training)
    high = max(item[1] for item in training)
    fixed = (low + high) / 2.0
    uncertainty = (high - low) / 2.0
    holdout_covered = all(
        item[0] >= low and item[1] <= high for item in validation
    )
    # No implicit tolerance: the estimator must supply an independently
    # justified budget before this diagnostic fit can even be considered.
    within_limit = (maximum_uncertainty_ms is not None
                    and uncertainty <= maximum_uncertainty_ms)
    eligible = holdout_covered and within_limit
    return {
        "schema": "post_release_fixed_transport_delay_calibration_v1",
        "time_basis": "pi_complete_uart_line_receive_to_firmware_packet_handler",
        "fixed_delay_ms": fixed,
        "empirical_uncertainty_ms": uncertainty,
        "observed_lower_ms": low,
        "observed_upper_ms": high,
        "median_pi_receive_to_send_ms": statistics.median(
            (lo for lo, _ in intervals)
        ),
        "training_count": len(training),
        "holdout_count": len(validation),
        "holdout_covered": holdout_covered,
        "maximum_uncertainty_ms": maximum_uncertainty_ms,
        "within_uncertainty_limit": within_limit,
        "calibration_eligible": eligible,
        "position_transport_calibrated": False,
        "physical_release_time_calibrated": False,
        "command_authority": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("records", nargs="+", type=Path,
                        help="JSON outputs from prop-off release probes, in time order")
    parser.add_argument("--minimum-samples", type=int, default=20)
    parser.add_argument("--maximum-uncertainty-ms", type=float)
    args = parser.parse_args()
    records = []
    for path in args.records:
        document = json.loads(path.read_text())
        if document.get("schema") == "post_release_prop_off_calibration_batch_v1":
            records.extend(document["records"])
        else:
            records.append(document)
    print(json.dumps(fit_release_transport_delay(
        records, minimum_samples=args.minimum_samples,
        maximum_uncertainty_ms=args.maximum_uncertainty_ms,
    ), indent=2))


if __name__ == "__main__":
    main()
