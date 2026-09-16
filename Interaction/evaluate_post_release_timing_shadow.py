"""Offline timing stress using the deterministic, non-authoritative harness.

This does not replace CrazySim or demonstrate that live position packets have
the assumed delay. It leaves every physical stop and safety gate unchanged.
"""

from __future__ import annotations

import argparse
from dataclasses import replace
import json
from pathlib import Path

from Interaction.contact_release_closed_loop_harness import (
    ClosedLoopFaults,
    ClosedLoopHarnessConfig,
    run_contact_release_closed_loop_harness,
)


def evaluate_timing_shadow(calibration_batch, *, position_delay_ms=(0, 1, 2, 3, 4, 5),
                           assumed_fixed_position_delay_ms=3.0):
    if calibration_batch.get("schema") != "bolt_clock_rate_prop_off_batch_v1":
        raise ValueError("clock-rate batch schema invalid")
    clock = calibration_batch["clock_fit"]
    release = calibration_batch["release_fit"]
    fixed = calibration_batch["firmware_delay_shadow"]
    if (not clock["holdout_compatible"]
            or not release["holdout_covered"]
            or fixed["command_authority"]):
        raise ValueError("release/clock shadow calibration invalid")
    base = ClosedLoopHarnessConfig()
    rows = []
    compensated_rows = []
    if (isinstance(assumed_fixed_position_delay_ms, bool)
            or not isinstance(assumed_fixed_position_delay_ms, (float, int))
            or not 0 <= assumed_fixed_position_delay_ms <= 100):
        raise ValueError("assumed fixed position delay invalid")
    compensated_config = replace(
        base,
        position_delay_s=assumed_fixed_position_delay_ms / 1000.0,
        position_delay_compensation_enabled=True,
    )
    for delay_ms in position_delay_ms:
        if isinstance(delay_ms, bool) or not isinstance(delay_ms, (float, int)) or delay_ms < 0:
            raise ValueError("position delay must be nonnegative")
        result = run_contact_release_closed_loop_harness(
            config=replace(base, position_delay_s=delay_ms / 1000.0)
        )
        rows.append({
            "assumed_position_delay_ms": delay_ms,
            "harness_passed": result.passed,
            "failed_hard_gates": [
                name for name, gate in result.gates.items() if not gate.passed
            ],
            "stop_overshoot_m": result.gates["stop_overshoot"].observed,
            "stop_overshoot_limit_m": result.gates["stop_overshoot"].limit,
            "terminal_speed_m_s": result.gates["terminal_speed"].observed,
            "terminal_speed_limit_m_s": result.gates["terminal_speed"].limit,
            "position_update_count": result.position_update_count,
        })
        corrected = run_contact_release_closed_loop_harness(
            config=compensated_config,
            faults=ClosedLoopFaults(position_delay_s=delay_ms / 1000.0),
        )
        compensated_rows.append({
            "actual_assumed_position_delay_ms": delay_ms,
            "harness_passed": corrected.passed,
            "failed_hard_gates": [
                name for name, gate in corrected.gates.items()
                if not gate.passed
            ],
            "stop_overshoot_m": corrected.gates["stop_overshoot"].observed,
            "terminal_speed_m_s": corrected.gates["terminal_speed"].observed,
        })
    return {
        "schema": "post_release_fixed_delay_timing_shadow_v1",
        "scope": "deterministic_harness_only_not_crazysim_or_flight",
        "position_delay_source": "unverified_0_to_5_ms_assumption",
        "release_fixed_delay_firmware_ms": fixed["fixed_delay_firmware_ms"],
        "release_uncertainty_firmware_ms": fixed["empirical_uncertainty_firmware_ms"],
        "position_delay_stress": rows,
        "fixed_position_delay_compensation_trial": {
            "nominal_delay_ms": assumed_fixed_position_delay_ms,
            "simulation_only": True,
            "actual_delay_stress": compensated_rows,
            "all_assumed_delays_passed": all(
                row["harness_passed"] for row in compensated_rows
            ),
        },
        "all_assumed_delays_passed": all(row["harness_passed"] for row in rows),
        "control_code_changed": False,
        "command_authority": False,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("calibration_batch", type=Path)
    args = parser.parse_args()
    data = json.loads(args.calibration_batch.read_text())
    print(json.dumps(evaluate_timing_shadow(data), indent=2))


if __name__ == "__main__":
    main()
