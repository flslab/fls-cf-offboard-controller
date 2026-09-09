"""Tests for the device-independent release LMPC terminal gate."""
from dataclasses import replace
import math
import unittest

from Interaction.release_lmpc_terminal_gate import (
    ATTITUDE_ZDISTANCE_COMMAND,
    ReleaseLMPCTerminalGate,
    ReleaseLMPCTerminalSample,
    classify_terminal_post_state_commands,
)
from Interaction.velocity_lmpc_safe_set import SafeSetLimits


def sample(state_time=1.0, **updates):
    values = dict(
        state_time_s=state_time,
        velocity_xy_m_s=(0.03, 0.04),  # full XY speed is exactly 0.05 m/s
        attitude_rp_rad=(math.radians(3.0), math.radians(-3.0)),
        attitude_rate_rp_rad_s=(
            math.radians(20.0), math.radians(-20.0),
        ),
        applied_command_kind=ATTITUDE_ZDISTANCE_COMMAND,
        applied_attitude_rp_rad=(
            math.radians(3.0), math.radians(-3.0),
        ),
        state_age_s=0.10,
        state_group_skew_s=0.03,
        boundary_margin_m=0.02,
    )
    values.update(updates)
    return ReleaseLMPCTerminalSample(**values)


class LifecycleAndDwellTests(unittest.TestCase):
    def test_requires_start_and_reset_disarms(self):
        gate = ReleaseLMPCTerminalGate()
        status = gate.update(sample())
        self.assertFalse(status.active)
        self.assertFalse(status.complete)
        self.assertEqual(status.reason, "not_started")

        status = gate.start()
        self.assertTrue(status.active)
        self.assertEqual(status.reason, "started")
        gate.update(sample())
        status = gate.reset()
        self.assertFalse(status.active)
        self.assertEqual(status.phase, "idle")
        self.assertEqual(status.dwell_s, 0.0)
        self.assertEqual(status.update_count, 0)

    def test_exactly_required_dwell_completes(self):
        gate = ReleaseLMPCTerminalGate()
        gate.start()
        first = gate.update(sample(1.00))
        second = gate.update(sample(1.04))
        final = gate.update(sample(1.08))

        self.assertEqual(first.reason, "terminal_dwell_started")
        self.assertEqual(second.reason, "terminal_dwell_accumulating")
        self.assertFalse(second.complete)
        self.assertTrue(final.complete)
        self.assertEqual(final.reason, "terminal_dwell_complete")
        self.assertAlmostEqual(final.dwell_s, 0.08)
        self.assertEqual(final.consecutive_terminal_samples, 3)
        self.assertAlmostEqual(final.sample_dt_s, 0.04)
        self.assertEqual(final.to_dict()["violations"], [])

    def test_complete_gate_is_revoked_if_state_leaves_terminal_before_handoff(self):
        gate = ReleaseLMPCTerminalGate()
        gate.start()
        gate.update(sample(1.00))
        gate.update(sample(1.04))
        self.assertTrue(gate.update(sample(1.08)).complete)

        escaped = gate.update(sample(
            1.12, velocity_xy_m_s=(0.0, 0.051)
        ))
        self.assertFalse(escaped.complete)
        self.assertEqual(escaped.phase, "tracking")
        self.assertEqual(escaped.reason, "xy_speed_above_terminal_limit")
        self.assertEqual(escaped.dwell_s, 0.0)

    def test_complete_gate_is_revoked_by_malformed_sample(self):
        malformed_samples = (
            replace(sample(1.12), state_time_s=float("nan")),
            replace(sample(1.12), velocity_xy_m_s=(0.0,)),
        )
        for malformed in malformed_samples:
            gate = ReleaseLMPCTerminalGate()
            gate.start()
            gate.update(sample(1.00))
            gate.update(sample(1.04))
            self.assertTrue(gate.update(sample(1.08)).complete)

            status = gate.update(malformed)

            with self.subTest(malformed=malformed):
                self.assertFalse(status.complete)
                self.assertEqual(status.phase, "tracking")
                self.assertEqual(status.dwell_s, 0.0)
                self.assertFalse(status.sample_in_terminal_set)

    def test_cross_axis_velocity_breaks_terminal_dwell(self):
        gate = ReleaseLMPCTerminalGate()
        gate.start()
        gate.update(sample(1.00, velocity_xy_m_s=(0.0, 0.04)))
        gate.update(sample(1.04, velocity_xy_m_s=(0.0, 0.04)))

        status = gate.update(sample(
            1.08, velocity_xy_m_s=(0.0, 0.0501)
        ))

        self.assertFalse(status.complete)
        self.assertFalse(status.sample_in_terminal_set)
        self.assertEqual(status.dwell_s, 0.0)
        self.assertEqual(status.reason, "xy_speed_above_terminal_limit")

    def test_position_and_velocity_commands_break_terminal_dwell(self):
        for command_kind in ("position", "velocity"):
            gate = ReleaseLMPCTerminalGate()
            gate.start()
            gate.update(sample(1.00))
            gate.update(sample(1.04))

            status = gate.update(sample(
                1.08, applied_command_kind=command_kind
            ))

            with self.subTest(command_kind=command_kind):
                self.assertFalse(status.complete)
                self.assertEqual(status.dwell_s, 0.0)
                self.assertEqual(
                    status.reason,
                    "applied_command_kind_not_attitude_zdistance",
                )

    def test_delayed_nonlevel_or_nonattitude_command_breaks_terminal_dwell(self):
        cases = (
            ({
                "pending_command_kinds": (ATTITUDE_ZDISTANCE_COMMAND,),
                "pending_attitude_rp_rad": ((math.radians(3.01), 0.0),),
            }, "pending_attitude_above_terminal_limit"),
            ({
                "pending_command_kinds": ("velocity_hover",),
                "pending_attitude_rp_rad": ((0.0, 0.0),),
            }, "pending_command_kind_not_attitude_zdistance"),
        )
        for updates, reason in cases:
            gate = ReleaseLMPCTerminalGate()
            gate.start()
            gate.update(sample(1.00))
            status = gate.update(sample(1.04, **updates))
            with self.subTest(reason=reason):
                self.assertFalse(status.complete)
                self.assertEqual(status.reason, reason)
                self.assertEqual(status.dwell_s, 0.0)

    def test_large_and_too_small_sample_intervals_restart_at_current_sample(self):
        for timestamp, reason in (
            (1.1001, "sample_dt_above_max"),
            (1.0405, "sample_dt_below_min"),
        ):
            gate = ReleaseLMPCTerminalGate()
            gate.start()
            gate.update(sample(1.00))
            gate.update(sample(1.04))

            status = gate.update(sample(timestamp))

            with self.subTest(reason=reason):
                self.assertFalse(status.complete)
                self.assertTrue(status.sample_in_terminal_set)
                self.assertEqual(status.reason, reason)
                self.assertEqual(status.dwell_s, 0.0)
                self.assertEqual(status.consecutive_terminal_samples, 1)


class HardGateTests(unittest.TestCase):
    def assert_rejected(self, changed, reason):
        gate = ReleaseLMPCTerminalGate()
        gate.start()
        gate.update(sample(1.00))
        status = gate.update(replace(sample(1.04), **changed))
        self.assertFalse(status.complete)
        self.assertFalse(status.sample_in_terminal_set)
        self.assertEqual(status.dwell_s, 0.0)
        self.assertEqual(status.reason, reason)

    def test_stale_skewed_and_boundary_samples_are_rejected(self):
        cases = (
            ({"state_age_s": 0.1001}, "state_not_fresh"),
            ({"state_age_s": -0.001}, "state_not_fresh"),
            (
                {"state_group_skew_s": 0.0301},
                "state_group_skew_exceeded",
            ),
            (
                {"state_group_skew_s": -0.001},
                "state_group_skew_exceeded",
            ),
            ({"boundary_margin_m": 0.0199}, "boundary_margin_too_small"),
        )
        for changed, reason in cases:
            with self.subTest(changed=changed):
                self.assert_rejected(changed, reason)

    def test_attitude_rate_and_applied_attitude_are_independent_hard_gates(self):
        cases = (
            (
                {"attitude_rp_rad": (math.radians(3.01), 0.0)},
                "attitude_above_terminal_limit",
            ),
            (
                {"attitude_rate_rp_rad_s": (math.radians(20.01), 0.0)},
                "attitude_rate_above_terminal_limit",
            ),
            (
                {"applied_attitude_rp_rad": (0.0, math.radians(3.01))},
                "applied_attitude_above_terminal_limit",
            ),
        )
        for changed, reason in cases:
            with self.subTest(changed=changed):
                self.assert_rejected(changed, reason)

    def test_nonfinite_bad_shape_and_wrong_sample_type_fail_closed(self):
        malformed_samples = (
            (replace(sample(), state_time_s=float("nan")), "invalid_state_time"),
            (replace(sample(), state_age_s=float("inf")), "invalid_state_age"),
            (replace(sample(), velocity_xy_m_s=(0.0,)), "invalid_velocity_xy"),
            (
                replace(sample(), attitude_rp_rad=(0.0, float("nan"))),
                "invalid_attitude_rp",
            ),
            (
                replace(sample(), attitude_rate_rp_rad_s="bad"),
                "invalid_attitude_rate_rp",
            ),
            (
                replace(sample(), applied_attitude_rp_rad=(0.0, None)),
                "invalid_applied_attitude_rp",
            ),
            (
                replace(sample(), applied_command_kind=None),
                "invalid_applied_command_kind",
            ),
            (object(), "invalid_sample_type"),
        )
        for malformed, reason in malformed_samples:
            gate = ReleaseLMPCTerminalGate()
            gate.start()
            gate.update(sample(1.00))
            status = gate.update(malformed)
            with self.subTest(reason=reason):
                self.assertFalse(status.complete)
                self.assertEqual(status.reason, reason)
                self.assertEqual(status.dwell_s, 0.0)
                self.assertIsNone(status.last_state_time_s)

    def test_safe_set_limits_are_injected(self):
        limits = SafeSetLimits(
            terminal_velocity_tolerance_m_s=0.10,
            terminal_dwell_s=0.02,
        )
        gate = ReleaseLMPCTerminalGate(limits)
        gate.start()
        gate.update(sample(
            1.00,
            velocity_xy_m_s=(0.06, 0.06),
            state_age_s=0.0,
            state_group_skew_s=0.0,
        ))
        status = gate.update(sample(
            1.02,
            velocity_xy_m_s=(0.06, 0.06),
            state_age_s=0.0,
            state_group_skew_s=0.0,
        ))
        self.assertTrue(status.complete)
        self.assertAlmostEqual(status.dwell_s, 0.02)


class PostTerminalCommandTests(unittest.TestCase):
    def test_only_real_position_handoff_closes_successfully(self):
        measured_position = (0.1, 0.2, 1.0)
        position = {
            "kind": "position",
            "position_m": [0.1, 0.2, 1.0],
        }
        self.assertEqual(
            classify_terminal_post_state_commands(
                (position,), "position_hold",
                terminal_position_m=measured_position,
            ),
            "position_handoff",
        )
        self.assertEqual(
            classify_terminal_post_state_commands(
                (position,), "attitude_coast",
                terminal_position_m=measured_position,
            ),
            "unsafe",
        )

        for bad_position in (
            {"kind": "position"},
            {"kind": "position", "position_m": [0.111, 0.2, 1.0]},
            {"kind": "position", "position_m": [0.1, float("nan"), 1.0]},
        ):
            with self.subTest(bad_position=bad_position):
                self.assertEqual(
                    classify_terminal_post_state_commands(
                        (bad_position,), "position_hold",
                        terminal_position_m=measured_position,
                    ),
                    "unsafe",
                )

        self.assertEqual(
            classify_terminal_post_state_commands(
                (position,), "position_hold"
            ),
            "unsafe",
        )

    def test_level_attitude_holds_open_and_other_sends_are_unsafe(self):
        level = {
            "kind": ATTITUDE_ZDISTANCE_COMMAND,
            "roll_deg": 3.0,
            "pitch_deg": -3.0,
        }
        self.assertEqual(
            classify_terminal_post_state_commands((level,), "attitude_coast"),
            "level_attitude_hold",
        )
        for commands in (
            (),
            ({"kind": "velocity_hover"},),
            ({**level, "roll_deg": 3.01},),
            (level, {"kind": "position"}),
        ):
            with self.subTest(commands=commands):
                self.assertEqual(
                    classify_terminal_post_state_commands(
                        commands, "attitude_coast"
                    ),
                    "unsafe",
                )


if __name__ == "__main__":
    unittest.main()
