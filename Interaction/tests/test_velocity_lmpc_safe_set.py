import json
import math
from dataclasses import replace
from pathlib import Path
import tempfile
import unittest

from Interaction.velocity_lmpc_safe_set import (
    ARTIFACT_KIND,
    LMPCContext,
    NoSafeSetCoverageError,
    SafeSetLimits,
    SafeSetValidationError,
    TERMINAL_OUTCOME,
    VelocityLMPCEpisode,
    VelocityLMPCSafeSet,
    VelocityLMPCSample,
    reverse_cost_to_go,
)


def context(speed=0.4, *, direction=1, fingerprint="plant-v1", **updates):
    values = dict(
        direction_sign=direction,
        initial_speed_m_s=speed,
        cross_speed_m_s=0.0,
        roll_rad=0.0,
        pitch_rad=0.0,
        roll_rate_rad_s=0.0,
        pitch_rate_rad_s=0.0,
        boundary_margin_m=1.0,
        battery_voltage_v=7.8,
        model_fingerprint=fingerprint,
    )
    values.update(updates)
    return LMPCContext(**values)


def sample(velocity, *, cross_velocity=0.0, dt=0.04, roll=0.0, pitch=0.0,
           roll_rate=0.0, pitch_rate=0.0, command=0.0, marker=0.0,
           position=0.0):
    return VelocityLMPCSample(
        state=(velocity, roll, marker),
        command=(command,),
        dt_s=dt,
        aligned_velocity_m_s=velocity,
        cross_velocity_m_s=cross_velocity,
        position_m=(0.0, position, 1.0),
        aligned_position_m=position,
        roll_rad=roll,
        pitch_rad=pitch,
        roll_rate_rad_s=roll_rate,
        pitch_rate_rad_s=pitch_rate,
    )


def episode(speed=0.4, *, episode_id="ep-1", direction=1,
            fingerprint="plant-v1", outcome=TERMINAL_OUTCOME,
            samples=None):
    samples = tuple(samples or (
        sample(speed, command=-0.10, marker=0.0),
        sample(0.20, command=-0.05, marker=1.0),
        sample(0.04, command=0.0, marker=2.0),
        sample(0.02, command=0.0, marker=3.0),
        sample(0.01, dt=0.0, command=0.0, marker=4.0),
    ))
    return VelocityLMPCEpisode.from_samples(
        episode_id=episode_id,
        context=context(speed, direction=direction, fingerprint=fingerprint),
        outcome=outcome,
        terminal_handoff_position_m=samples[-1].position_m,
        samples=samples,
    )


def database():
    return VelocityLMPCSafeSet(
        state_dimension=3,
        command_dimension=1,
        state_scales=(0.5, math.radians(10.0), 1.0),
    )


class ContextAndSampleSchemaTests(unittest.TestCase):
    def test_context_requires_exact_direction_and_finite_physical_values(self):
        for updates in (
            {"direction": 0},
            {"direction": True},
            {"speed": 0.0},
            {"cross_speed_m_s": float("nan")},
            {"boundary_margin_m": 0.0},
            {"battery_voltage_v": -1.0},
            {"fingerprint": "contains whitespace"},
        ):
            speed = updates.pop("speed", 0.4)
            direction = updates.pop("direction", 1)
            fingerprint = updates.pop("fingerprint", "plant-v1")
            with self.subTest(updates=updates), self.assertRaises(
                SafeSetValidationError
            ):
                context(
                    speed, direction=direction, fingerprint=fingerprint,
                    **updates,
                )

    def test_sample_requires_finite_nonempty_vectors_and_nonnegative_dt(self):
        with self.assertRaises(SafeSetValidationError):
            replace(sample(0.4), state=())
        with self.assertRaises(SafeSetValidationError):
            replace(sample(0.4), command=(float("inf"),))
        with self.assertRaises(SafeSetValidationError):
            replace(sample(0.4), dt_s=-0.01)
        with self.assertRaises(SafeSetValidationError):
            replace(sample(0.4), position_m=(0.0, float("nan"), 1.0))

    def test_nested_json_schema_rejects_missing_and_unknown_fields(self):
        artifact = database()
        artifact.add_episode(episode())
        raw = artifact.to_dict()
        self.assertIn(
            "aligned_position_m", raw["episodes"][0]["samples"][0]
        )
        raw["episodes"][0]["context"]["unexpected"] = 1
        with self.assertRaisesRegex(SafeSetValidationError, "unknown"):
            VelocityLMPCSafeSet.from_dict(raw)

        raw = artifact.to_dict()
        del raw["episodes"][0]["samples"][0]["roll_rad"]
        with self.assertRaisesRegex(SafeSetValidationError, "missing"):
            VelocityLMPCSafeSet.from_dict(raw)

    def test_json_rejects_duplicate_keys_and_nonfinite_constants(self):
        with self.assertRaisesRegex(SafeSetValidationError, "duplicate JSON key"):
            VelocityLMPCSafeSet.loads(
                '{"schema_version":1,"schema_version":1}'
            )
        with self.assertRaisesRegex(SafeSetValidationError, "non-finite"):
            VelocityLMPCSafeSet.loads(
                '{"schema_version":NaN}'
            )


class EpisodeAdmissionTests(unittest.TestCase):
    def test_successful_episode_is_admitted_with_reverse_cost_to_go(self):
        artifact = database()
        item = episode()

        result = artifact.add_episode(item)

        self.assertTrue(result["passed"])
        self.assertAlmostEqual(result["terminal_dwell_s"], 0.08)
        self.assertEqual(item.cost_to_go_s, (0.16, 0.12, 0.08, 0.04, 0.0))
        self.assertEqual(reverse_cost_to_go(item.samples), item.cost_to_go_s)
        self.assertEqual(artifact.episodes, (item,))

    def test_rejection_never_mutates_database(self):
        artifact = database()
        failures = {
            "wrong outcome": episode(outcome="fallback"),
            "reversal": episode(samples=(
                sample(0.4, command=-0.1), sample(0.1),
                sample(-0.03), sample(0.0), sample(0.0, dt=0.0),
            )),
            "short terminal dwell": episode(samples=(
                sample(0.4, command=-0.1), sample(0.2),
                sample(0.06), sample(0.02), sample(0.01, dt=0.0),
            )),
            "nonlevel terminal": episode(samples=(
                sample(0.4, command=-0.1), sample(0.2),
                sample(0.04), sample(0.02),
                sample(0.01, dt=0.0, roll=math.radians(4.0)),
            )),
            "nonzero cross-axis terminal speed": episode(samples=(
                sample(0.4, command=-0.1), sample(0.2),
                sample(0.04), sample(0.02),
                sample(0.01, cross_velocity=0.051, dt=0.0),
            )),
            "gapped telemetry": episode(samples=(
                sample(0.4, dt=0.06, command=-0.1), sample(0.2),
                sample(0.04), sample(0.02), sample(0.01, dt=0.0),
            )),
            "nonzero final dt": episode(samples=(
                sample(0.4, command=-0.1), sample(0.2),
                sample(0.04), sample(0.02), sample(0.01, dt=0.04),
            )),
            "stale telemetry": episode(samples=(
                sample(0.4, command=-0.1),
                replace(sample(0.2), state_age_s=0.11),
                sample(0.04), sample(0.02), sample(0.01, dt=0.0),
            )),
            "skewed telemetry": episode(samples=(
                sample(0.4, command=-0.1),
                replace(sample(0.2), state_group_skew_s=0.04),
                sample(0.04), sample(0.02), sample(0.01, dt=0.0),
            )),
            "boundary margin": episode(samples=(
                sample(0.4, command=-0.1),
                replace(sample(0.2), boundary_margin_m=0.01),
                sample(0.04), sample(0.02), sample(0.01, dt=0.0),
            )),
            "safety event": episode(samples=(
                sample(0.4, command=-0.1),
                replace(sample(0.2), safety_violation=True),
                sample(0.04), sample(0.02), sample(0.01, dt=0.0),
            )),
        }
        for name, item in failures.items():
            with self.subTest(name=name), self.assertRaises(
                SafeSetValidationError
            ):
                artifact.add_episode(item)
            self.assertEqual(artifact.episodes, ())

    def test_opaque_state_cannot_hide_command_or_attitude_limit_violation(self):
        artifact = database()
        bad_command = list(episode().samples)
        bad_command[1] = replace(bad_command[1], command=(math.radians(31.0),))
        with self.assertRaisesRegex(SafeSetValidationError, "command exceeds"):
            artifact.add_episode(episode(samples=bad_command))
        bad_tilt = list(episode().samples)
        bad_tilt[1] = replace(bad_tilt[1], roll_rad=math.radians(31.0))
        with self.assertRaisesRegex(SafeSetValidationError, "attitude exceeds"):
            artifact.add_episode(episode(samples=bad_tilt))

    def test_opaque_state_velocity_must_match_explicit_measurement(self):
        artifact = database()
        samples = list(episode().samples)
        samples[1] = replace(samples[1], state=(0.35, 0.0, 1.0))

        with self.assertRaisesRegex(
            SafeSetValidationError, "opaque state disagrees"
        ):
            artifact.add_episode(episode(samples=samples))

    def test_position_trace_and_terminal_backup_are_measured_consistently(self):
        artifact = database()
        item = episode()
        bad_trace = list(item.samples)
        bad_trace[1] = replace(
            bad_trace[1], position_m=(0.0, 0.02, 1.0)
        )
        with self.assertRaisesRegex(
            SafeSetValidationError, "aligned position disagrees"
        ):
            artifact.add_episode(episode(samples=bad_trace))

        with self.assertRaisesRegex(
            SafeSetValidationError, "handoff target"
        ):
            artifact.add_episode(replace(
                item,
                terminal_handoff_position_m=(0.011, 0.0, 1.0),
            ))

    def test_terminal_delay_queue_must_also_be_level(self):
        base = episode()
        samples = tuple(
            replace(
                item,
                state=(
                    item.aligned_velocity_m_s, item.roll_rad, 0.0,
                    0.0,
                    math.radians(4.0) if index == len(base.samples)-1 else 0.0,
                ),
            )
            for index, item in enumerate(base.samples)
        )
        artifact = VelocityLMPCSafeSet(
            state_dimension=5,
            command_dimension=1,
            state_scales=(0.5, 0.1, 1.0, 0.1, 0.1),
        )
        with self.assertRaisesRegex(
            SafeSetValidationError, "full measured terminal set"
        ):
            artifact.add_episode(VelocityLMPCEpisode.from_samples(
                episode_id="nonlevel-delay-queue",
                context=base.context,
                outcome=TERMINAL_OUTCOME,
                terminal_handoff_position_m=(0.0, 0.0, 1.0),
                samples=samples,
            ))

    def test_terminal_delay_queue_remains_mandatory_after_json_load(self):
        base = episode()
        samples = tuple(
            replace(
                item,
                state=(item.aligned_velocity_m_s, item.roll_rad, 0.0, 0.0),
            )
            for item in base.samples
        )
        artifact = VelocityLMPCSafeSet(
            state_dimension=4,
            command_dimension=1,
            state_scales=(0.5, 0.1, 1.0, 0.1),
        )
        artifact.add_episode(VelocityLMPCEpisode.from_samples(
            episode_id="round-trip-terminal-queue",
            context=base.context,
            outcome=TERMINAL_OUTCOME,
            terminal_handoff_position_m=(0.0, 0.0, 1.0),
            samples=samples,
        ))
        raw = artifact.to_dict()
        raw["episodes"][0]["samples"][-1]["state"][-1] = math.radians(4.0)

        with self.assertRaisesRegex(
            SafeSetValidationError, "full measured terminal set"
        ):
            VelocityLMPCSafeSet.from_dict(raw)

    def test_aligned_velocity_path_limit_is_artifact_bound(self):
        limited = VelocityLMPCSafeSet(
            state_dimension=3,
            command_dimension=1,
            state_scales=(0.5, math.radians(10.0), 1.0),
            limits=SafeSetLimits(max_abs_aligned_velocity_m_s=0.3),
        )
        with self.assertRaisesRegex(
            SafeSetValidationError, "aligned velocity exceeds the path limit"
        ):
            limited.add_episode(episode())
        self.assertEqual(limited.episodes, ())

    def test_delayed_command_queue_is_bounded_for_the_entire_path(self):
        base = episode()
        samples = tuple(
            replace(
                item,
                state=(
                    item.aligned_velocity_m_s,
                    item.roll_rad,
                    0.0,
                    math.radians(31.0) if index == 1 else 0.0,
                ),
            )
            for index, item in enumerate(base.samples)
        )
        artifact = VelocityLMPCSafeSet(
            state_dimension=4,
            command_dimension=1,
            state_scales=(0.5, 0.1, 1.0, 0.1),
        )

        with self.assertRaisesRegex(
            SafeSetValidationError, "delayed command queue exceeds"
        ):
            artifact.add_episode(VelocityLMPCEpisode.from_samples(
                episode_id="unsafe-delayed-queue",
                context=base.context,
                outcome=TERMINAL_OUTCOME,
                terminal_handoff_position_m=(0.0, 0.0, 1.0),
                samples=samples,
            ))
        self.assertEqual(artifact.episodes, ())

    def test_zero_duration_safe_tail_entry_change_is_rejected(self):
        base = episode()
        samples = tuple(
            replace(
                item,
                state=(
                    item.aligned_velocity_m_s,
                    item.roll_rad,
                    0.0,
                    0.01 if index == len(base.samples)-1 else item.command[0],
                ),
            )
            for index, item in enumerate(base.samples)
        )
        artifact = VelocityLMPCSafeSet(
            state_dimension=4,
            command_dimension=1,
            state_scales=(0.5, 0.1, 1.0, 0.1),
        )
        with self.assertRaisesRegex(
            SafeSetValidationError, "entry command change at zero dt_s"
        ):
            artifact.add_episode(VelocityLMPCEpisode.from_samples(
                episode_id="zero-dt-entry-change",
                context=base.context,
                outcome=TERMINAL_OUTCOME,
                terminal_handoff_position_m=(0.0, 0.0, 1.0),
                samples=samples,
            ))

    def test_initial_cross_speed_must_match_first_measured_sample(self):
        artifact = database()
        item = episode()
        with self.assertRaisesRegex(
            SafeSetValidationError, "initial cross speed"
        ):
            artifact.add_episode(replace(
                item,
                context=replace(item.context, cross_speed_m_s=0.011),
            ))

    def test_costs_dimensions_initial_speed_and_duplicate_ids_are_checked(self):
        artifact = database()
        item = episode()
        with self.assertRaisesRegex(SafeSetValidationError, "cost_to_go"):
            artifact.add_episode(replace(item, cost_to_go_s=(0.0,)*5))
        wrong_dimension = list(item.samples)
        wrong_dimension[0] = replace(wrong_dimension[0], state=(0.4, 0.0))
        with self.assertRaisesRegex(SafeSetValidationError, "state dimension"):
            artifact.add_episode(episode(samples=wrong_dimension))
        inconsistent = replace(item, context=context(0.8))
        with self.assertRaisesRegex(SafeSetValidationError, "initial speed"):
            artifact.add_episode(inconsistent)

        artifact.add_episode(item)
        with self.assertRaisesRegex(SafeSetValidationError, "duplicate"):
            artifact.add_episode(item)


class ConditionalQueryTests(unittest.TestCase):
    def setUp(self):
        self.artifact = database()
        self.artifact.add_episode(episode(0.4, episode_id="slow"))
        self.artifact.add_episode(episode(0.8, episode_id="fast"))
        self.artifact.add_episode(episode(
            0.6, episode_id="reverse-direction", direction=-1,
        ))
        self.artifact.add_episode(episode(
            0.6, episode_id="other-plant", fingerprint="plant-v2",
        ))

    def test_query_interpolates_only_between_speed_brackets(self):
        result = self.artifact.query(
            context(0.6), (0.021, 0.0, 3.0), neighbors_per_bracket=2,
        )

        self.assertEqual(result.lower_initial_speed_m_s, 0.4)
        self.assertEqual(result.upper_initial_speed_m_s, 0.8)
        self.assertAlmostEqual(result.upper_interpolation_weight, 0.5)
        self.assertEqual(
            {point.initial_speed_m_s for point in result.points}, {0.4, 0.8}
        )
        self.assertTrue(all(
            point.episode_id in {"slow", "fast"} for point in result.points
        ))
        self.assertTrue(all(
            point.episode_id not in {"reverse-direction", "other-plant"}
            for point in result.points
        ))

    def test_query_refuses_speed_extrapolation(self):
        for speed in (0.39, 0.81):
            with self.subTest(speed=speed), self.assertRaisesRegex(
                NoSafeSetCoverageError, "convex hull"
            ):
                self.artifact.query(context(speed), (0.2, 0.0, 0.0))

    def test_exact_speed_uses_only_that_conditional_partition(self):
        result = self.artifact.query(
            context(0.4), (0.01, 0.0, 4.0), neighbors_per_bracket=3,
        )
        self.assertEqual(result.lower_initial_speed_m_s, 0.4)
        self.assertEqual(result.upper_initial_speed_m_s, 0.4)
        self.assertEqual(result.upper_interpolation_weight, 0.0)
        self.assertEqual({point.episode_id for point in result.points}, {"slow"})
        self.assertEqual(result.points[0].sample_index, 4)
        self.assertEqual(result.points[0].cost_to_go_s, 0.0)

    def test_query_exposes_measured_suffix_distance_and_constraint_envelopes(self):
        artifact = database()
        item = episode(samples=(
            sample(0.4, command=-0.10, marker=0.0, position=0.000),
            sample(
                0.20, command=-0.05, marker=1.0, position=0.010,
                roll=0.02, roll_rate=0.10,
            ),
            sample(0.04, command=0.0, marker=2.0, position=0.025),
            sample(0.02, command=0.0, marker=3.0, position=0.030),
            sample(
                0.01, dt=0.0, command=0.0, marker=4.0,
                position=0.029,
            ),
        ))
        artifact.add_episode(item)

        result = artifact.query(
            context(0.4), (0.20, 0.02, 1.0),
            neighbors_per_bracket=5,
        )
        point = next(
            value for value in result.points if value.sample_index == 1
        )
        self.assertAlmostEqual(point.remaining_forward_distance_m, 0.020)
        self.assertAlmostEqual(
            point.tail_max_abs_aligned_velocity_m_s, 0.20
        )
        self.assertAlmostEqual(point.tail_max_abs_command, 0.05)
        self.assertAlmostEqual(point.tail_max_abs_tilt_rad, 0.02)
        self.assertAlmostEqual(point.tail_max_abs_rate_rad_s, 0.10)
        self.assertAlmostEqual(point.tail_max_command_slew_rad_s, 1.25)

        terminal = next(
            value for value in result.points if value.sample_index == 4
        )
        self.assertEqual(terminal.remaining_forward_distance_m, 0.0)
        self.assertEqual(terminal.tail_max_abs_aligned_velocity_m_s, 0.01)
        self.assertEqual(terminal.tail_max_abs_command, 0.0)
        self.assertEqual(terminal.tail_max_abs_tilt_rad, 0.0)
        self.assertEqual(terminal.tail_max_abs_rate_rad_s, 0.0)
        self.assertEqual(terminal.tail_max_command_slew_rad_s, 0.0)

    def test_entry_slew_uses_newest_queue_command_and_survives_round_trip(self):
        base = episode(samples=(
            replace(
                sample(0.4, command=-0.10, marker=0.0),
                state=(0.4, 0.0, 0.0, -0.10, -0.10),
            ),
            replace(
                sample(0.20, command=-0.05, marker=1.0),
                state=(0.20, 0.0, 1.0, 0.40, -0.20),
            ),
            replace(
                sample(0.04, command=0.0, marker=2.0),
                state=(0.04, 0.0, 2.0, 0.0, 0.0),
            ),
            replace(
                sample(0.02, command=0.0, marker=3.0),
                state=(0.02, 0.0, 3.0, 0.0, 0.0),
            ),
            replace(
                sample(0.01, dt=0.0, command=0.0, marker=4.0),
                state=(0.01, 0.0, 4.0, 0.0, 0.0),
            ),
        ))
        artifact = VelocityLMPCSafeSet(
            state_dimension=5,
            command_dimension=1,
            state_scales=(0.5, 0.1, 1.0, 0.1, 0.1),
        )
        artifact.add_episode(base)
        loaded = VelocityLMPCSafeSet.loads(artifact.dumps())

        result = loaded.query(
            context(0.4), (0.20, 0.0, 1.0, 0.40, -0.20),
            neighbors_per_bracket=5,
        )
        point = next(
            value for value in result.points if value.sample_index == 1
        )
        self.assertAlmostEqual(
            point.tail_max_command_slew_rad_s,
            abs(-0.05-(-0.20))/0.04,
        )
        self.assertAlmostEqual(
            point.tail_max_abs_aligned_velocity_m_s, 0.20
        )

    def test_direction_and_model_fingerprint_are_exact_isolation_keys(self):
        negative = self.artifact.query(
            context(0.6, direction=-1), (0.2, 0.0, 1.0)
        )
        self.assertEqual(
            {point.episode_id for point in negative.points},
            {"reverse-direction"},
        )
        other = self.artifact.query(
            context(0.6, fingerprint="plant-v2"), (0.2, 0.0, 1.0)
        )
        self.assertEqual(
            {point.episode_id for point in other.points}, {"other-plant"}
        )
        with self.assertRaises(NoSafeSetCoverageError):
            self.artifact.query(
                context(0.6, fingerprint="unknown"), (0.2, 0.0, 1.0)
            )

    def test_release_context_gates_boundary_battery_and_initial_motion(self):
        rejected = (
            context(0.6, boundary_margin_m=0.99),
            context(0.6, battery_voltage_v=7.29),
            context(0.6, cross_speed_m_s=0.061),
            context(0.6, roll_rad=math.radians(3.1)),
            context(0.6, roll_rate_rad_s=math.radians(30.1)),
        )
        for item in rejected:
            with self.subTest(context=item), self.assertRaises(
                NoSafeSetCoverageError
            ):
                self.artifact.query(item, (0.2, 0.0, 1.0))

    def test_query_validates_dimensions_and_neighbor_count(self):
        with self.assertRaisesRegex(SafeSetValidationError, "dimension"):
            self.artifact.query(context(0.6), (0.2, 0.0))
        with self.assertRaisesRegex(SafeSetValidationError, "at least"):
            self.artifact.query(
                context(0.6), (0.2, 0.0, 0.0), neighbors_per_bracket=0
            )


class ArtifactPersistenceTests(unittest.TestCase):
    def test_strict_round_trip_and_atomic_file_publish(self):
        artifact = database()
        artifact.add_episode(episode())
        encoded = artifact.dumps()
        decoded = VelocityLMPCSafeSet.loads(encoded)

        self.assertEqual(decoded.to_dict(), artifact.to_dict())
        self.assertEqual(
            decoded.limits.max_abs_aligned_velocity_m_s,
            artifact.limits.max_abs_aligned_velocity_m_s,
        )
        self.assertEqual(json.loads(encoded)["kind"], ARTIFACT_KIND)

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/"safe-set.json"
            artifact.save(path)
            loaded = VelocityLMPCSafeSet.load(path)
            self.assertEqual(loaded.to_dict(), artifact.to_dict())
            self.assertEqual(list(Path(directory).glob("*.tmp")), [])

    def test_artifact_schema_version_kind_and_dimensions_are_strict(self):
        raw = database().to_dict()
        for name, mutate in (
            ("version", lambda item: item.update(schema_version=2)),
            ("boolean version", lambda item: item.update(schema_version=True)),
            ("kind", lambda item: item.update(kind="wrong")),
            ("unknown", lambda item: item.update(extra=True)),
            ("scales", lambda item: item.update(state_scales=[1.0, 1.0])),
        ):
            changed = json.loads(json.dumps(raw))
            mutate(changed)
            with self.subTest(name=name), self.assertRaises(
                SafeSetValidationError
            ):
                VelocityLMPCSafeSet.from_dict(changed)

    def test_serialized_forged_cost_is_rejected_on_load(self):
        artifact = database()
        artifact.add_episode(episode())
        raw = artifact.to_dict()
        raw["episodes"][0]["cost_to_go_s"][0] += 1.0
        with self.assertRaisesRegex(SafeSetValidationError, "cost_to_go"):
            VelocityLMPCSafeSet.from_dict(raw)


class LimitValidationTests(unittest.TestCase):
    def test_limits_reject_inverted_or_nonfinite_ranges(self):
        for updates in (
            dict(min_sample_dt_s=0.06, max_sample_dt_s=0.05),
            dict(max_abs_command=0.0),
            dict(terminal_dwell_s=float("nan")),
            dict(reverse_velocity_tolerance_m_s=-0.01),
            dict(terminal_tilt_tolerance_rad=1.0, max_path_tilt_rad=0.5),
        ):
            with self.subTest(updates=updates), self.assertRaises(
                SafeSetValidationError
            ):
                SafeSetLimits(**updates)


if __name__ == "__main__":
    unittest.main()
