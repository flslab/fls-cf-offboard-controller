from copy import deepcopy
from types import SimpleNamespace
import unittest

from Interaction.contact_attitude_experiment import (
    INERTIAL_POSITION,
    ONBOARD_MIRROR,
    experiment_run_config,
    prepare_contact_attitude_experiment_mission,
    validate_contact_attitude_cli,
)


def mission():
    return {
        "Interaction": {"action": "translation", "config": {
            "detection_method": "momentum_impulse",
            "virtual_object": {
                "contact_detection": {
                    "source": "potentiometer",
                    "force_threshold_n": 0.18,
                    "onset_dwell_s": 0.03,
                },
                "release_behavior": {
                    "mode": "potentiometer_coast",
                    "force_drop_n": 0.04,
                    "candidate_lead_drop_n": 0.005,
                    "decrease_rate_n_s": 0.05,
                    "unloaded_force_n": 0.17,
                    "unloaded_dwell_s": 0.05,
                    "max_sample_gap_s": 0.15,
                    "candidate_stall_timeout_s": 0.15,
                    "candidate_sensor_stale_timeout_s": 0.25,
                },
            },
            "wrench_interaction": {
                "state_source": "onboard",
                "shadow_mode": False,
                "initial_contact_arming": {
                    "enabled": True,
                    "apply_after_each_interaction": True,
                    "max_xy_speed_m_s": 0.03,
                    "stationary_dwell_s": 0.50,
                    "max_sample_gap_s": 0.10,
                },
            },
        }},
    }


def args(run, **overrides):
    values = dict(
        contact_attitude_run=run,
        interaction=True,
        sense=True,
        vicon=True,
        ground_test=False,
        droneless=False,
        vicon_mode="pointcloud" if run == 1 else "rigidbody",
        vicon_full_pose=run == 3,
        obj_name=None if run == 1 else "FLS",
        save_vicon=False,
        log=True,
    )
    values.update(overrides)
    return SimpleNamespace(**values)


class ContactAttitudeExperimentTests(unittest.TestCase):
    def test_three_runs_have_the_requested_input_routes(self):
        first = experiment_run_config(1)
        second = experiment_run_config(2)
        third = experiment_run_config(3)
        self.assertEqual(first["shadow_mode"], ONBOARD_MIRROR)
        self.assertEqual(first["vicon_mode"], "pointcloud")
        self.assertFalse(first["vicon_orientation_forwarded"])
        self.assertEqual(second["shadow_mode"], INERTIAL_POSITION)
        self.assertEqual(second["vicon_mode"], "rigidbody")
        self.assertFalse(second["vicon_orientation_forwarded"])
        self.assertEqual(third["shadow_mode"], INERTIAL_POSITION)
        self.assertTrue(third["vicon_orientation_forwarded"])

    def test_mission_override_is_private_and_shadow_only(self):
        original = mission()
        prepared = prepare_contact_attitude_experiment_mission(original, 2)
        wrench = prepared["Interaction"]["config"]["wrench_interaction"]
        self.assertNotIn(
            "contact_attitude_shadow_enabled",
            original["Interaction"]["config"]["wrench_interaction"],
        )
        self.assertTrue(wrench["contact_attitude_shadow_enabled"])
        self.assertEqual(wrench["contact_attitude_shadow_mode"], INERTIAL_POSITION)
        self.assertEqual(wrench["contact_attitude_experiment_run"], 2)
        self.assertFalse(wrench["contact_attitude_vicon_orientation_forwarded"])
        self.assertTrue(wrench[
            'contact_attitude_shadow_post_release_vicon_position_fusion'
        ])
        self.assertFalse(wrench["shadow_mode"])

    def test_requires_exact_physical_lifecycle_sources(self):
        cases = (
            (
                ("Interaction", "config", "detection_method"),
                "velocity",
                "detection_method=momentum_impulse",
            ),
            (
                ("Interaction", "config", "wrench_interaction", "state_source"),
                "mocap",
                "state_source=onboard",
            ),
            (
                ("Interaction", "config", "virtual_object", "contact_detection", "source"),
                "wrench_observer",
                "source=potentiometer",
            ),
            (
                ("Interaction", "config", "virtual_object", "release_behavior", "mode"),
                "observer_brake",
                "mode=potentiometer_coast",
            ),
        )
        for path, value, message in cases:
            with self.subTest(path=path):
                configured = mission()
                target = configured
                for key in path[:-1]:
                    target = target[key]
                target[path[-1]] = value
                with self.assertRaisesRegex(ValueError, message):
                        prepare_contact_attitude_experiment_mission(configured, 2)

    def test_requires_continuous_initial_contact_arming(self):
        cases = (
            ("enabled", False, "enabled=true"),
            (
                "apply_after_each_interaction", False,
                "apply_after_each_interaction=true",
            ),
            ("max_xy_speed_m_s", 0.031, "no greater than 0.03"),
            ("stationary_dwell_s", 0.49, "at least 0.50"),
            ("max_sample_gap_s", 0.101, "no greater than 0.10"),
        )
        for key, value, message in cases:
            with self.subTest(key=key):
                configured = mission()
                configured["Interaction"]["config"]["wrench_interaction"][
                    "initial_contact_arming"
                ][key] = value
                with self.assertRaisesRegex(ValueError, message):
                    prepare_contact_attitude_experiment_mission(configured, 2)

    def test_requires_finite_positive_initial_contact_arming_limits(self):
        for key in (
            "max_xy_speed_m_s", "stationary_dwell_s", "max_sample_gap_s",
        ):
            for invalid in (0.0, float("nan"), float("inf"), True):
                with self.subTest(key=key, invalid=invalid):
                    configured = mission()
                    configured["Interaction"]["config"][
                        "wrench_interaction"
                    ]["initial_contact_arming"][key] = invalid
                    with self.assertRaisesRegex(
                            ValueError, "finite and positive"):
                        prepare_contact_attitude_experiment_mission(
                            configured, 2
                        )

    def test_requires_positive_finite_thresholds_and_dwells(self):
        paths = (
            ("contact_detection", "force_threshold_n"),
            ("contact_detection", "onset_dwell_s"),
            ("release_behavior", "force_drop_n"),
            ("release_behavior", "candidate_lead_drop_n"),
            ("release_behavior", "decrease_rate_n_s"),
            ("release_behavior", "unloaded_force_n"),
            ("release_behavior", "unloaded_dwell_s"),
            ("release_behavior", "max_sample_gap_s"),
            ("release_behavior", "candidate_stall_timeout_s"),
            ("release_behavior", "candidate_sensor_stale_timeout_s"),
        )
        for section, key in paths:
            for invalid in (0.0, float("nan"), float("inf"), True):
                with self.subTest(section=section, key=key, invalid=invalid):
                    configured = mission()
                    configured["Interaction"]["config"]["virtual_object"][
                        section
                    ][key] = invalid
                    with self.assertRaisesRegex(
                            ValueError, "finite and positive"):
                        prepare_contact_attitude_experiment_mission(
                            configured, 2
                        )

    def test_release_thresholds_have_safe_ordering(self):
        configured = mission()
        release = configured["Interaction"]["config"]["virtual_object"][
            "release_behavior"
        ]
        release["unloaded_force_n"] = 0.18
        with self.assertRaisesRegex(ValueError, "below contact"):
            prepare_contact_attitude_experiment_mission(configured, 2)

        configured = mission()
        release = configured["Interaction"]["config"]["virtual_object"][
            "release_behavior"
        ]
        release["candidate_lead_drop_n"] = 0.05
        with self.assertRaisesRegex(ValueError, "no greater than force_drop"):
            prepare_contact_attitude_experiment_mission(configured, 2)

    def test_missing_lifecycle_mapping_is_rejected(self):
        configured = mission()
        del configured["Interaction"]["config"]["virtual_object"][
            "contact_detection"
        ]
        with self.assertRaisesRegex(ValueError, "contact_detection mapping"):
            prepare_contact_attitude_experiment_mission(configured, 2)

    def test_invalid_mission_does_not_mutate_caller(self):
        original = mission()
        original["Interaction"]["config"]["virtual_object"][
            "release_behavior"
        ]["unloaded_force_n"] = float("nan")
        saved = deepcopy(original)
        with self.assertRaises(ValueError):
            prepare_contact_attitude_experiment_mission(original, 2)
        self.assertEqual(original, saved)

    def test_embedded_experiment_run_cannot_be_silently_rewritten(self):
        configured = mission()
        configured['Interaction']['config']['wrench_interaction'][
            'contact_attitude_experiment_run'
        ] = 3
        with self.assertRaisesRegex(ValueError, 'does not match'):
            prepare_contact_attitude_experiment_mission(configured, 2)

    def test_all_three_cli_routes_validate(self):
        for run in (1, 2, 3):
            with self.subTest(run=run):
                self.assertEqual(
                    validate_contact_attitude_cli(args(run)),
                    experiment_run_config(run),
                )

    def test_position_only_rigidbody_rejects_full_pose(self):
        with self.assertRaisesRegex(ValueError, "without --vicon-full-pose"):
            validate_contact_attitude_cli(args(2, vicon_full_pose=True))

    def test_pure_inertial_release_option_requires_custom_shadow_run(self):
        with self.assertRaisesRegex(ValueError, 'requires run 2 or 3'):
            validate_contact_attitude_cli(args(
                1, contact_attitude_shadow_no_vicon_position=True
            ))
        self.assertEqual(
            validate_contact_attitude_cli(args(
                2, contact_attitude_shadow_no_vicon_position=True
            )),
            experiment_run_config(2),
        )

    def test_full_pose_run_requires_rigidbody_name(self):
        with self.assertRaisesRegex(ValueError, "requires --obj-name"):
            validate_contact_attitude_cli(args(3, obj_name=None))

    def test_protocol_requires_sensor_backed_interaction(self):
        with self.assertRaisesRegex(ValueError, "requires --sense"):
            validate_contact_attitude_cli(args(1, sense=False))

    def test_protocol_requires_logging(self):
        with self.assertRaisesRegex(ValueError, "requires --log"):
            validate_contact_attitude_cli(args(1, log=False))

    def test_protocol_requires_one_translation_iteration(self):
        configured = mission()
        configured['Interaction']['action'] = 'rotation_test'
        with self.assertRaisesRegex(ValueError, 'action=translation'):
            prepare_contact_attitude_experiment_mission(configured, 2)

        configured = mission()
        configured['Interaction']['iteration'] = 2
        with self.assertRaisesRegex(ValueError, 'iteration=1'):
            prepare_contact_attitude_experiment_mission(configured, 2)

    def test_log_only_vicon_mode_is_rejected(self):
        with self.assertRaisesRegex(ValueError, 'position input must be forwarded'):
            validate_contact_attitude_cli(args(1, save_vicon=True))


if __name__ == "__main__":
    unittest.main()
