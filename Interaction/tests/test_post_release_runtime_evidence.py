import unittest

from Interaction.contact_attitude_experiment import (
    ARDUINO_TO_CF_RELEASE_CLOCK_MAPPING_BASIS,
)
from Interaction.contact_attitude_observer import CF_TIMESTAMP_MODULUS_MS
from Interaction.contact_attitude_shadow import (
    DEFAULT_IMU_QUALITY_PROVENANCE_ID,
    ContactAttitudeShadow,
    ContactAttitudeShadowConfig,
    shadow_evidence_config_kwargs,
)
from Interaction.post_release_runtime_evidence import (
    ArduinoToCfClockMapping,
)


def mapping_config(**overrides):
    unwrapped = 3 * CF_TIMESTAMP_MODULUS_MS + 1234
    result = {
        "basis": ARDUINO_TO_CF_RELEASE_CLOCK_MAPPING_BASIS,
        "calibration_id": "bench-clock-fit-2026-09-14",
        "uncertainty_ms": 0.25,
        "arduino_reference_timestamp_ms": 1000,
        "cf_reference_timestamp_ms": unwrapped % CF_TIMESTAMP_MODULUS_MS,
        "cf_reference_unwrapped_timestamp_ms": unwrapped,
        "cf_ms_per_arduino_ms": 1.001,
        "scale_error_ppm": 20.0,
        "max_abs_delta_ms": 5000,
    }
    result.update(overrides)
    return result


class ArduinoToCfClockMappingTests(unittest.TestCase):
    def test_affine_mapping_returns_raw_unwrapped_and_rounding_budget(self):
        mapping = ArduinoToCfClockMapping.from_mapping(mapping_config())

        result = mapping.map_arduino_timestamp(1123)

        expected_float = (
            mapping.cf_reference_unwrapped_timestamp_ms + 123 * 1.001
        )
        expected = round(expected_float)
        self.assertEqual(result.unwrapped_cf_timestamp_ms, expected)
        self.assertEqual(
            result.cf_timestamp_ms, expected % CF_TIMESTAMP_MODULUS_MS
        )
        self.assertAlmostEqual(
            result.mapping_uncertainty_ms,
            0.25 + 123 * 20.0e-6 + abs(expected_float - expected),
        )
        self.assertEqual(
            result.release_kwargs()["release_clock_mapping_calibration_id"],
            "bench-clock-fit-2026-09-14",
        )

    def test_arduino_uint32_wrap_is_unwrapped_about_calibration_anchor(self):
        mapping = ArduinoToCfClockMapping.from_mapping(mapping_config(
            arduino_reference_timestamp_ms=(1 << 32) - 4,
            cf_ms_per_arduino_ms=1.0,
        ))

        result = mapping.map_arduino_timestamp(3)

        self.assertEqual(result.arduino_delta_from_reference_ms, 7)
        self.assertEqual(
            result.unwrapped_cf_timestamp_ms,
            mapping.cf_reference_unwrapped_timestamp_ms + 7,
        )

    def test_outside_calibrated_interval_fails_closed(self):
        mapping = ArduinoToCfClockMapping.from_mapping(mapping_config(
            max_abs_delta_ms=50,
        ))
        with self.assertRaisesRegex(ValueError, "outside calibrated interval"):
            mapping.map_arduino_timestamp(1100)

    def test_raw_and_unwrapped_cf_anchor_must_match(self):
        with self.assertRaisesRegex(ValueError, "raw/unwrapped"):
            ArduinoToCfClockMapping.from_mapping(mapping_config(
                cf_reference_timestamp_ms=999,
            ))

    def test_missing_or_unknown_schema_fields_fail(self):
        config = mapping_config()
        config.pop("calibration_id")
        with self.assertRaisesRegex(ValueError, "missing.*calibration_id"):
            ArduinoToCfClockMapping.from_mapping(config)
        with self.assertRaisesRegex(ValueError, "unknown.*typo"):
            ArduinoToCfClockMapping.from_mapping({
                **mapping_config(), "typo": 1,
            })

    def test_untrusted_basis_and_implausible_scale_fail(self):
        with self.assertRaisesRegex(ValueError, "basis"):
            ArduinoToCfClockMapping.from_mapping(mapping_config(
                basis="host_receive_guess",
            ))
        with self.assertRaisesRegex(ValueError, "scale"):
            ArduinoToCfClockMapping.from_mapping(mapping_config(
                cf_ms_per_arduino_ms=1000.0,
            ))

    def test_independent_arduino_mapping_never_supports_authority(self):
        exact = ArduinoToCfClockMapping.from_mapping(mapping_config(
            uncertainty_ms=0.0,
            cf_ms_per_arduino_ms=1.0,
            scale_error_ppm=0.0,
        ))
        self.assertFalse(exact.supports_exact_release_epoch())
        for override in (
            {"uncertainty_ms": 0.001},
            {"cf_ms_per_arduino_ms": 1.000001},
            {"scale_error_ppm": 0.001},
        ):
            with self.subTest(override=override):
                mapping = ArduinoToCfClockMapping.from_mapping(
                    mapping_config(**override)
                )
                self.assertFalse(mapping.supports_exact_release_epoch())

    def test_nonzero_clock_uncertainty_produces_multiple_possible_imu_epochs(self):
        mapping = ArduinoToCfClockMapping.from_mapping(mapping_config(
            uncertainty_ms=0.2,
            cf_ms_per_arduino_ms=1.0,
            scale_error_ppm=0.0,
        ))
        event = mapping.map_arduino_timestamp(1007)
        center = event.unwrapped_cf_timestamp_ms

        self.assertEqual(
            event.interval_unwrapped_cf_ms(),
            (center - 0.2, center + 0.2),
        )
        self.assertEqual(
            event.possible_first_free_imu_epochs(
                [center - 2, center - 1, center, center + 1, center + 2],
                max_imu_gap_ms=2.0,
            ),
            (center, center + 1),
        )
        self.assertFalse(mapping.supports_exact_release_epoch())

    def test_release_interval_without_imu_coverage_or_across_gap_fails(self):
        mapping = ArduinoToCfClockMapping.from_mapping(mapping_config(
            uncertainty_ms=1.2,
            cf_ms_per_arduino_ms=1.0,
            scale_error_ppm=0.0,
        ))
        event = mapping.map_arduino_timestamp(1007)
        center = event.unwrapped_cf_timestamp_ms
        with self.assertRaisesRegex(ValueError, 'coverage'):
            event.possible_first_free_imu_epochs(
                [center - 1, center, center + 1],
                max_imu_gap_ms=5.0,
            )
        with self.assertRaisesRegex(ValueError, 'IMU gap'):
            event.possible_first_free_imu_epochs(
                [center - 4, center - 3, center, center + 3, center + 4],
                max_imu_gap_ms=2.0,
            )
        with self.assertRaisesRegex(ValueError, 'strictly increasing'):
            event.possible_first_free_imu_epochs(
                [center - 2, center - 2, center + 2],
                max_imu_gap_ms=5.0,
            )


class ShadowRuntimeEvidenceConfigTests(unittest.TestCase):
    def test_missing_evidence_stays_explicitly_uncalibrated(self):
        kwargs = shadow_evidence_config_kwargs({})
        config = ContactAttitudeShadowConfig(**kwargs)

        self.assertFalse(config.post_release_imu_quality_calibrated)
        self.assertEqual(
            config.post_release_imu_quality_provenance_id,
            DEFAULT_IMU_QUALITY_PROVENANCE_ID,
        )
        self.assertFalse(config.absolute_yaw_reference_certified)
        self.assertFalse(config.body_rate_measurement_calibrated)
        self.assertIsNone(config.body_rate_measurement_std_deg_s)

    def test_named_calibrations_reach_machine_readable_snapshot(self):
        kwargs = shadow_evidence_config_kwargs({
            "imu_quality": {
                "calibrated": True,
                "provenance_id": "cf231-bmi088-range-fit-v1",
                "max_abs_gyro_deg_s": 800.0,
                "max_abs_accel_g": 12.0,
                "max_gyro_step_deg_s": 200.0,
                "max_accel_step_g": 3.0,
            },
            "absolute_yaw_reference": {
                "certified": True,
                "certificate_id": "vicon-yaw-alignment-v1",
                "yaw_deg": 12.5,
            },
            "body_rate_measurement": {
                "calibrated": True,
                "std_deg_s": [0.2, 0.3, 0.4],
                "calibration_id": "cf231-bmi088-range-fit-v1",
            },
        })
        snapshot = ContactAttitudeShadow(
            ContactAttitudeShadowConfig(**kwargs)
        ).snapshot()

        self.assertTrue(snapshot["post_release_imu_quality_calibrated"])
        self.assertEqual(
            snapshot["post_release_imu_quality_provenance_id"],
            "cf231-bmi088-range-fit-v1",
        )
        self.assertTrue(snapshot["absolute_yaw_reference_certified"])
        self.assertEqual(
            snapshot["body_rate_measurement_std_deg_s"],
            [0.2, 0.3, 0.4],
        )
        self.assertEqual(
            snapshot["body_rate_measurement_calibration_id"],
            "cf231-bmi088-range-fit-v1",
        )
        self.assertEqual(snapshot["absolute_yaw_reference_yaw_deg"], 12.5)

    def test_calibrated_claims_require_named_non_datasheet_evidence(self):
        with self.assertRaisesRegex(ValueError, "datasheet"):
            shadow_evidence_config_kwargs({
                "imu_quality": {"calibrated": True},
            })
        with self.assertRaisesRegex(ValueError, "needs std"):
            shadow_evidence_config_kwargs({
                "body_rate_measurement": {"calibrated": True},
            })
        with self.assertRaisesRegex(ValueError, "needs a certificate"):
            shadow_evidence_config_kwargs({
                "absolute_yaw_reference": {"certified": True},
            })
        with self.assertRaisesRegex(ValueError, "needs a yaw value"):
            shadow_evidence_config_kwargs({
                "absolute_yaw_reference": {
                    "certified": True,
                    "certificate_id": "yaw-fit-v1",
                },
            })

    def test_joint_imu_rate_evidence_and_positive_std_are_required(self):
        common = {
            "imu_quality": {
                "calibrated": True,
                "provenance_id": "joint-imu-fit-v1",
                "max_abs_gyro_deg_s": 800.0,
                "max_abs_accel_g": 12.0,
                "max_gyro_step_deg_s": 200.0,
                "max_accel_step_g": 3.0,
            },
        }
        with self.assertRaisesRegex(ValueError, "same.*joint calibration"):
            shadow_evidence_config_kwargs({
                **common,
                "body_rate_measurement": {
                    "calibrated": True,
                    "std_deg_s": [0.2, 0.3, 0.4],
                    "calibration_id": "different-rate-fit-v1",
                },
            })
        with self.assertRaisesRegex(ValueError, "positive XYZ"):
            shadow_evidence_config_kwargs({
                **common,
                "body_rate_measurement": {
                    "calibrated": True,
                    "std_deg_s": [0.2, 0.0, 0.4],
                    "calibration_id": "joint-imu-fit-v1",
                },
            })

    def test_uncalibrated_rate_cannot_smuggle_std_into_authority(self):
        with self.assertRaisesRegex(ValueError, "uncalibrated body-rate"):
            shadow_evidence_config_kwargs({
                "body_rate_measurement": {
                    "calibrated": False,
                    "std_deg_s": [0.1, 0.1, 0.1],
                    "calibration_id": "stale-fit",
                },
            })


if __name__ == "__main__":
    unittest.main()
