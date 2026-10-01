"""Period compatibility checks for the deployed FLS logging protocol."""

import struct
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from cflib.crazyflie.log import LogConfig
from Interaction import config as interaction_config
from Interaction.log_manager import InteractionLogger


class FakeCallbacks:
    def __init__(self):
        self.callbacks = []

    def add_callback(self, callback):
        self.callbacks.append(callback)


class FakeLogConfig:
    instances = []

    def __init__(self, name, period_in_ms):
        self.name = name
        self.period_in_ms = period_in_ms
        self.variables = []
        self.data_received_cb = FakeCallbacks()
        self.started = False
        self.instances.append(self)

    def add_variable(self, name, value_type):
        self.variables.append((name, value_type))

    def start(self):
        self.started = True


class InteractionLogPeriodTests(unittest.TestCase):
    def setUp(self):
        FakeLogConfig.instances = []
        self.logger = InteractionLogger.__new__(InteractionLogger)
        self.logger.cf_log_data = None
        self.logger.cf_var_logger = None
        self.logger._cf_log_group_callback = Mock()
        self.cf = SimpleNamespace(log=SimpleNamespace(add_config=Mock()))

    def initialize(self, config, default_period=10):
        with patch('Interaction.log_manager.LogConfig', FakeLogConfig):
            self.logger.init_cf_logger(
                self.cf, config, cf_log_period=default_period
            )
        return FakeLogConfig.instances

    def test_default_period_matches_ordinary_logger_compensation(self):
        blocks = self.initialize({
            'STATE': {'stateEstimate.x': {'type': 'float', 'data': []}},
        })

        self.assertEqual(len(blocks), 1)
        self.assertEqual(blocks[0].period_in_ms, 100)
        self.assertEqual(
            blocks[0].variables, [('stateEstimate.x', 'float')]
        )
        self.assertTrue(blocks[0].started)

    def test_firmware_brake_uses_small_status_log_without_pi_imu_stream(self):
        mission = {'Interaction': {'config': {'wrench_interaction': {
            'firmware_auto_brake': {'enabled': True},
        }}}}
        selected = interaction_config.log_vars_for_mission(mission)
        self.assertIn('FIRMWARE_BRAKE', selected)
        self.assertNotIn('GYRO_1KHZ', selected)
        for optional in ('YAW_CTL', 'POS_CTL_I_D', 'POS_VEL_CTL',
                         'ATT_RATE_CTL'):
            self.assertNotIn(optional, selected)
        self.assertEqual(set(selected), {
            'FIRMWARE_KIN', 'FIRMWARE_ACT', 'FIRMWARE_BRAKE',
        })
        blocks = self.initialize(selected)
        by_name = {block.name: block for block in blocks}
        self.assertEqual(by_name['FIRMWARE_KIN'].period_in_ms, 100)
        self.assertEqual(by_name['FIRMWARE_ACT'].period_in_ms, 100)
        self.assertEqual(
            sum({'int16_t': 2, 'uint32_t': 4}[kind]
                for _, kind in by_name['FIRMWARE_KIN'].variables), 22)
        self.assertEqual(
            sum({'int16_t': 2, 'uint16_t': 2, 'float': 4}[kind]
                for _, kind in by_name['FIRMWARE_ACT'].variables), 18)
        self.assertEqual(by_name['FIRMWARE_BRAKE'].period_in_ms, 1000)
        self.assertEqual(by_name['FIRMWARE_BRAKE'].variables, [
            ('hlCommander.pRelReady', 'uint8_t'),
            ('hlCommander.pRelAutoSt', 'uint8_t'),
            ('hlCommander.pRelAbort', 'uint8_t'),
            ('hlCommander.pRelAutoTime', 'uint8_t'),
            ('hlCommander.pRelEvtVer', 'uint8_t'),
            ('hlCommander.pRelMode', 'uint8_t'),
            ('hlCommander.pRelRejR', 'uint8_t'),
            ('hlCommander.pRelRejD', 'uint8_t'),
            ('hlCommander.pRelTau', 'FP16'),
            ('hlCommander.scV', 'FP16'),
            ('hlCommander.scCmd', 'FP16'),
            ('hlCommander.scPeak', 'FP16'),
            ('hlCommander.pRelStale0', 'uint8_t'),
        ])
        sizes = {'uint8_t': 1, 'FP16': 2}
        self.assertEqual(sum(
            sizes[kind] for _, kind in by_name['FIRMWARE_BRAKE'].variables
        ), 17)

    def test_brake_log_period_is_opt_in_and_rejects_bad_values(self):
        def selected(period):
            mission = {'Interaction': {'config': {'wrench_interaction': {
                'firmware_auto_brake': {'enabled': True,
                                        'brake_log_period_ms': period},
            }}}}
            return interaction_config.firmware_brake_log_vars(mission)

        self.assertEqual(selected(20)['log_period_ms'], 20)
        # Absent, non-integer, boolean and out-of-range values keep 100 ms.
        for bad in (None, 'fast', True, 5, 9.5):
            self.assertEqual(selected(bad)['log_period_ms'], 100)
        self.assertEqual(
            interaction_config.firmware_brake_log_vars({})['log_period_ms'], 100)
        # The shared default is never mutated by an override.
        self.assertEqual(
            interaction_config.FIRMWARE_BRAKE_LOG_VARS['log_period_ms'], 100)

    def test_firmware_optional_logs_can_be_enabled_without_changing_legacy(self):
        wrench = {
            'firmware_auto_brake': {
                'enabled': True, 'include_optional_log_groups': True,
            },
        }
        mission = {'Interaction': {'config': {'wrench_interaction': wrench}}}
        selected = interaction_config.log_vars_for_mission(mission)
        for optional in ('YAW_CTL', 'POS_CTL_I_D', 'POS_VEL_CTL',
                         'ATT_RATE_CTL'):
            self.assertIn(optional, selected)
        self.assertIn('FIRMWARE_BRAKE', selected)

        wrench['firmware_auto_brake']['include_optional_log_groups'] = 'yes'
        with self.assertRaisesRegex(ValueError, 'include_optional_log_groups'):
            interaction_config.log_vars_for_mission(mission)

    def test_firmware_compressed_logs_have_scoped_legacy_fallback(self):
        wrench = {'firmware_auto_brake': {
            'enabled': True, 'compressed_state_logs': False,
        }}
        mission = {'Interaction': {'config': {'wrench_interaction': wrench}}}
        selected = interaction_config.log_vars_for_mission(mission)
        self.assertEqual(set(selected), {
            'VEL_ORI', 'POS_ACC', 'RATE_EST', 'MOT_BAT', 'FIRMWARE_BRAKE',
        })
        wrench['firmware_auto_brake']['compressed_state_logs'] = 'yes'
        with self.assertRaisesRegex(ValueError, 'compressed_state_logs'):
            interaction_config.log_vars_for_mission(mission)

    def test_half_precision_response_readback_preserves_prearm_tolerance(self):
        for response_s in (0.02, 0.08, 0.14, 0.20):
            readback = struct.unpack('<e', struct.pack('<e', response_s))[0]
            self.assertLess(abs(readback-response_s), 0.001)

    def test_enabled_yaw_command_model_keeps_yaw_log_required(self):
        mission = {'Interaction': {'config': {'wrench_interaction': {
            'firmware_auto_brake': {'enabled': True},
            'motor_model': {'yaw_command_model': {'enabled': True}},
        }}}}
        selected = interaction_config.log_vars_for_mission(mission)
        self.assertIn('YAW_CTL', selected)
        self.assertTrue(interaction_config.onboard_yaw_log_required(mission))

    def test_installed_cflib_encodes_compensated_period_as_legacy_byte(self):
        self.assertEqual(LogConfig('STATE', period_in_ms=100).period, 10)

    def test_crazysim_uses_upstream_ten_millisecond_period_units(self):
        self.logger.args = SimpleNamespace(crazysim=True)

        blocks = self.initialize({
            'STATE': {'stateEstimate.x': {'type': 'float', 'data': []}},
            'REFERENCE': {
                'log_period_ms': 20,
                'kalmanPRel.q0': {'type': 'float', 'data': []},
            },
        })

        self.assertEqual(blocks[0].period_in_ms, 10)
        self.assertEqual(blocks[1].period_in_ms, 20)

    def test_crazysim_reference_state_is_100_hz_and_status_proves_imu_rate(self):
        self.logger.args = SimpleNamespace(crazysim=True)
        blocks = self.initialize(
            interaction_config.log_vars_for_crazysim({}), default_period=10
        )
        by_name = {block.name: block for block in blocks}
        self.assertEqual(by_name['P_REL_STATUS'].period_in_ms, 20)
        self.assertEqual(by_name['P_REL_ATT'].period_in_ms, 10)
        self.assertEqual(by_name['P_REL_ACC'].period_in_ms, 10)
        self.assertEqual(by_name['P_REL_HLC'].period_in_ms, 20)
        self.assertEqual(by_name['ATT_DES'].period_in_ms, 10)
        self.assertEqual(
            [name for name, _ in by_name['ATT_DES'].variables],
            ['controller.roll', 'controller.pitch'],
        )
        status_names = {
            name for name, _ in by_name['P_REL_STATUS'].variables
        }
        self.assertIn('kalmanPRel.imuN', status_names)
        self.assertIn('kalmanPRel.maxGapUs', status_names)
        self.assertIn('kalmanPRel.brkReady', status_names)
        handoff_names = {
            name for name, _ in by_name['P_REL_HLC'].variables
        }
        self.assertIn('kalmanPRel.transEpoch', handoff_names)
        self.assertIn('hlCommander.pRelUsed', handoff_names)

    def test_group_override_is_consumed_without_mutating_source(self):
        config = {
            'STATE': {
                'log_period_ms': 10,
                'stateEstimate.x': {'type': 'float', 'data': []},
            },
        }
        blocks = self.initialize(config, default_period=20)

        self.assertEqual(blocks[0].period_in_ms, 100)
        self.assertNotIn('log_period_ms', self.logger.cf_log_data['STATE'])
        self.assertEqual(config['STATE']['log_period_ms'], 10)
        self.assertEqual(
            blocks[0].variables, [('stateEstimate.x', 'float')]
        )

    def test_gyro_diagnostic_group_requests_one_millisecond_period(self):
        blocks = self.initialize({
            'GYRO_1KHZ': interaction_config.GYRO_1KHZ,
        })
        gyro = next(block for block in blocks if block.name == 'GYRO_1KHZ')

        # InteractionLogger compensates for the deployed firmware's 1 ms
        # legacy period units.  cflib therefore receives 10 ms and encodes
        # byte 1, which that firmware executes once per millisecond.
        self.assertEqual(gyro.period_in_ms, 10)
        self.assertEqual(LogConfig('GYRO_1KHZ', 10).period, 1)
        self.assertEqual(
            gyro.variables,
            [
                ('gyro.x', 'float'),
                ('gyro.y', 'float'),
                ('gyro.z', 'float'),
            ],
        )
        self.assertEqual(
            interaction_config.GYRO_1KHZ['log_period_ms'], 1
        )

    def test_ordinary_interaction_opens_no_one_kilohertz_log(self):
        blocks = self.initialize(interaction_config.log_vars_for_mission({
            'Interaction': {'config': {'wrench_interaction': {}}},
        }))
        self.assertNotIn('GYRO_1KHZ', {block.name for block in blocks})
        self.assertTrue(all(block.period_in_ms >= 100 for block in blocks))

    def test_packed_contact_imu_requests_one_millisecond_period(self):
        blocks = self.initialize(interaction_config.log_vars_for_mission({
            'Interaction': {'config': {'wrench_interaction': {
                'contact_attitude_shadow_enabled': True,
            }}}
        }))
        packed = next(block for block in blocks if block.name == 'GYRO_1KHZ')
        self.assertEqual(packed.period_in_ms, 10)
        self.assertEqual(LogConfig('GYRO_1KHZ', 10).period, 1)
        self.assertEqual(packed.variables, [
            ('contactImu.gx', 'FP16'),
            ('contactImu.gy', 'FP16'),
            ('contactImu.gz', 'FP16'),
            ('contactImu.ax', 'FP16'),
            ('contactImu.ay', 'FP16'),
            ('contactImu.az', 'FP16'),
            ('contactImu.px', 'FP16'),
            ('contactImu.py', 'FP16'),
            ('contactImu.pz', 'FP16'),
            ('contactImu.vx', 'FP16'),
            ('contactImu.vy', 'FP16'),
            ('contactImu.vz', 'FP16'),
            ('contactImu.epoch', 'uint16_t'),
        ])
        self.assertEqual(
            interaction_config.CONTACT_IMU_1KHZ['log_period_ms'], 1
        )
        sizes = {'FP16': 2, 'uint16_t': 2}
        self.assertEqual(sum(sizes[kind] for _, kind in packed.variables), 26)
        self.assertFalse(any(block.name == 'ACC_ALIGN' for block in blocks))
        self.assertFalse(any(
            block.name == 'CONTACT_STATE_SEED' for block in blocks
        ))

    def test_shadow_imu_groups_are_default_off_and_explicitly_opt_in(self):
        disabled = interaction_config.log_vars_for_mission({
            'Interaction': {'config': {}}
        })
        self.assertNotIn('GYRO_1KHZ', disabled)
        self.assertNotIn('ACC_ALIGN', disabled)
        enabled = interaction_config.log_vars_for_mission({
            'Interaction': {'config': {'wrench_interaction': {
                'contact_attitude_shadow_enabled': True,
            }}}
        })
        self.assertNotIn('ACC_ALIGN', interaction_config.LOG_VARS)
        self.assertNotIn('ACC_ALIGN', enabled)
        self.assertNotIn('CONTACT_STATE_SEED', enabled)
        self.assertEqual(
            tuple(enabled['GYRO_1KHZ']),
            ('log_period_ms', 'contactImu.gx', 'contactImu.gy',
             'contactImu.gz', 'contactImu.ax', 'contactImu.ay',
             'contactImu.az', 'contactImu.px', 'contactImu.py',
             'contactImu.pz', 'contactImu.vx', 'contactImu.vy',
             'contactImu.vz', 'contactImu.epoch'),
        )
        self.assertIs(disabled, interaction_config.LOG_VARS)
        top_level_only = interaction_config.log_vars_for_mission({
            'Interaction': {'config': {
                'contact_attitude_shadow_enabled': True,
                'wrench_interaction': {},
            }}
        })
        self.assertNotIn('ACC_ALIGN', top_level_only)

    def test_nested_shadow_mode_selects_only_required_streams(self):
        mirror = interaction_config.log_vars_for_mission({
            'Interaction': {'config': {'wrench_interaction': {
                'contact_attitude_shadow_enabled': True,
                'contact_attitude_shadow_mode': 'onboard_mirror',
            }}}
        })
        inertial = interaction_config.log_vars_for_mission({
            'Interaction': {'config': {'wrench_interaction': {
                'contact_attitude_shadow_enabled': True,
                'contact_attitude_shadow_mode': 'inertial_position',
            }}}
        })
        self.assertIs(mirror, interaction_config.LOG_VARS)
        self.assertNotIn('GYRO_1KHZ', mirror)
        self.assertNotIn('ACC_ALIGN', mirror)
        self.assertNotIn('ACC_ALIGN', inertial)
        self.assertNotIn('CONTACT_STATE_SEED', inertial)
        self.assertIn('contactImu.ax', inertial['GYRO_1KHZ'])
        self.assertIn('contactImu.vx', inertial['GYRO_1KHZ'])

    def test_unknown_shadow_mode_is_rejected_pre_arm(self):
        with self.assertRaisesRegex(ValueError, 'contact_attitude_shadow_mode'):
            interaction_config.log_vars_for_mission({
                'Interaction': {'config': {'wrench_interaction': {
                    'contact_attitude_shadow_enabled': True,
                    'contact_attitude_shadow_mode': 'guess',
                }}}
            })

    def test_shadow_logging_flag_rejects_truthy_non_boolean_values(self):
        with self.assertRaisesRegex(ValueError, 'must be boolean'):
            interaction_config.log_vars_for_mission({
                'Interaction': {'config': {'wrench_interaction': {
                    'contact_attitude_shadow_enabled': 'true',
                }}}
            })


if __name__ == '__main__':
    unittest.main()
