"""Two-block firmware-owned interaction telemetry stays time-aligned."""

from types import SimpleNamespace
from collections import defaultdict, deque
import threading
import unittest
from unittest.mock import patch

import numpy as np
from cflib.utils.encoding import compress_quaternion

from controller import Controller
from Interaction.compressed_state_logs import decode_kinematic_packet
from Interaction.compressed_state_logs import MAX_PAIR_SKEW_S
from Interaction.contact_attitude_observer import quaternion_from_native_rpy
from Interaction.interactions import InteractionsControl
from Interaction.log_manager import InteractionLogger


def compressed_packets():
    w, x, y, z = quaternion_from_native_rpy(0.15, -0.10, 0.40)
    kin = {
        'stateEstimateZ.x': 1200,
        'stateEstimateZ.y': -500,
        'stateEstimateZ.z': 900,
        'stateEstimateZ.vx': 1300,
        'stateEstimateZ.vy': -250,
        'stateEstimateZ.vz': 0,
        'stateEstimateZ.quat': int(compress_quaternion((x, y, z, w))),
        'stateEstimateZ.rateRoll': 100,
        'stateEstimateZ.ratePitch': -200,
        'stateEstimateZ.rateYaw': 300,
    }
    act = {
        'stateEstimateZ.ax': 100,
        'stateEstimateZ.ay': -200,
        'stateEstimateZ.az': 9810,
        'motor.m1': 30000,
        'motor.m2': 30001,
        'motor.m3': 30002,
        'motor.m4': 30003,
        'pm.vbat': 8.0,
    }
    return kin, act


def buffered_logs():
    logs = InteractionLogger.__new__(InteractionLogger)
    logs.cf_log_data = {'FIRMWARE_KIN': {}, 'FIRMWARE_ACT': {}}
    logs.cf_log_group_times = {}
    logs.cf_log_group_packets = defaultdict(lambda: deque(maxlen=1000))
    logs.cf_log_group_packet_metadata = defaultdict(lambda: deque(maxlen=1000))
    logs.cf_log_packet_lock = threading.Lock()
    return logs


def publish(logs, group, epoch, host_time, data):
    with logs.cf_log_packet_lock:
        logs.cf_log_group_times[group] = host_time
        logs.cf_log_group_packets[group].append({**data, 'time': host_time})
        logs.cf_log_group_packet_metadata[group].append({
            'time': host_time, 'cf_timestamp_ms': epoch,
            'cf_timestamp_basis': 'crazyflie_log_transport_tick_v1',
        })


class PairedLogManager:
    def __init__(self, *, pair_skew=0.003):
        self.kin, self.act = compressed_packets()
        self.cf_log_data = {'FIRMWARE_KIN': {}, 'FIRMWARE_ACT': {}}
        self.pair_skew = pair_skew

    def get_latest_group_log_time(self, group):
        return 1000.0 if group == 'FIRMWARE_KIN' else None

    def get_latest_group_log_data(self, group):
        return dict(self.kin if group == 'FIRMWARE_KIN' else self.act)

    def get_nearest_group_log_data(self, group, timestamp):
        if group == 'FIRMWARE_KIN':
            return {**self.kin, 'time': timestamp}, 0.0
        return None, None

    def get_nearest_group_log_metadata(self, group, timestamp):
        if group == 'FIRMWARE_KIN':
            return {'cf_timestamp_ms': 5000,
                    'cf_timestamp_basis': 'crazyflie_log_transport_tick_v1'}, 0.0
        return None, None

    def get_nearest_group_log_data_by_cf_timestamp(self, group, timestamp):
        if group == 'FIRMWARE_ACT' and timestamp == 5000:
            return {**self.act, 'time': 1000.003}, self.pair_skew
        return None, None


class CompressedStateLogTests(unittest.TestCase):
    def control(self, logs):
        control = InteractionsControl.__new__(InteractionsControl)
        control.log_manager = logs
        control.mission = {'Interaction': {'config': {'wrench_interaction': {
            'firmware_auto_brake': {'enabled': True},
        }}}}
        return control

    def test_decodes_world_velocity_and_legacy_pitch_direction(self):
        kin, _ = compressed_packets()
        position, velocity, attitude, rates = decode_kinematic_packet(kin)
        np.testing.assert_allclose(position, [1.2, -0.5, 0.9])
        np.testing.assert_allclose(velocity, [1.3, -0.25, 0.0])
        np.testing.assert_allclose(attitude, [0.15, 0.10, 0.40], atol=0.002)
        np.testing.assert_allclose(rates, [0.1, -0.2, 0.3])

    def test_pairs_two_blocks_by_firmware_tick(self):
        state = self.control(PairedLogManager())._get_synchronized_onboard_wrench_state()
        self.assertIsNotNone(state)
        np.testing.assert_allclose(state['velocity'], [1.3, -0.25, 0.0])
        self.assertEqual(state['cf_timestamp_ms'], 5000)
        self.assertEqual(state['motor_skew_s'], 0.003)
        self.assertEqual(state['motor_state']['pm.vbat'], 8.0)
        self.assertIsNone(state['yaw_control_command'])

    def test_missing_cycle_or_bad_quaternion_fails_closed(self):
        logs = PairedLogManager(pair_skew=0.010)
        self.assertIsNone(
            self.control(logs)._get_synchronized_onboard_wrench_state())
        logs.pair_skew = 0.0
        logs.kin['stateEstimateZ.quat'] = -1
        self.assertIsNone(
            self.control(logs)._get_synchronized_onboard_wrench_state())

    def test_recorded_half_pair_uses_previous_complete_then_advances(self):
        # Exact receipt/tick timing of the 2026-09-18 16:14:56 failure.
        logs = buffered_logs()
        kin, act = compressed_packets()
        control = self.control(logs)
        publish(logs, 'FIRMWARE_KIN', 36123, 1789773296.720467, kin)
        publish(logs, 'FIRMWARE_ACT', 36124, 1789773296.725514, act)
        publish(logs, 'FIRMWARE_KIN', 36134, 1789773296.7331688,
                {**kin, 'stateEstimateZ.vx': 114})
        state = control._get_synchronized_onboard_wrench_state()
        self.assertEqual(state['cf_timestamp_ms'], 36123)
        self.assertEqual(state['time'], 1789773296.720467)
        self.assertEqual(state['motor_skew_s'], .001)
        # Reading the half pair twice must not fabricate another sample.
        repeated = control._get_synchronized_onboard_wrench_state()
        self.assertEqual(repeated['time'], state['time'])
        publish(logs, 'FIRMWARE_ACT', 36134, 1789773296.7335045, act)
        state = control._get_synchronized_onboard_wrench_state()
        self.assertEqual(state['cf_timestamp_ms'], 36134)
        self.assertEqual(state['time'], 1789773296.7331688)
        self.assertEqual(state['motor_skew_s'], 0.)
        self.assertAlmostEqual(state['velocity'][0], .114)

    def test_actuator_first_does_not_force_incomplete_new_pair(self):
        logs = buffered_logs()
        kin, act = compressed_packets()
        publish(logs, 'FIRMWARE_KIN', 5000, 1000., kin)
        publish(logs, 'FIRMWARE_ACT', 5000, 1000.001, act)
        publish(logs, 'FIRMWARE_ACT', 5010, 1000.011, act)
        state = self.control(logs)._get_synchronized_onboard_wrench_state()
        self.assertEqual(state['cf_timestamp_ms'], 5000)
        self.assertEqual(state['motor_skew_s'], 0.)

    def test_unmatched_traffic_never_refreshes_old_complete_state(self):
        logs = buffered_logs()
        kin, act = compressed_packets()
        publish(logs, 'FIRMWARE_KIN', 5000, 1000., kin)
        publish(logs, 'FIRMWARE_ACT', 5000, 1000.001, act)
        for step in range(1, 101):
            publish(logs, 'FIRMWARE_KIN', 5000 + step * 10, 1000. + step * .01, kin)
        state = self.control(logs)._get_synchronized_onboard_wrench_state()
        self.assertEqual(state['time'], 1000.)
        self.assertEqual(1001. - state['time'], 1.)
        self.assertEqual(state['cf_timestamp_ms'], 5000)
        # State age is still computed by the existing caller; no timestamp is
        # replaced with the newest KIN or the current wall clock.

    def test_no_pair_still_fails_and_ten_ms_gate_is_not_relaxed(self):
        logs = buffered_logs()
        kin, act = compressed_packets()
        publish(logs, 'FIRMWARE_KIN', 5010, 1000.010, kin)
        self.assertIsNone(self.control(logs)._get_synchronized_onboard_wrench_state())
        publish(logs, 'FIRMWARE_ACT', 5000, 1000.010, act)
        self.assertIsNone(self.control(logs)._get_synchronized_onboard_wrench_state())

    def test_pair_clock_wrap_and_independent_host_delivery(self):
        logs = buffered_logs()
        kin, act = compressed_packets()
        publish(logs, 'FIRMWARE_KIN', 0xfffffe, 1000., kin)
        publish(logs, 'FIRMWARE_ACT', 1, 1000.020, act)
        publish(logs, 'FIRMWARE_KIN', 12, 1000.030, kin)
        state = self.control(logs)._get_synchronized_onboard_wrench_state()
        self.assertEqual(state['cf_timestamp_ms'], 0xfffffe)
        self.assertEqual(state['motor_skew_s'], .003)
        self.assertEqual(state['time'], 1000.)

    def test_invalid_complete_pair_is_not_hidden_by_older_valid_pair(self):
        logs = buffered_logs()
        kin, act = compressed_packets()
        for epoch in (5000, 5010):
            publish(logs, 'FIRMWARE_KIN', epoch, 1000. + (epoch-5000)/1000.,
                    kin if epoch == 5000 else {**kin, 'stateEstimateZ.quat': -1})
            publish(logs, 'FIRMWARE_ACT', epoch, 1000. + (epoch-5000)/1000., act)
        self.assertIsNone(self.control(logs)._get_synchronized_onboard_wrench_state())

    def test_pair_copies_and_metadata_consistency(self):
        logs = buffered_logs()
        kin, act = compressed_packets()
        publish(logs, 'FIRMWARE_KIN', 5000, 1000., kin)
        publish(logs, 'FIRMWARE_ACT', 5000, 1000.001, act)
        pair = logs.get_latest_paired_group_log_data(
            'FIRMWARE_KIN', 'FIRMWARE_ACT', max_skew_s=MAX_PAIR_SKEW_S)
        pair[0]['stateEstimateZ.vx'] = 999
        pair[1]['cf_timestamp_ms'] = 0
        pair[2]['motor.m1'] = 0
        state = self.control(logs)._get_synchronized_onboard_wrench_state()
        self.assertEqual(state['cf_timestamp_ms'], 5000)
        self.assertEqual(state['motor_state']['motor.m1'], 30000)
        logs.cf_log_group_packet_metadata['FIRMWARE_ACT'].clear()
        self.assertIsNone(self.control(logs)._get_synchronized_onboard_wrench_state())

    def test_prearm_accepts_only_decoded_synchronized_pair(self):
        controller = Controller.__new__(Controller)
        controller._uses_onboard_wrench_state = lambda: True
        controller.log_manager = PairedLogManager()
        controller.mission = self.control(controller.log_manager).mission
        controller.args = SimpleNamespace(contact_attitude_run=None)
        controller.verify_onboard_wrench_logging()

        controller.log_manager.pair_skew = 0.010
        with patch('controller.time.monotonic', side_effect=(0.0, 0.0, 5.1)):
            with patch('controller.time.sleep'):
                with self.assertRaisesRegex(RuntimeError, 'pair skew'):
                    controller.verify_onboard_wrench_logging()

    def test_landing_position_uses_compressed_millimetres(self):
        kin, _ = compressed_packets()
        logger = InteractionLogger.__new__(InteractionLogger)
        logger.cf_log_data = {'FIRMWARE_KIN': {
            name: {'data': [value]} for name, value in kin.items()
        }}
        self.assertEqual(
            logger.get_latest_cf_log_data('VEL_POS', 'stateEstimate.y'),
            -0.5)


if __name__ == '__main__':
    unittest.main()
