import ast
from pathlib import Path
from types import SimpleNamespace
import time
import unittest
from unittest.mock import Mock

from Interaction.contact_attitude_prearm import ContactAttitudePrearmHandle
from Interaction.log_manager import (
    CONTACT_SOURCE_TIMESTAMP_BASIS,
    CfLogPacket,
    MocapFramePacket,
)


ROOT = Path(__file__).resolve().parents[2]


def controller_methods(names, **namespace):
    tree = ast.parse((ROOT/'controller.py').read_text())
    controller = next(
        node for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == 'Controller'
    )
    selected = [
        node for node in controller.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    exec(
        compile(ast.Module(body=selected, type_ignores=[]), 'controller.py',
                'exec'),
        namespace,
    )
    return namespace


class FakeLogManager:
    def __init__(self, fail_mocap=False):
        self.groups = {}
        self.cf_listeners = []
        self.mocap_listeners = []
        self.fail_mocap = fail_mocap

    def add_log_group(self, name):
        self.groups[name] = []

    def add_log_entry(self, group, entry):
        self.groups.setdefault(group, []).append(entry)

    def add_cf_packet_listener(self, listener):
        self.cf_listeners.append(listener)

        def unsubscribe():
            if listener in self.cf_listeners:
                self.cf_listeners.remove(listener)
        return unsubscribe

    def add_mocap_frame_listener(self, listener):
        if self.fail_mocap:
            raise RuntimeError('mocap registration failed')
        self.mocap_listeners.append(listener)

        def unsubscribe():
            if listener in self.mocap_listeners:
                self.mocap_listeners.remove(listener)
        return unsubscribe


def cf_packet(
        group, sequence=0, host_monotonic_s=10.0,
        source_provenance_valid=True):
    return CfLogPacket(
        sequence=sequence,
        group=group,
        cf_timestamp_ms=sequence,
        host_receive_time_s=10.0,
        data={},
        host_receive_monotonic_s=host_monotonic_s,
        transport_cf_timestamp_ms=(
            sequence if source_provenance_valid else None
        ),
        source_cf_timestamp_basis=(
            CONTACT_SOURCE_TIMESTAMP_BASIS
            if source_provenance_valid else None
        ),
        source_snapshot_atomic=source_provenance_valid,
    )


def mocap_packet(
        *, sequence=0, host_monotonic_s=10.0,
        orientation_forwarded=False):
    return MocapFramePacket(
        sequence=sequence,
        group='frames',
        host_receive_time_s=20.0,
        data={
            'tvec': [0.0, 0.0, 1.0],
            'quat': [0.0, 0.0, 0.0, 1.0],
            'position_forwarded_to_onboard_ekf': True,
            'orientation_forwarded_to_onboard_ekf': orientation_forwarded,
        },
        host_receive_monotonic_s=host_monotonic_s,
    )


class ContactAttitudePrearmTests(unittest.TestCase):
    def build(self, **changes):
        arguments = dict(
            log_manager=FakeLogManager(),
            mode='inertial_position',
            experiment_run=2,
            vicon_orientation_forwarded=False,
            alignment_yaw_deg=0.0,
        )
        arguments.update(changes)
        return ContactAttitudePrearmHandle.build(**arguments)

    def test_inactive_listener_proves_readiness_without_growing_queue(self):
        handle = self.build()
        handle.receive_cf_packet(cf_packet('GYRO_1KHZ'))

        self.assertEqual(len(handle.shadow._queue), 0)
        self.assertEqual(handle.missing_seen_groups(), ())
        handle.mark_verified()
        shadow = handle.activate(
            mode='inertial_position', experiment_run=2,
            vicon_orientation_forwarded=False,
        )
        handle.receive_cf_packet(cf_packet('GYRO_1KHZ', 4))
        self.assertIs(shadow, handle.shadow)
        self.assertEqual(len(handle.shadow._queue), 1)

    def test_missing_group_or_protocol_mismatch_fails_closed(self):
        handle = self.build()
        with self.assertRaisesRegex(RuntimeError, 'has not observed'):
            handle.mark_verified()

        handle.receive_cf_packet(cf_packet('GYRO_1KHZ'))
        handle.mark_verified()
        with self.assertRaisesRegex(RuntimeError, 'does not match'):
            handle.activate(
                mode='inertial_position', experiment_run=3,
                vicon_orientation_forwarded=False,
            )

    def test_unproven_firmware_source_timestamp_fails_closed(self):
        handle = self.build()
        handle.receive_cf_packet(cf_packet(
            'GYRO_1KHZ', source_provenance_valid=False
        ))

        self.assertEqual(handle.missing_seen_groups(), ('GYRO_1KHZ',))
        with self.assertRaisesRegex(
                RuntimeError,
                'unproven firmware source timestamps: GYRO_1KHZ'):
            handle.mark_verified()

    def test_registration_failure_rolls_back_first_listener(self):
        logger = FakeLogManager(fail_mocap=True)
        with self.assertRaisesRegex(RuntimeError, 'registration failed'):
            self.build(log_manager=logger)
        self.assertEqual(logger.cf_listeners, [])
        self.assertEqual(logger.mocap_listeners, [])

    def test_close_is_idempotent_and_stops_delivery(self):
        handle = self.build()
        logger = handle.log_manager
        handle.receive_cf_packet(cf_packet('GYRO_1KHZ'))
        handle.mark_verified()
        handle.activate(
            mode='inertial_position', experiment_run=2,
            vicon_orientation_forwarded=False,
        )
        handle.close()
        handle.close()

        self.assertTrue(handle.closed)
        self.assertEqual(logger.cf_listeners, [])
        self.assertEqual(logger.mocap_listeners, [])
        with self.assertRaisesRegex(RuntimeError, 'already closed'):
            handle.activate(
                mode='inertial_position', experiment_run=2,
                vicon_orientation_forwarded=False,
            )

    def test_close_attempts_every_unsubscriber_after_one_fails(self):
        handle = self.build()
        cleanup_calls = []
        failure_count = 0

        def succeeds():
            cleanup_calls.append('succeeded')

        def fails_once():
            nonlocal failure_count
            cleanup_calls.append('failed')
            failure_count += 1
            if failure_count == 1:
                raise RuntimeError('unsubscribe failed')

        handle._unsubscribers = [succeeds, fails_once]
        with self.assertRaisesRegex(RuntimeError, 'unsubscribe failed'):
            handle.close()

        self.assertEqual(cleanup_calls, ['failed', 'succeeded'])
        self.assertFalse(handle.closed)
        handle.close()
        self.assertEqual(cleanup_calls, ['failed', 'succeeded', 'failed'])
        self.assertTrue(handle.closed)

    def test_freshness_names_stale_cf_group(self):
        handle = self.build()
        handle.receive_cf_packet(cf_packet(
            'GYRO_1KHZ', host_monotonic_s=10.0
        ))
        handle.receive_mocap_frame(mocap_packet(host_monotonic_s=10.29))
        handle.mark_verified()

        with self.assertRaisesRegex(
                RuntimeError, 'missing or stale: GYRO_1KHZ'):
            handle.assert_fresh(now_s=10.30, max_age_s=0.25)

    def test_freshness_rejects_stale_mocap(self):
        handle = self.build()
        handle.receive_cf_packet(cf_packet(
            'GYRO_1KHZ', host_monotonic_s=10.29
        ))
        handle.receive_mocap_frame(mocap_packet(host_monotonic_s=10.0))
        handle.mark_verified()

        with self.assertRaisesRegex(
                RuntimeError, 'Vicon stream is missing or stale'):
            handle.assert_fresh(now_s=10.30, max_age_s=0.25)

    def test_mission_embedded_run_requires_matching_cli_selection(self):
        namespace = controller_methods({
            'prepare_contact_attitude_experiment_mission'
        })
        controller = SimpleNamespace(
            args=SimpleNamespace(contact_attitude_run=None),
            mission={
                'Interaction': {
                    'config': {
                        'wrench_interaction': {
                            'contact_attitude_experiment_run': 2,
                        },
                    },
                },
            },
            missions=[],
        )

        with self.assertRaisesRegex(ValueError, '--contact-attitude-run'):
            namespace['prepare_contact_attitude_experiment_mission'](
                controller
            )

    def test_repeated_controller_setup_replaces_prearm_listeners(self):
        namespace = controller_methods(
            {'setup_contact_attitude_shadow'}, time=time
        )
        logs = FakeLogManager()
        controller = SimpleNamespace(
            args=SimpleNamespace(contact_attitude_run=2, drone_id='cf1'),
            mission={
                'Interaction': {
                    'config': {
                        'wrench_interaction': {
                            'contact_attitude_shadow_mode': (
                                'inertial_position'
                            ),
                            'contact_attitude_experiment_run': 2,
                            'contact_attitude_vicon_orientation_forwarded': (
                                False
                            ),
                        },
                    },
                },
                'drones': {'cf1': {'target': [0.0, 0.0, 1.0, 0.0]}},
            },
            log_manager=logs,
            contact_attitude_shadow=None,
            _contact_attitude_shadow_prearm=None,
        )

        namespace['setup_contact_attitude_shadow'](controller)
        first = controller._contact_attitude_shadow_prearm
        namespace['setup_contact_attitude_shadow'](controller)
        second = controller._contact_attitude_shadow_prearm

        self.assertTrue(first.closed)
        self.assertIsNot(first, second)
        self.assertEqual(len(logs.cf_listeners), 1)
        self.assertEqual(len(logs.mocap_listeners), 1)
        self.assertIs(logs.contact_attitude_shadow_prearm, second)
        self.assertIs(controller.contact_attitude_shadow, second.shadow)
        second.close()

    def test_arm_freshness_failure_sends_no_arming_request(self):
        namespace = controller_methods(
            {'arm'}, time=Mock(), logger=Mock()
        )
        controller = SimpleNamespace(
            args=SimpleNamespace(ground_test=False, skip_arm=False),
            verify_contact_attitude_final_prearm_ready=Mock(
                side_effect=RuntimeError('stale pre-arm input')
            ),
            cf=Mock(),
            log_manager=Mock(),
        )

        with self.assertRaisesRegex(RuntimeError, 'stale pre-arm input'):
            namespace['arm'](controller)
        controller.cf.platform.send_arming_request.assert_not_called()
        controller.log_manager.add_log_entry.assert_not_called()


if __name__ == '__main__':
    unittest.main()
