import json
import hashlib
import math
import multiprocessing
from multiprocessing.connection import wait as connection_wait
import os
from pathlib import Path
import struct
import socket
import threading
import time
import unittest
from unittest.mock import patch

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from Interaction.post_release_pi_planner import (
    ACK, CHUNK, COMMIT, DATA_BYTES, PLAN_BODY, PLAN_CHUNK, PLAN_COMMIT,
    PLAN_RESULT, RESULT, SNAPSHOT, SNAPSHOT_ACK, SNAPSHOT_BODY, VERSION,
    FragmentAssembler, NativePlanner, PiEventPlanner, decode_snapshot,
    encode_chunks as raw_encode_chunks, forward_delta_us,
    WIRE_SNAPSHOT_BODY, WIRE_PLAN_BODY, MODEL_QUERY, MODEL_BODY, MODEL_REQUEST,
    MODEL_PART, compact_snapshot, expand_snapshot, compact_plan, validate_model,
)


TEST_MODEL = MODEL_BODY.pack(6., 7.1, 6., 1., 1., 1., .05, .07, .08)

def encode_chunks(kind, session, sequence, token, body):
    if kind == SNAPSHOT:
        body = compact_snapshot(body)
    return raw_encode_chunks(kind, session, sequence, token, body)


def snapshot(*, epoch=1_000_000, window=400_000, speed=1., yaw=0.):
    values = [0.] * 28
    values[0:2] = [speed * math.cos(math.radians(yaw)),
                   speed * math.sin(math.radians(yaw))]
    values[3], values[6] = 10., 30.  # actual pitch and rate, not reference
    values[8:10] = [math.cos(math.radians(yaw)), math.sin(math.radians(yaw))]
    values[10] = yaw
    values[14:17] = [6., 7.1, 6.]
    values[17:20] = [1., 1., 1.]
    values[20:23] = [.05, .07, .08]
    values[24] = 20.  # already issued constant rapid pitch reference
    return SNAPSHOT_BODY.pack(epoch, (epoch + window) & 0xffffffff, 2000, 1000, *values)


def packet(data, channel=1):
    result = CRTPPacket()
    result.set_header(CRTPPort.SETPOINT_HL, channel)
    result.data = data
    return result


class ProtocolTests(unittest.TestCase):
    def test_exact_wire_sizes_and_roundtrip(self):
        self.assertEqual((WIRE_SNAPSHOT_BODY.size, WIRE_PLAN_BODY.size), (92, 32))
        self.assertEqual(RESULT.size, 24)
        data = snapshot()
        chunks = encode_chunks(SNAPSHOT, 0x12345678, 9, 17, data)
        self.assertEqual(len(chunks), 6)
        self.assertTrue(all(len(item) <= 30 for item in chunks))
        assembly = FragmentAssembler(SNAPSHOT, 0x12345678, 9, WIRE_SNAPSHOT_BODY.size)
        found = None
        for chunk in reversed(chunks):
            result = assembly.accept(chunk)
            if result is not None:
                found = result
        self.assertEqual(expand_snapshot(found, TEST_MODEL), data)
        self.assertEqual(expand_snapshot(assembly.accept(chunks[0]), TEST_MODEL), data)

    def test_missing_part_never_completes(self):
        assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        chunks = encode_chunks(SNAPSHOT, 99, 7, 4, snapshot())
        for chunk in chunks[:-1]:
            self.assertIsNone(assembly.accept(chunk))
        self.assertIsNone(assembly.accept(chunks[0]))

    def test_cross_session_sequence_and_version_ignored(self):
        assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        for session, seq in ((98, 7), (99, 8)):
            for chunk in encode_chunks(SNAPSHOT, session, seq, 4, snapshot()):
                self.assertIsNone(assembly.accept(chunk))
        chunk = bytearray(encode_chunks(SNAPSHOT, 99, 7, 4, snapshot())[0])
        chunk[1] = 1
        self.assertIsNone(assembly.accept(chunk))
        self.assertEqual(assembly.parts, {})

    def test_conflicting_duplicate_latches_rejection(self):
        assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        chunks = encode_chunks(SNAPSHOT, 99, 7, 4, snapshot())
        assembly.accept(chunks[0])
        broken = bytearray(chunks[0]); broken[-1] ^= 1
        with self.assertRaisesRegex(ValueError, 'conflicting'):
            assembly.accept(broken)
        self.assertTrue(assembly.rejected)
        for chunk in chunks:
            self.assertIsNone(assembly.accept(chunk))

    def test_token_change_is_not_cross_assembled(self):
        assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        first = encode_chunks(SNAPSHOT, 99, 7, 4, snapshot())
        second = encode_chunks(SNAPSHOT, 99, 7, 5, snapshot())
        assembly.accept(first[0])
        with self.assertRaisesRegex(ValueError, 'token'):
            assembly.accept(second[1])

    def test_crc_and_exact_final_length(self):
        chunks = encode_chunks(SNAPSHOT, 99, 7, 4, snapshot())
        assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        with self.assertRaisesRegex(ValueError, 'length'):
            assembly.accept(chunks[-1] + b'\0')
        corrupted = list(chunks)
        corrupted[-1] = corrupted[-1][:-1] + bytes([corrupted[-1][-1] ^ 1])
        with self.assertRaisesRegex(ValueError, 'CRC'):
            for chunk in corrupted:
                assembly.accept(chunk)

    def test_bad_part_index_count(self):
        for index, count in ((8, 8), (0, 9), (0, 0)):
            assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
            bad = CHUNK.pack(SNAPSHOT, VERSION, 7, 99, 4, index, count) + b'\0' * DATA_BYTES
            with self.assertRaisesRegex(ValueError, 'count/index'):
                assembly.accept(bad)

    def test_clock_wrap_and_no_cross_clock_math(self):
        self.assertEqual(forward_delta_us(20, 0xfffffff0), 36)
        epoch, latest, state_age, attitude_age, _ = decode_snapshot(snapshot(epoch=0xffff0000))
        self.assertEqual(forward_delta_us(latest, epoch), 400_000)
        self.assertEqual((state_age, attitude_age), (2000, 1000))

    def test_nonfinite_and_dead_snapshot_window_rejected(self):
        values = list(SNAPSHOT_BODY.unpack(snapshot()))
        values[4] = float('nan')
        with self.assertRaisesRegex(ValueError, 'nonfinite'):
            decode_snapshot(SNAPSHOT_BODY.pack(*values))
        for window in (0, 19999, 1000001, 0xffffffff):
            with self.assertRaisesRegex(ValueError, 'future start window'):
                decode_snapshot(snapshot(window=window))

    def test_stale_age_fields_rejected(self):
        for field, age in ((2, 40001), (3, 20001)):
            values = list(SNAPSHOT_BODY.unpack(snapshot()))
            values[field] = age
            with self.assertRaisesRegex(ValueError, 'age exceeds'):
                decode_snapshot(SNAPSHOT_BODY.pack(*values))


class NativePlannerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.planner = NativePlanner()

    @classmethod
    def tearDownClass(cls):
        cls.planner.close()

    def test_real_kernel_uses_reference_not_predicted_actual_angles(self):
        result = self.planner.solve(snapshot())
        delay, duration, *parameters = PLAN_BODY.unpack(result['body'])
        self.assertEqual(delay, 40_000)
        self.assertEqual(parameters[:6], [0., 20., 0., 0., 0., 0.])
        self.assertLess(result['predicted_start_speed_mps'], 1.)
        self.assertLessEqual(result['predicted_terminal_speed_mps'], .012)
        self.assertGreaterEqual(result['predicted_min_forward_mps'], -.025)
        self.assertGreater(duration, .13)
        self.assertEqual(result['start_us'], 1_040_000)

    def test_world_yaw_rotation_preserves_relative_plan(self):
        a = PLAN_BODY.unpack(self.planner.solve(snapshot())['body'])
        b = PLAN_BODY.unpack(self.planner.solve(snapshot(yaw=90.))['body'])
        for first, second in zip(a, b):
            self.assertAlmostEqual(first, second, places=3)

    def test_too_late_or_infeasible_snapshot_not_faked(self):
        with self.assertRaisesRegex(ValueError, 'feasible'):
            self.planner.solve(snapshot(speed=.2))
        with self.assertRaisesRegex(ValueError, 'future start'):
            self.planner.solve(snapshot(window=5_000))

    def test_start_deadline_cap_and_modular_epoch(self):
        result = self.planner.solve(snapshot(epoch=0xffff0000, window=30_000))
        self.assertEqual(result['start_delay_us'], 30_000)
        self.assertEqual(result['start_us'], (0xffff0000 + 30_000) & 0xffffffff)

    def test_unwind_prefix_uses_nonzero_reference_derivatives(self):
        values = list(SNAPSHOT_BODY.unpack(snapshot(speed=1.03)))
        values[-2:] = [.02, .32]
        result = self.planner.solve(SNAPSHOT_BODY.pack(*values))
        _, _, *parameters = PLAN_BODY.unpack(result['body'])
        self.assertLess(parameters[1], 20.)
        self.assertLess(parameters[4], 0.)
        self.assertLessEqual(result['predicted_terminal_speed_mps'], .012)
        self.assertGreaterEqual(result['predicted_min_forward_mps'], -.025)

    def test_vendored_kernel_pinned_hashes_and_paired_source_parity(self):
        bundled = Path(__file__).resolve().parents[1] / 'native' / 'post_release'
        paired = Path(os.environ.get('FLS_JOINT_FW', str(
            Path.home() / 'Documents/FLS_Research/crazyflie-firmware-master-post-release')))
        hashes = {
            'post_release_joint_unwind.c': 'a58fe2864fcf7a0a98e78f2f3140dbb5f4d22709cb7ecaa2869a70725230f0b0',
            'post_release_joint_unwind.h': '596b25187d7fa5a8690a1d9338fbbab993c6433a090806e8c5d7c1ed051be9ea',
            'post_release_forward_stop.c': '2a2cfec556ec32d0eacab4f7f6b9e45fe21b439fe5002481ab04ccd4efa0303c',
            'post_release_forward_stop.h': '6171bf8050d74d1f2f4315d09e2707287b276a1b87caf0a068e2fc95704460f9',
        }
        for name, expected in hashes.items():
            body = (bundled / name).read_bytes()
            self.assertEqual(hashlib.sha256(body).hexdigest(), expected)
            source = paired / 'src/modules' / ('src' if name.endswith('.c') else 'interface') / 'kalman_core' / name
            if source.exists():
                self.assertEqual(body, source.read_bytes(), 'paired source drift: ' + name)


class FakeCf:
    def __init__(self):
        self.callback = None
        self.sent = []
        self.send_threads = []
        self.reply = True
        self.check_lock = None
        self.assembly = FragmentAssembler(PLAN_CHUNK, 99, 7, WIRE_PLAN_BODY.size)
        self.plan = None

    def add_port_callback(self, port, callback):
        self.callback = callback

    def remove_port_callback(self, port, callback):
        self.callback = None

    def send_packet(self, value):
        if self.check_lock is not None:
            completed = threading.Event()
            def reader():
                self.check_lock()
                completed.set()
            reader_thread = threading.Thread(target=reader)
            reader_thread.start()
            if not completed.wait(.1):
                raise RuntimeError('transport I/O held service state lock')
            reader_thread.join()
        data = bytes(value.data)
        self.sent.append(data)
        self.send_threads.append(threading.current_thread().name)
        if data[0] == MODEL_REQUEST:
            _, _, nonce = MODEL_QUERY.unpack(data)
            for part in raw_encode_chunks(MODEL_PART, nonce, 0, 0, TEST_MODEL):
                self.callback(packet(part))
        if data[0] == PLAN_CHUNK:
            body = self.assembly.accept(data)
            if body is not None:
                self.plan = body
                if self.reply:
                    delay = WIRE_PLAN_BODY.unpack(self.plan)[0]
                    self.callback(packet(RESULT.pack(PLAN_RESULT, VERSION, 7, 99, 4,
                                                     0, 1_000_000 + delay, 1_005_000, 0)))


class ServiceTests(unittest.TestCase):
    def test_superseding_prefix_cancels_old_worker_and_old_packets(self):
        service = PiEventPlanner(FakeCf())
        service._model_body = TEST_MODEL
        service._assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        service._status.update(ready=True, release_begin_s=10.,
                               snapshot_fragment_count=0, snapshot_duplicate_count=0)
        old = encode_chunks(SNAPSHOT, 99, 7, 65535, snapshot())
        new = encode_chunks(SNAPSHOT, 99, 7, 0, snapshot())
        for chunk in old:
            service._on_packet(packet(chunk))
        generation = service._generation
        for chunk in new:
            service._on_packet(packet(chunk))
        self.assertEqual(service._generation, generation + 1)
        self.assertEqual(service._assembly.token, 0)
        service._handle_plan(generation, None, 'late old failure')
        for chunk in old:
            service._on_packet(packet(chunk))
        self.assertIsNone(service.status()['error'])
        self.assertEqual(service.status()['superseded_snapshot_count'], 1)
        self.assertEqual(service._assembly.token, 0)

    def test_execution_notice_is_async_same_fc_clock_and_immutable(self):
        service = PiEventPlanner(FakeCf())
        service._model_body = TEST_MODEL
        service._sent_plan = {'start_us': 100, 'snapshot_us': 0xfffffff0}
        accepted = (PLAN_RESULT, VERSION, 7, 99, 4, 0, 100, 40, 0)
        executed = (PLAN_RESULT, VERSION, 7, 99, 4, 0, 100, 40, 120)
        service._handle_event('result', 0, accepted)
        self.assertEqual(service.status()['phase'], 'accepted')
        self.assertNotIn('fc_snapshot_to_first_curve_ms', service.status())
        service._handle_event('result', 0, executed)
        self.assertEqual(service.status()['phase'], 'executing')
        self.assertAlmostEqual(service.status()['fc_snapshot_to_first_curve_ms'], .136)
        self.assertAlmostEqual(service.status()['fc_plan_to_first_curve_ms'], .080)
        service._handle_event('result', 0, accepted)
        service._handle_event('result', 0, executed[:-1] + (125,))
        self.assertEqual(service.status()['fc_first_curve_us'], 120)
        self.assertEqual(service.cf.sent, [])

    def test_execution_notice_can_arrive_before_acceptance(self):
        service = PiEventPlanner(FakeCf())
        service._model_body = TEST_MODEL
        service._sent_plan = {'start_us': 100, 'snapshot_us': 0xfffffff0}
        service._handle_event('result', 0, (PLAN_RESULT, VERSION, 7, 99, 4, 0, 100, 40, 120))
        service._handle_event('result', 0, (PLAN_RESULT, VERSION, 7, 99, 4, 0, 100, 40, 0))
        self.assertEqual(service.status()['phase'], 'executing')

    def test_refuses_before_prewarm(self):
        service = PiEventPlanner(FakeCf())
        service._model_body = TEST_MODEL
        self.assertFalse(service.status()['ready'])
        with self.assertRaisesRegex(RuntimeError, 'prewarmed'):
            service.begin_release(99, 7)
        service.close()

    def test_spawn_process_end_to_end_and_idempotent_snapshot(self):
        cf = FakeCf()
        service = PiEventPlanner(cf)
        service._model_body = TEST_MODEL
        try:
            service.start()
            cf.check_lock = service.status
            self.assertTrue(service.status()['ready'])
            self.assertNotEqual(service._process.pid, __import__('os').getpid())
            service.begin_release(99, 7)
            chunks = encode_chunks(SNAPSHOT, 99, 7, 4, snapshot())
            for chunk in reversed(chunks):
                cf.callback(packet(chunk))
            deadline = time.monotonic() + 2.
            while service.status()['phase'] not in ('accepted', 'plan_failed', 'plan_late') and time.monotonic() < deadline:
                time.sleep(.001)
            state = service.status()
            self.assertEqual(state['phase'], 'accepted', state)
            self.assertEqual(state['accepted_start_us'], 1_040_000)
            self.assertEqual(state['plan_send_count'], 1)
            for chunk in chunks:
                cf.callback(packet(chunk))
            time.sleep(.02)
            self.assertEqual(service.status()['plan_send_count'], 1)
            self.assertTrue(all(name == 'post-release-plan-link' for name, data in zip(cf.send_threads, cf.sent) if data[0] != MODEL_REQUEST))
            json.dumps(service.status())
            self.assertFalse(any(data[0] in (SNAPSHOT_ACK, PLAN_COMMIT) for data in cf.sent))
            self.assertEqual(len(cf.sent), 3)
            evidence = service.drain_diagnostics()
            snap = next(row for row in evidence if row['event'] == 'snapshot')
            self.assertEqual(bytes.fromhex(snap['expanded_body_hex']), snapshot())
            self.assertEqual(bytes.fromhex(snap['model_body_hex']), TEST_MODEL)
            plan = next(row for row in evidence if row['event'] == 'planner_result')
            self.assertEqual(compact_plan(bytes.fromhex(plan['expanded_plan_hex'])),
                             bytes.fromhex(plan['wire_plan_hex']))
            self.assertTrue(any(row['event'] == 'firmware_result' for row in evidence))
            json.dumps(evidence)
        finally:
            service.close()
        self.assertFalse(service.status()['ready'])
        self.assertIsNone(cf.callback)

    def test_spawn_pipeline_wakes_without_maintenance_poll(self):
        # Deliberately make the maintenance timer longer than this test's
        # deadline: snapshot, worker result and FC result must all wake it.
        cf = FakeCf()
        service = PiEventPlanner(cf)
        service._model_body = TEST_MODEL
        def long_wait(objects, timeout):
            return connection_wait(objects, timeout=5.)
        with patch('Interaction.post_release_pi_planner.wait', side_effect=long_wait):
            try:
                service.start()
                service.begin_release(99, 7)
                for chunk in encode_chunks(SNAPSHOT, 99, 7, 4, snapshot()):
                    cf.callback(packet(chunk))
                deadline = time.monotonic() + 2.
                while service.status()['phase'] != 'accepted' and time.monotonic() < deadline:
                    time.sleep(.001)
                self.assertEqual(service.status()['phase'], 'accepted', service.status())
                self.assertEqual(service.status()['plan_send_count'], 1)
            finally:
                service.close()
        self.assertFalse(service._thread.is_alive())
        self.assertFalse(service._process.is_alive())
        self.assertIsNone(service._wake_read)
        self.assertIsNone(service._wake_write)

    def test_late_plan_is_never_shifted_or_sent(self):
        service = PiEventPlanner(FakeCf())
        service._model_body = TEST_MODEL
        service._assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        service._assembly.first_receive_s = 10.
        with patch('Interaction.post_release_pi_planner.time.monotonic', return_value=10.071):
            service._handle_plan(0, {'start_delay_us': 80_000}, None)
        self.assertEqual(service.status()['phase'], 'plan_late')
        self.assertEqual(service.cf.sent, [])

    def test_old_worker_result_cannot_cross_release(self):
        service = PiEventPlanner(FakeCf())
        service._model_body = TEST_MODEL
        service._generation = 2
        service._handle_plan(1, None, 'old failure')
        self.assertIsNone(service.status()['error'])
        self.assertEqual(service.cf.sent, [])

    def test_ack_not_sent_from_callback(self):
        cf = FakeCf()
        service = PiEventPlanner(cf)
        service._model_body = TEST_MODEL
        service._assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        service._status.update(snapshot_fragment_count=0, snapshot_duplicate_count=0)
        for chunk in encode_chunks(SNAPSHOT, 99, 7, 4, snapshot()):
            service._on_packet(packet(chunk))
        self.assertEqual(cf.sent, [])
        self.assertEqual(service._events.qsize(), 1)

    def test_result_timeout_does_not_resend_at_or_after_start(self):
        cf = FakeCf()
        service = PiEventPlanner(cf)
        service._model_body = TEST_MODEL
        service._assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        service._assembly.token = 4
        service._assembly.first_receive_s = 10.
        service._sent_plan = {'start_delay_us': 80_000, 'start_us': 1_080_000}
        service._status.update(phase='awaiting_result', last_plan_send_s=10., plan_send_count=1)
        with patch('Interaction.post_release_pi_planner.time.monotonic', return_value=10.071):
            service._tick()
        self.assertEqual(service.status()['phase'], 'result_timeout')
        self.assertEqual(cf.sent, [])

    def test_bounded_retry_and_matching_rejection(self):
        cf = FakeCf(); cf.reply = False
        service = PiEventPlanner(cf)
        service._model_body = TEST_MODEL
        service._assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        service._assembly.token = 4
        service._assembly.first_receive_s = 10.
        service._sent_plan = {'body': PLAN_BODY.pack(80_000, .8, *([0.] * 12)),
                              'start_delay_us': 80_000, 'start_us': 1_080_000}
        service._status.update(phase='awaiting_result', last_plan_send_s=10., plan_send_count=1)
        with patch('Interaction.post_release_pi_planner.time.monotonic', return_value=10.021):
            service._tick()
        with patch('Interaction.post_release_pi_planner.time.monotonic', return_value=10.042):
            service._tick()
        with patch('Interaction.post_release_pi_planner.time.monotonic', return_value=10.065):
            service._tick()
        service._flush_io()
        self.assertEqual(service.status()['plan_send_count'], 3)
        self.assertEqual(sum(data[0] == PLAN_CHUNK for data in cf.sent), 4)
        service._handle_event('result', 0, (PLAN_RESULT, VERSION, 7, 99, 4, 110, 0, 0, 0))
        self.assertEqual(service.status()['phase'], 'rejected')
        self.assertEqual(service.status()['firmware_errno'], 110)
        service._handle_event('result', 0, (PLAN_RESULT, VERSION, 7, 99, 4, 0, 1_080_000, 1_005_000, 0))
        self.assertEqual(service.status()['phase'], 'rejected')

    def test_conflicting_snapshot_cannot_launch_pending_plan(self):
        service = PiEventPlanner(FakeCf())
        service._model_body = TEST_MODEL
        service._assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        service._assembly.rejected = True
        service._handle_event('snapshot', 0, snapshot())
        service._handle_plan(0, {}, None)
        self.assertEqual(service.status()['phase'], 'snapshot_rejected')
        self.assertEqual(service.cf.sent, [])

    def test_wrong_success_epoch_is_rejected_not_retimed(self):
        service = PiEventPlanner(FakeCf())
        service._model_body = TEST_MODEL
        service._sent_plan = {'start_us': 1_080_000}
        service._handle_event('result', 0, (PLAN_RESULT, VERSION, 7, 99, 4, 0, 1_090_000, 1_005_000, 0))
        self.assertEqual(service.status()['phase'], 'result_rejected')

    def test_incomplete_commit_retries_identical_plan_then_accepts(self):
        cf = FakeCf(); cf.reply = False
        service = PiEventPlanner(cf)
        service._model_body = TEST_MODEL
        service._assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        service._assembly.token = 4
        service._assembly.first_receive_s = 10.
        body = PLAN_BODY.pack(80_000, .8, *([0.] * 12))
        service._sent_plan = {'body': body, 'start_delay_us': 80_000, 'start_us': 1_080_000}
        service._status.update(phase='awaiting_result', last_plan_send_s=10., plan_send_count=1)
        with patch('Interaction.post_release_pi_planner.time.monotonic', return_value=10.01):
            service._handle_event('result', 0, (PLAN_RESULT, VERSION, 7, 99, 4, 11, 0, 0, 0))
            service._tick()
            service._flush_io()
        self.assertFalse(service._result_seen)
        self.assertEqual(cf.plan, compact_plan(body))
        self.assertEqual(service.status()['plan_send_count'], 2)
        service._handle_event('result', 0, (PLAN_RESULT, VERSION, 7, 99, 4, 0, 1_080_000, 1_005_000, 0))
        self.assertEqual(service.status()['phase'], 'accepted')
        self.assertEqual(service.status()['accepted_start_us'], 1_080_000)

    def test_snapshot_timeout_does_not_claim_hold(self):
        service = PiEventPlanner(FakeCf())
        service._model_body = TEST_MODEL
        service._status.update(phase='waiting_snapshot', release_begin_s=10.)
        with patch('Interaction.post_release_pi_planner.time.monotonic', return_value=11.01):
            service._tick()
        self.assertEqual(service.status()['phase'], 'snapshot_timeout')
        self.assertNotIn('hold', service.status())


class ModelCacheTests(unittest.TestCase):
    def test_dynamic_integral_bias_and_direction_preserved_bit_exact(self):
        values = list(SNAPSHOT_BODY.unpack(snapshot(yaw=47.)))
        values[15:18] = [.123, -.456, .789]
        body = SNAPSHOT_BODY.pack(*values)
        self.assertEqual(expand_snapshot(compact_snapshot(body), TEST_MODEL), body)

    def test_missing_or_changed_cache_never_reaches_worker(self):
        wire = compact_snapshot(snapshot())
        other = list(MODEL_BODY.unpack(TEST_MODEL)); other[0] += .1
        for cache in (None, MODEL_BODY.pack(*other)):
            with self.assertRaisesRegex(ValueError, 'model changed'):
                expand_snapshot(wire, cache)
            service = PiEventPlanner(FakeCf())
            service._model_body = cache
            service._assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
            service._status.update(snapshot_fragment_count=0, snapshot_duplicate_count=0)
            for chunk in encode_chunks(SNAPSHOT, 99, 7, 4, snapshot()):
                service._on_packet(packet(chunk))
            self.assertTrue(service._events.empty())
            self.assertEqual(service.status()['phase'], 'snapshot_rejected')
            self.assertEqual(service.cf.sent, [])

    def test_tau_float32_boundaries_and_invalid_model(self):
        values = list(MODEL_BODY.unpack(TEST_MODEL))
        for tau in (.01, .20):
            values[6] = tau
            validate_model(MODEL_BODY.pack(*values))
        for tau in (.009, .201, float('nan')):
            values[6] = tau
            with self.assertRaises(ValueError):
                validate_model(MODEL_BODY.pack(*values))

    def test_startup_timeout_is_bounded_and_not_ready(self):
        cf = FakeCf()
        cf.send_packet = lambda packet: None
        service = PiEventPlanner(cf)
        with self.assertRaisesRegex(RuntimeError, 'timeout'):
            service.sync_model(timeout_s=.01)
        self.assertFalse(service.status()['ready'])
        self.assertIsNone(service._model_body)
        self.assertIsNone(service._model_assembly)

    def test_stale_nonce_response_is_ignored(self):
        service = PiEventPlanner(FakeCf())
        service._model_assembly = FragmentAssembler(MODEL_PART, 123, 0, MODEL_BODY.size)
        for part in raw_encode_chunks(MODEL_PART, 122, 0, 0, TEST_MODEL):
            service._on_packet(packet(part))
        self.assertIsNone(service._model_body)
        self.assertFalse(service._model_event.is_set())
        for part in reversed(raw_encode_chunks(MODEL_PART, 123, 0, 0, TEST_MODEL)):
            service._on_packet(packet(part))
        self.assertEqual(service._model_body, TEST_MODEL)
        self.assertTrue(service._model_event.is_set())

    def test_corrupt_model_crc_never_ready(self):
        service = PiEventPlanner(FakeCf())
        service._model_assembly = FragmentAssembler(MODEL_PART, 123, 0, MODEL_BODY.size)
        parts = raw_encode_chunks(MODEL_PART, 123, 0, 0, TEST_MODEL)
        parts[-1] = parts[-1][:-1] + bytes([parts[-1][-1] ^ 1])
        for part in parts:
            service._on_packet(packet(part))
        self.assertIsNone(service._model_body)
        self.assertIn('CRC', service._model_error)

    def test_no_model_sync_during_release(self):
        service = PiEventPlanner(FakeCf())
        service._assembly = FragmentAssembler(SNAPSHOT, 99, 7, WIRE_SNAPSHOT_BODY.size)
        with self.assertRaisesRegex(RuntimeError, 'before release'):
            service.sync_model()
        self.assertEqual(service.cf.sent, [])


class WakeupTests(unittest.TestCase):
    def setUp(self):
        self.service = PiEventPlanner(FakeCf())
        self.service._model_body = TEST_MODEL
        self.service._connection, self.peer = multiprocessing.Pipe()
        self.service._wake_read, self.service._wake_write = socket.socketpair()
        self.service._wake_read.setblocking(False)
        self.service._wake_write.setblocking(False)

    def tearDown(self):
        self.service.close()
        self.peer.close()

    def assert_wakes(self, action):
        entered, finished = threading.Event(), threading.Event()
        failures = []
        def observed_wait(objects, timeout):
            entered.set()
            return connection_wait(objects, timeout)
        def waiter():
            try:
                self.service._wait_for_work(timeout_s=5.)
            except Exception as exc:
                failures.append(exc)
            finally:
                finished.set()
        with patch('Interaction.post_release_pi_planner.wait', side_effect=observed_wait):
            thread = threading.Thread(target=waiter)
            thread.start()
            try:
                self.assertTrue(entered.wait(1.))
                action()
                self.assertTrue(finished.wait(1.), 'wait only ended on maintenance timeout')
                self.assertEqual(failures, [])
            finally:
                self.service._stop.set()
                self.service._notify_work()
                thread.join(1.)

    def test_worker_pipe_wakes_without_fc_packet(self):
        self.assert_wakes(lambda: self.peer.send(('result', (0, None, 'test'))))
        self.assertEqual(self.service._connection.recv(), ('result', (0, None, 'test')))

    def test_callback_queue_wakes_idle_service(self):
        event = ('result', 0, None)
        self.assert_wakes(lambda: self.service._queue_event(event))
        self.assertEqual(self.service._events.get_nowait(), event)

    def test_stop_wakes_idle_service(self):
        def stop():
            self.service._stop.set()
            self.service._notify_work()
        self.assert_wakes(stop)

    def test_pending_queue_never_waits_for_another_wakeup(self):
        # A previous socket read may coalesce several queued events.
        self.service._events.put_nowait(('result', 0, None))
        with patch('Interaction.post_release_pi_planner.wait') as wait_mock:
            self.service._wait_for_work()
        wait_mock.assert_not_called()

    def test_maintenance_period_unchanged_and_both_sources_registered(self):
        with patch('Interaction.post_release_pi_planner.wait', return_value=[]) as wait_mock:
            self.service._wait_for_work()
        wait_mock.assert_called_once_with(
            [self.service._connection, self.service._wake_read], timeout=.002)


if __name__ == '__main__':
    unittest.main()
