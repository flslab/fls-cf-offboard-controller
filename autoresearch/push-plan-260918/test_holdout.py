#!/usr/bin/env python3
"""Independent cross-language transport acceptance; no drone/network I/O.

Loads production FC C, lets it serialize the snapshot, runs the real spawned
Pi worker, and feeds its returned fragments into production FC validation.
This proves protocol/execution boundaries, not closed-loop flight stability.
"""
import ctypes as C
import errno
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import zlib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from Interaction.post_release_pi_planner import (
    CHUNK, DEFAULT_START_DELAY_US, WIRE_PLAN_BODY as PLAN_BODY, PLAN_CHUNK, PLAN_COMMIT, PLAN_RESULT,
    RESULT, SNAPSHOT_ACK, WIRE_SNAPSHOT_BODY as SNAPSHOT_BODY, VERSION, PiEventPlanner,
    MODEL_REQUEST, MODEL_QUERY, MODEL_BODY,
    encode_chunks,
)

FW = Path('/Users/shuqinzhu/Documents/FLS_Research/crazyflie-firmware-master-post-release')
F3 = C.c_float * 3


class Snapshot(C.Structure):
    _fields_ = [('epoch', C.c_uint32), ('latest', C.c_uint32),
                ('state_age', C.c_uint16), ('attitude_age', C.c_uint16),
                ('velocity', C.c_float * 2), ('angle', F3), ('rate', F3),
                ('direction', C.c_float * 2), ('yaw', C.c_float),
                ('bias', F3), ('kp', F3), ('ki', F3), ('tau', F3), ('ref', F3),
                ('prefix_elapsed', C.c_float), ('prefix_duration', C.c_float)]


class State(C.Structure):
    _fields_ = [('session', C.c_uint32), ('sequence', C.c_uint16),
                ('token', C.c_uint16), ('valid', C.c_bool), ('ack', C.c_bool),
                ('committed', C.c_bool), ('invalid', C.c_bool),
                ('executed', C.c_bool), ('epoch', C.c_uint32),
                ('latest', C.c_uint32), ('start', C.c_uint32), ('crc', C.c_uint32),
                ('received_us', C.c_uint32), ('executed_us', C.c_uint32),
                ('snapshot', C.c_uint8 * 128), ('wire_snapshot', C.c_uint8 * 96),
                ('plan', C.c_uint8 * 36),
                ('mask', C.c_uint8)]


class Piece(C.Structure):
    _fields_ = [('coefficient', (C.c_float * 8) * 3), ('duration', C.c_float)]


class Plan(C.Structure):
    _fields_ = [('piece', Piece * 2), ('knot', F3), ('duration', C.c_float),
                ('end', F3)]


def payload_packet(data):
    packet = CRTPPacket()
    packet.set_header(CRTPPort.SETPOINT_HL, 1)
    packet.data = bytes(data)
    return packet


class Fc:
    def __init__(self, library):
        self.lib = C.CDLL(str(library))
        self.lib.postReleasePushCapture.restype = C.c_bool
        self.lib.postReleasePushFragment.restype = C.c_size_t
        self.lib.postReleasePushResult.restype = C.c_size_t
        self.lib.postReleasePushMarkExecuting.restype = C.c_bool
        self.lib.jointEvaluate.restype = C.c_bool
        self.lib.postReleaseUnwindPlan.restype = C.c_bool
        self.lib.postReleaseUnwindEvaluate.restype = C.c_bool
        self.state, self.plan = State(), Plan()
        self.callback = None
        self.accepts = []
        self.parts = []
        self.pending_results = []
        self.defer_results = False

    def begin(self, *, epoch=0xffff8000, window=300000, token=29,
              prefix_elapsed=0., prefix_duration=0., speed=1.25,
              actual_pitch=12., pitch_rate=20., reference_pitch=None):
        self.lib.postReleasePushReset(C.byref(self.state), C.c_uint32(0x73ca9012),
                                     C.c_uint16(65534), C.c_uint16(token))
        yaw = -47.0
        angle = math.radians(yaw)
        self.snapshot = Snapshot(epoch, (epoch + window) & 0xffffffff, 1500, 800,
            (C.c_float * 2)(speed * math.cos(angle), speed * math.sin(angle)),
            F3(0, actual_pitch, 0), F3(0, pitch_rate, 0),
            (C.c_float * 2)(math.cos(angle), math.sin(angle)), yaw,
            F3(0, .05, 0), F3(6, 7.1, 6), F3(1, 1, 1),
            F3(.05, .07, .08), F3(0, reference_pitch if reference_pitch is not None else
                math.degrees(math.atan(4 / 9.81)), 0), prefix_elapsed, prefix_duration)
        assert self.lib.postReleasePushCapture(C.byref(self.state), C.byref(self.snapshot))
        self.started = time.monotonic()

    def now(self):
        return (self.snapshot.epoch + round((time.monotonic() - self.started) * 1e6)) & 0xffffffff

    def chunks(self):
        result = []
        for i in range(6):
            out = (C.c_uint8 * 30)()
            count = self.lib.postReleasePushFragment(C.byref(self.state), i, out)
            assert 0 < count <= 30
            result.append(bytes(out[:count]))
        return result

    def add_port_callback(self, port, callback):
        assert port == CRTPPort.SETPOINT_HL
        self.callback = callback

    def remove_port_callback(self, port, callback):
        self.callback = None

    def receive(self, data, now=None, can_schedule=True):
        scheduled, complete = C.c_bool(), C.c_bool()
        code = self.lib.postReleasePushReceivePart(C.byref(self.state), data, len(data),
            C.c_uint32(self.now() if now is None else now), C.c_bool(can_schedule),
            C.byref(self.plan), C.byref(scheduled), C.byref(complete))
        return code, scheduled.value, complete.value

    def result_packet(self, request, code):
        out = (C.c_uint8 * RESULT.size)()
        count = self.lib.postReleasePushResult(request, len(request), code,
            C.c_uint32(self.state.start), C.c_uint32(self.state.received_us),
            C.c_uint32(self.state.executed_us), out)
        assert count == 24
        return bytes(out[:count])

    def execute(self, now=None):
        now = self.state.start if now is None else now
        assert self.lib.postReleasePushMarkExecuting(C.byref(self.state), C.c_uint32(now))
        self.callback(payload_packet(self.result_packet(self.chunks()[0], 0)))

    def send_packet(self, packet):
        assert packet.channel == 0
        data = bytes(packet.data)
        assert len(data) <= 30
        if data[0] == MODEL_REQUEST:
            _, _, nonce = MODEL_QUERY.unpack(data)
            model = F3(6, 7.1, 6), F3(1, 1, 1), F3(.05, .07, .08)
            values = (C.c_float * 9)(*[v for group in model for v in group])
            for i in range(3):
                out = (C.c_uint8 * 30)()
                count = self.lib.postReleasePushModelFragment(values, C.c_uint32(nonce), i, out)
                self.callback(payload_packet(bytes(out[:count])))
            return
        assert data[0] == PLAN_CHUNK, 'v3 must not send standalone ACK/COMMIT'
        self.parts.append(data)
        before = bytes(self.plan)
        code, scheduled, complete = self.receive(data)
        if complete:
            self.accepts.append((data, code, scheduled))
            if not scheduled and code == 0:
                assert bytes(self.plan) == before
            reply = self.result_packet(data, code)
            if self.defer_results:
                self.pending_results.append(reply)
            else:
                self.callback(payload_packet(reply))


def wait_phase(service, phases, timeout=1.5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        result = service.status()
        if result['phase'] in phases:
            return result
        time.sleep(.001)
    raise AssertionError(service.status())


def check_kernel_and_transport(library):
    fc = Fc(library)
    service = PiEventPlanner(fc)
    try:
        service.start()
        assert service.status()['ready']
        service.begin_release(0x73ca9012, 65534)
        fc.begin()
        chunks = fc.chunks()
        body = b''.join(packet[CHUNK.size:] for packet in chunks)
        assert len(body) == 96
        assert SNAPSHOT_BODY.unpack(body[:-4])[0] == 0xffff8000
        assert zlib.crc32(body[:-4]) == struct.unpack('<I', body[-4:])[0]
        # Reverse order plus an identical repeated middle fragment.
        for index in [4, 4, 5, 3, 2, 1, 0]:
            fc.callback(payload_packet(chunks[index]))
        result = wait_phase(service, ('accepted', 'plan_failed', 'plan_late', 'rejected', 'service_failed'))
        assert result['phase'] == 'accepted', result
        assert fc.state.committed and sum(item[2] for item in fc.accepts) == 1
        assert fc.state.start == (0xffff8000 + DEFAULT_START_DELAY_US) & 0xffffffff
        assert 0 < fc.plan.duration <= 1.6
        ref, rate, acc, jerk = F3(), F3(), F3(), F3()
        assert fc.lib.jointEvaluate(C.byref(fc.plan), C.c_float(0), ref, rate, acc, jerk)
        for i in range(3):
            assert abs(ref[i] - fc.snapshot.ref[i]) < 1e-4
            assert abs(rate[i]) < 1e-4
        saved_plan, last_part = bytes(fc.plan), fc.accepts[-1][0]
        code, scheduled, complete = fc.receive(last_part, (fc.state.start + 100000) & 0xffffffff, False)
        assert (code, scheduled, complete) == (0, False, True)
        assert bytes(fc.plan) == saved_plan
        # Wrong session after successful commit is not idempotent authorization.
        wrong = bytearray(last_part); wrong[4] ^= 1
        assert fc.receive(bytes(wrong), fc.now())[0] != 0
        fc.execute((fc.state.start + 123) & 0xffffffff)
        result = wait_phase(service, ('executing',))
        assert abs(result['fc_snapshot_to_first_curve_ms'] - (DEFAULT_START_DELAY_US / 1000 + .123)) < 1e-6
        assert result['fc_plan_to_first_curve_ms'] >= 5.
        return {'phase': result['phase'], 'wrapped_start_us': fc.state.start,
                'duration_s': fc.plan.duration, 'compute_s': result['compute_s'],
                'snapshot_to_plan_s': result['snapshot_to_plan_s'],
                'worker_pid': service._process.pid,
                'plan_body': bytes(fc.state.plan[:32]), 'parts': fc.parts,
                'fc_snapshot_to_first_curve_ms': result['fc_snapshot_to_first_curve_ms']}
    finally:
        service.close()


def check_adversarial_boundaries(library, successful):
    for lead, should_accept in ((5000, True), (4999, False), (0, False), (-1, False)):
        fc = Fc(library); fc.begin()
        start = (fc.snapshot.epoch + PLAN_BODY.unpack(successful['plan_body'])[0]) & 0xffffffff
        scheduled = False
        for part in reversed(successful['parts']):
            code, scheduled, complete = fc.receive(part, (start - lead) & 0xffffffff)
        assert (code == 0 and scheduled) == should_accept, (lead, code, scheduled)
    fc = Fc(library); fc.begin()
    for part in successful['parts'][:-1]:
        assert fc.receive(part)[0:3] == (0, False, False)
    assert not fc.state.committed
    fc = Fc(library); fc.begin()
    for part in successful['parts']:
        code, scheduled, complete = fc.receive(part, can_schedule=False)
    assert code == errno.EBUSY
    assert not fc.state.committed
    fc = Fc(library); fc.begin()
    first = successful['parts'][0]
    assert fc.receive(first)[0] == 0
    conflict = first[:-1] + bytes([first[-1] ^ 1])
    assert fc.receive(conflict)[0] != 0
    assert fc.state.invalid and not fc.state.committed
    assert fc.receive(first)[0] != 0
    # A valid-CRC NaN endpoint must not reach the scheduled plan.
    fc = Fc(library); fc.begin()
    values = list(PLAN_BODY.unpack(successful['plan_body'])); values[-1] = float('nan')
    bad = PLAN_BODY.pack(*values)
    for part in encode_chunks(PLAN_CHUNK, fc.state.session, fc.state.sequence, fc.state.token, bad):
        code, scheduled, complete = fc.receive(part)
    assert code != 0 and not fc.state.committed
    # V3 omits initial references entirely; validate finite endpoint limits.
    # C3 reference/rate continuity is checked on reconstructed plans below.
    for index in (5, 6):
        fc = Fc(library); fc.begin()
        values = list(PLAN_BODY.unpack(successful['plan_body'])); values[index] = 10.0
        bad = PLAN_BODY.pack(*values)
        for part in encode_chunks(PLAN_CHUNK, fc.state.session, fc.state.sequence, fc.state.token, bad):
            code, scheduled, complete = fc.receive(part)
        assert code != 0 and not fc.state.committed


def check_local_prefix_supersession(library):
    """An accepted rapid plan can be invalidated before its start. Its delayed
    result/parts must not defeat the newer, actively unwinding FC generation.
    The new C3 boundary is independently reconstructed by production FC C.
    """
    fc = Fc(library)
    service = PiEventPlanner(fc)
    try:
        service.start()
        service.begin_release(0x73ca9012, 65534)
        fc.defer_results = True
        fc.begin(token=65535)
        for chunk in fc.chunks():
            fc.callback(payload_packet(chunk))
        deadline = time.monotonic() + 1.5
        while not fc.pending_results and time.monotonic() < deadline:
            status = service.status()
            assert status['phase'] not in ('plan_failed', 'plan_late', 'service_failed'), status
            time.sleep(.001)
        assert fc.state.committed and fc.pending_results
        assert service.status()['phase'] == 'awaiting_result'
        old_result, old_parts = fc.pending_results[0], list(fc.parts)
        # Token wraps 65535->0; FC now follows a different immutable prefix.
        fc.begin(epoch=0xffff9000, token=0, prefix_elapsed=.04,
                 prefix_duration=.31, speed=1.0)
        fc.defer_results = False
        chunks = fc.chunks()
        fc.callback(payload_packet(chunks[5]))
        fc.callback(payload_packet(old_result))
        assert service.status()['superseded_snapshot_count'] == 1
        assert service.status().get('accepted_start_us') is None
        for old_part in old_parts:
            assert fc.receive(old_part)[0] != 0
        assert not fc.state.committed and fc.state.mask == 0
        for index in [3, 4, 4, 2, 1, 5, 0]:
            fc.callback(payload_packet(chunks[index]))
        result = wait_phase(service, ('accepted', 'plan_failed', 'plan_late', 'rejected', 'service_failed'))
        assert result['phase'] == 'accepted', result
        assert fc.state.committed and fc.state.token == 0
        assert result['prefix_duration_s'] > .30 and result['snapshot_token'] == 0
        # Compare reference, rate, acceleration AND jerk at the splice epoch.
        zero, prefix = F3(), Piece()
        assert fc.lib.postReleaseUnwindPlan(fc.snapshot.ref, zero, zero, zero,
            C.c_float(fc.snapshot.prefix_duration), C.byref(prefix))
        a, r, ac, j, pa, pr, pac, pj = [F3() for _ in range(8)]
        elapsed = fc.snapshot.prefix_elapsed + DEFAULT_START_DELAY_US / 1e6
        assert fc.lib.postReleaseUnwindEvaluate(C.byref(prefix), C.c_float(elapsed), pa, pr, pac, pj)
        assert fc.lib.jointEvaluate(C.byref(fc.plan), C.c_float(0), a, r, ac, j)
        for actual, expected, tolerance in ((a, pa, .002), (r, pr, .02), (ac, pac, .1), (j, pj, 1.)):
            assert max(abs(actual[i] - expected[i]) for i in range(3)) < tolerance
        assert abs(pac[1]) > 1 and abs(pj[1]) > 1
        # Late old result cannot overwrite newer accepted/executing timing.
        accepted_epoch = result['accepted_start_us']
        fc.callback(payload_packet(old_result))
        assert service.status()['accepted_start_us'] == accepted_epoch
        fc.execute((fc.state.start + 234) & 0xffffffff)
        result = wait_phase(service, ('executing',))
        assert abs(result['fc_snapshot_to_first_curve_ms'] - (DEFAULT_START_DELAY_US / 1000 + .234)) < 1e-6
        return {'new_token': result['snapshot_token'],
                'superseded_count': result['superseded_snapshot_count'],
                'prefix_duration_s': result['prefix_duration_s'],
                'curve_duration_s': result['duration_s'],
                'first_curve_ms': result['fc_snapshot_to_first_curve_ms']}
    finally:
        service.close()


def main():
    assert C.sizeof(Snapshot) == 124
    with tempfile.TemporaryDirectory(prefix='fls-push-plan-holdout-') as directory:
        library = Path(directory) / ('firmware.dylib' if sys.platform == 'darwin' else 'firmware.so')
        source = FW / 'src/modules/src/kalman_core'
        subprocess.run(['cc', '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                        '-dynamiclib' if sys.platform == 'darwin' else '-shared', '-fPIC',
                        '-I' + str(FW / 'src/modules/interface/kalman_core'),
                        *(str(source / name) for name in ('post_release_push_plan.c',
                          'post_release_joint_unwind.c', 'post_release_forward_stop.c')),
                        '-lm', '-o', str(library)], check=True)
        result = check_kernel_and_transport(library)
        check_adversarial_boundaries(library, result)
        result['local_prefix_supersession'] = check_local_prefix_supersession(library)
        for key in ('plan_body', 'parts'):
            del result[key]
        print(json.dumps({'passed': True, 'scope': 'production C / spawned Pi protocol only',
                          **result}, sort_keys=True))
        print('PASS')


if __name__ == '__main__':
    main()
