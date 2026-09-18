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
    CHUNK, COMMIT, PLAN_BODY, PLAN_CHUNK, PLAN_COMMIT, PLAN_RESULT,
    RESULT, SNAPSHOT_ACK, SNAPSHOT_BODY, VERSION, PiEventPlanner,
    encode_chunks,
)

FW = Path('/Users/shuqinzhu/Documents/FLS_Research/crazyflie-firmware-master-post-release')
F3 = C.c_float * 3


class Snapshot(C.Structure):
    _fields_ = [('epoch', C.c_uint32), ('latest', C.c_uint32),
                ('state_age', C.c_uint16), ('attitude_age', C.c_uint16),
                ('velocity', C.c_float * 2), ('angle', F3), ('rate', F3),
                ('direction', C.c_float * 2), ('yaw', C.c_float),
                ('bias', F3), ('kp', F3), ('ki', F3), ('tau', F3), ('ref', F3)]


class State(C.Structure):
    _fields_ = [('session', C.c_uint32), ('sequence', C.c_uint16),
                ('token', C.c_uint16), ('valid', C.c_bool), ('ack', C.c_bool),
                ('committed', C.c_bool), ('epoch', C.c_uint32),
                ('latest', C.c_uint32), ('start', C.c_uint32), ('crc', C.c_uint32),
                ('snapshot', C.c_uint8 * 120), ('plan', C.c_uint8 * 60),
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
        self.state, self.plan = State(), Plan()
        self.callback = None
        self.commits = []
        self.parts = []

    def begin(self, *, epoch=0xffff0000, window=300000, token=29):
        self.lib.postReleasePushReset(C.byref(self.state), C.c_uint32(0x73ca9012),
                                     C.c_uint16(65534), C.c_uint16(token))
        yaw = -47.0
        angle = math.radians(yaw)
        self.snapshot = Snapshot(epoch, (epoch + window) & 0xffffffff, 1500, 800,
            (C.c_float * 2)(1.25 * math.cos(angle), 1.25 * math.sin(angle)),
            F3(0, 12, 0), F3(0, 20, 0),
            (C.c_float * 2)(math.cos(angle), math.sin(angle)), yaw,
            F3(0, .05, 0), F3(6, 7.1, 6), F3(1, 1, 1),
            F3(.05, .07, .08), F3(0, math.degrees(math.atan(4 / 9.81)), 0))
        assert self.lib.postReleasePushCapture(C.byref(self.state), C.byref(self.snapshot))
        self.started = time.monotonic()

    def now(self):
        return (self.snapshot.epoch + round((time.monotonic() - self.started) * 1e6)) & 0xffffffff

    def chunks(self):
        result = []
        for i in range(7):
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

    def accept_part(self, data):
        return self.lib.postReleasePushAcceptPart(C.byref(self.state), data, len(data))

    def commit(self, data, now, can_schedule=True):
        scheduled = C.c_bool()
        code = self.lib.postReleasePushCommit(C.byref(self.state), data, len(data),
            C.c_uint32(now), C.c_bool(can_schedule), C.byref(self.plan), C.byref(scheduled))
        return code, scheduled.value

    def send_packet(self, packet):
        assert packet.channel == 0
        data = bytes(packet.data)
        assert len(data) <= 30
        if data[0] == SNAPSHOT_ACK:
            assert self.lib.postReleasePushAcknowledge(C.byref(self.state), data, len(data)) == 0
        elif data[0] == PLAN_CHUNK:
            self.parts.append(data)
            assert self.accept_part(data) == 0
        elif data[0] == PLAN_COMMIT:
            before = bytes(self.plan)
            code, scheduled = self.commit(data, self.now())
            self.commits.append((data, code, scheduled))
            if not scheduled and code == 0:
                assert bytes(self.plan) == before
            out = (C.c_uint8 * 16)()
            count = self.lib.postReleasePushResult(data, len(data), code,
                C.c_uint32(self.state.start), out)
            self.callback(payload_packet(out[:count]))


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
        assert len(body) == 120
        assert SNAPSHOT_BODY.unpack(body[:-4])[0] == 0xffff0000
        assert zlib.crc32(body[:-4]) == struct.unpack('<I', body[-4:])[0]
        # Reverse order plus an identical repeated middle fragment.
        for index in [6, 4, 4, 5, 3, 2, 1, 0]:
            fc.callback(payload_packet(chunks[index]))
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            result = service.status()
            if result['phase'] in ('accepted', 'plan_failed', 'plan_late', 'rejected', 'service_failed'):
                break
            time.sleep(.001)
        assert result['phase'] == 'accepted', result
        assert fc.state.committed and sum(item[2] for item in fc.commits) == 1
        assert fc.state.start == (0xffff0000 + 80000) & 0xffffffff
        assert 0 < fc.plan.duration <= 1.6
        ref, rate, acc, jerk = F3(), F3(), F3(), F3()
        assert fc.lib.jointEvaluate(C.byref(fc.plan), C.c_float(0), ref, rate, acc, jerk)
        for i in range(3):
            assert abs(ref[i] - fc.snapshot.ref[i]) < 1e-4
            assert abs(rate[i]) < 1e-4
        saved_plan, commit = bytes(fc.plan), fc.commits[-1][0]
        code, scheduled = fc.commit(commit, (fc.state.start + 100000) & 0xffffffff, False)
        assert (code, scheduled) == (0, False)
        assert bytes(fc.plan) == saved_plan
        # Wrong session after successful commit is not idempotent authorization.
        wrong = bytearray(commit); wrong[4] ^= 1
        assert fc.commit(bytes(wrong), fc.now())[0] != 0
        return {'phase': result['phase'], 'wrapped_start_us': fc.state.start,
                'duration_s': fc.plan.duration, 'compute_s': result['compute_s'],
                'snapshot_to_plan_s': result['snapshot_to_plan_s'],
                'worker_pid': service._process.pid,
                'plan_body': bytes(fc.state.plan[:56]), 'parts': fc.parts,
                'commit': commit}
    finally:
        service.close()


def check_adversarial_boundaries(library, successful):
    for lead, should_accept in ((5000, True), (4999, False), (0, False), (-1, False)):
        fc = Fc(library); fc.begin()
        for part in reversed(successful['parts']):
            assert fc.accept_part(part) == 0
            assert fc.accept_part(part) == 0
        start = (fc.snapshot.epoch + PLAN_BODY.unpack(successful['plan_body'])[0]) & 0xffffffff
        code, scheduled = fc.commit(successful['commit'], (start - lead) & 0xffffffff)
        assert (code == 0 and scheduled) == should_accept, (lead, code, scheduled)
    fc = Fc(library); fc.begin()
    for part in successful['parts'][:-1]:
        assert fc.accept_part(part) == 0
    assert fc.commit(successful['commit'], fc.now())[0] == errno.EAGAIN
    assert not fc.state.committed
    fc = Fc(library); fc.begin()
    for part in successful['parts']:
        assert fc.accept_part(part) == 0
    assert fc.commit(successful['commit'], fc.now(), False)[0] == errno.EBUSY
    assert not fc.state.committed
    fc = Fc(library); fc.begin()
    first = successful['parts'][0]
    assert fc.accept_part(first) == 0
    conflict = first[:-1] + bytes([first[-1] ^ 1])
    assert fc.accept_part(conflict) != 0
    assert fc.state.mask == 0
    # A valid-CRC NaN endpoint must not reach the scheduled plan.
    fc = Fc(library); fc.begin()
    values = list(PLAN_BODY.unpack(successful['plan_body'])); values[-1] = float('nan')
    bad = PLAN_BODY.pack(*values)
    for part in encode_chunks(PLAN_CHUNK, fc.state.session, fc.state.sequence, fc.state.token, bad):
        assert fc.accept_part(part) == 0
    commit = COMMIT.pack(PLAN_COMMIT, VERSION, fc.state.sequence, fc.state.session,
                         fc.state.token, zlib.crc32(bad))
    assert fc.commit(commit, fc.now())[0] != 0 and not fc.state.committed
    # CRC-valid finite commands cannot discontinuously replace the actual
    # rapid-brake reference or invent nonzero initial reference rates.
    for index in (3, 6):
        fc = Fc(library); fc.begin()
        values = list(PLAN_BODY.unpack(successful['plan_body'])); values[index] += 1.0
        bad = PLAN_BODY.pack(*values)
        for part in encode_chunks(PLAN_CHUNK, fc.state.session, fc.state.sequence, fc.state.token, bad):
            assert fc.accept_part(part) == 0
        commit = COMMIT.pack(PLAN_COMMIT, VERSION, fc.state.sequence, fc.state.session,
                             fc.state.token, zlib.crc32(bad))
        assert fc.commit(commit, fc.now())[0] != 0 and not fc.state.committed


def main():
    assert C.sizeof(Snapshot) == 116
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
        for key in ('plan_body', 'parts', 'commit'):
            del result[key]
        print(json.dumps({'passed': True, 'scope': 'production C / spawned Pi protocol only',
                          **result}, sort_keys=True))
        print('PASS')


if __name__ == '__main__':
    main()
