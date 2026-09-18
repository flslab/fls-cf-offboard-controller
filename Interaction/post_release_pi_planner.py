"""Opt-in, event-only FC snapshot -> independent Pi planner -> FC schedule.

No periodic state subscription is created. ``start`` compiles/warms the native
kernel and a separate spawn process before arming. Packet callbacks only
validate/assemble/queue; a service thread handles ACKs, planning and sends.
The FC owns timing/late rejection and the bounded emergency level return.
``status`` is JSON-safe; ``ready``, ``phase`` and ``error`` are stable fields.
"""

from __future__ import annotations

import ctypes
import math
import multiprocessing
from pathlib import Path
import queue
import shutil
import struct
import subprocess
import sys
import tempfile
import threading
import time
import zlib

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

VERSION = 1
SNAPSHOT = 19
SNAPSHOT_ACK = 20
PLAN_CHUNK = 21
PLAN_COMMIT = 22
PLAN_RESULT = 23
CHUNK = struct.Struct('<BBHIHBB')
ACK = struct.Struct('<BBHIH')
COMMIT = struct.Struct('<BBHIHI')
RESULT = struct.Struct('<BBHIHHI')
SNAPSHOT_BODY = struct.Struct('<IIHH26f')
PLAN_BODY = struct.Struct('<If12f')
DATA_BYTES = 18
DEFAULT_START_DELAY_US = 80_000
MIN_START_DELAY_US = 20_000
LOCAL_SEND_MARGIN_US = 10_000
MAX_SNAPSHOT_WINDOW_US = 1_000_000


def forward_delta_us(later, earlier):
    """FC-only modular interval; never subtract a Pi timestamp from an FC one."""
    return (int(later) - int(earlier)) & 0xffffffff


def encode_chunks(kind, session_id, sequence, token, body):
    body = bytes(body)
    framed = body + struct.pack('<I', zlib.crc32(body) & 0xffffffff)
    count = (len(framed) + DATA_BYTES - 1) // DATA_BYTES
    if count > 255:
        raise ValueError('body too large')
    return [CHUNK.pack(kind, VERSION, sequence, session_id, token, index, count)
            + framed[index * DATA_BYTES:(index + 1) * DATA_BYTES]
            for index in range(count)]


class FragmentAssembler:
    """Fixed-size assembly with exact identity/length/CRC and conflict latch."""

    def __init__(self, kind, session_id, sequence, body_size):
        self.kind, self.session_id, self.sequence = kind, session_id, sequence
        self.size = body_size + 4
        self.count = (self.size + DATA_BYTES - 1) // DATA_BYTES
        self.token = None
        self.parts = {}
        self.rejected = False
        self.first_receive_s = None

    def accept(self, data, *, now_s=None):
        data = bytes(data)
        if len(data) < CHUNK.size:
            return None
        kind, version, seq, session, token, index, count = CHUNK.unpack_from(data)
        if (kind != self.kind or version != VERSION or seq != self.sequence or
                session != self.session_id):
            return None
        if self.rejected:
            return None
        if count != self.count or index >= count:
            raise ValueError('invalid fragment count/index')
        expected = min(DATA_BYTES, self.size - index * DATA_BYTES)
        payload = data[CHUNK.size:]
        if len(payload) != expected:
            raise ValueError('invalid fragment length')
        if self.token is None:
            self.token = token
            self.first_receive_s = time.monotonic() if now_s is None else now_s
        elif token != self.token:
            # The FC is allowed only one immutable snapshot per release.
            raise ValueError('snapshot token changed within release')
        if index in self.parts and self.parts[index] != payload:
            self.rejected = True
            self.parts.clear()
            raise ValueError('conflicting duplicate fragment')
        self.parts[index] = payload
        if len(self.parts) != count:
            return None
        body = b''.join(self.parts[i] for i in range(count))
        if zlib.crc32(body[:-4]) & 0xffffffff != struct.unpack('<I', body[-4:])[0]:
            self.rejected = True
            raise ValueError('fragment CRC mismatch')
        return body[:-4]


def decode_snapshot(body):
    if len(body) != SNAPSHOT_BODY.size:
        raise ValueError('snapshot size mismatch')
    epoch, latest, state_age_us, attitude_age_us, *values = SNAPSHOT_BODY.unpack(body)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('snapshot contains nonfinite state')
    if state_age_us > 40_000 or attitude_age_us > 20_000:
        raise ValueError('snapshot state/attitude age exceeds protocol bound')
    window = forward_delta_us(latest, epoch)
    if not MIN_START_DELAY_US <= window <= MAX_SNAPSHOT_WINDOW_US:
        raise ValueError('snapshot has no usable future start window')
    if abs(math.hypot(*values[8:10]) - 1.0) > .01:
        raise ValueError('snapshot direction is not normalized')
    return epoch, latest, state_age_us, attitude_age_us, values


class NativePlanner:
    """Build the vendored firmware numerical kernel, never flight I/O."""

    def __init__(self):
        self._directory = tempfile.TemporaryDirectory(prefix='fls-pi-joint-')
        source = Path(__file__).with_name('native') / 'post_release'
        compiler = shutil.which('cc') or shutil.which('gcc')
        if compiler is None:
            self._directory.cleanup()
            raise RuntimeError('pi_joint requires a C compiler before arm')
        library = Path(self._directory.name) / ('joint.dylib' if sys.platform == 'darwin' else 'joint.so')
        command = [compiler, '-std=c11', '-O2', '-Wall', '-Wextra', '-Werror',
                   '-dynamiclib' if sys.platform == 'darwin' else '-shared', '-fPIC',
                   '-I' + str(source), str(source / 'pi_joint_bridge.c'),
                   str(source / 'post_release_joint_unwind.c'),
                   str(source / 'post_release_forward_stop.c'), '-lm', '-o', str(library)]
        try:
            subprocess.run(command, check=True, capture_output=True, text=True, timeout=20)
            self._library = ctypes.CDLL(str(library))
            self._solve = self._library.piJointSolve
            self._solve.argtypes = [ctypes.POINTER(ctypes.c_float), ctypes.c_float,
                                   ctypes.POINTER(ctypes.c_float)]
            self._solve.restype = ctypes.c_int
            # A real kernel call before ready avoids first-use runtime cost at release.
            warm = [0.] * 26
            warm[8] = 1.
            warm[14:17] = [6., 7.1, 6.]
            warm[17:20] = [1., 1., 1.]
            warm[20:23] = [.05, .07, .08]
            self._solve((ctypes.c_float * 26)(*warm), .08, (ctypes.c_float * 17)())
        except BaseException:
            self._directory.cleanup()
            raise

    def solve(self, body):
        epoch, latest, state_age_us, attitude_age_us, values = decode_snapshot(body)
        delay_us = min(DEFAULT_START_DELAY_US, forward_delta_us(latest, epoch))
        out = (ctypes.c_float * 17)()
        started = time.monotonic()
        solved = self._solve((ctypes.c_float * 26)(*values), delay_us / 1e6, out)
        compute_s = time.monotonic() - started
        if not solved:
            raise ValueError('no feasible joint plan from event snapshot')
        duration, *parameters = list(out)[:13]
        if not all(math.isfinite(v) for v in out):
            raise ValueError('native planner returned nonfinite plan')
        return {'body': PLAN_BODY.pack(delay_us, duration, *parameters),
                'snapshot_us': epoch, 'start_delay_us': delay_us,
                'trusted_state_age_us': state_age_us, 'attitude_age_us': attitude_age_us,
                'start_us': (epoch + delay_us) & 0xffffffff,
                'duration_s': duration, 'compute_s': compute_s,
                'predicted_terminal_speed_mps': out[13],
                'predicted_min_forward_mps': out[14],
                'predicted_start_speed_mps': out[15]}

    def close(self):
        self._directory.cleanup()


def _planner_process(connection):
    planner = None
    try:
        planner = NativePlanner()
        connection.send(('ready', None))
        while True:
            message = connection.recv()
            if message is None:
                break
            generation, body = message
            try:
                connection.send(('result', (generation, planner.solve(body), None)))
            except Exception as exc:
                connection.send(('result', (generation, None, str(exc))))
    except (EOFError, BrokenPipeError):
        pass
    except Exception as exc:
        try:
            connection.send(('startup_error', str(exc)))
        except (EOFError, BrokenPipeError):
            pass
    finally:
        if planner is not None:
            planner.close()
        connection.close()


class PiEventPlanner:
    """Long-lived service; one immutable planning job per armed release."""

    def __init__(self, cf):
        self.cf = cf
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._events = queue.Queue(maxsize=16)
        self._process = self._thread = self._connection = None
        self._callback = self._on_packet
        self._listening = False
        self._generation = 0
        self._assembly = None
        self._snapshot_queued = False
        self._ack_retry_queued = False
        self._result_seen = False
        self._sent_plan = None
        self._outbound = []
        self._process_jobs = []
        self._status = {'ready': False, 'phase': 'not_started', 'error': None}

    def start(self, timeout_s=25.):
        if self._process is not None:
            if self.status()['ready']:
                return
            raise RuntimeError('Pi planner already started but is not healthy')
        context = multiprocessing.get_context('spawn')
        parent, child = context.Pipe()
        self._connection = parent
        self._process = context.Process(target=_planner_process, args=(child,),
                                        name='post-release-planner', daemon=True)
        try:
            self._process.start()
            child.close()
            if not parent.poll(timeout_s):
                raise RuntimeError('Pi planner prewarm timed out before arm')
            kind, payload = parent.recv()
            if kind != 'ready':
                raise RuntimeError('Pi planner prewarm failed: %s' % payload)
            self.cf.add_port_callback(CRTPPort.SETPOINT_HL, self._callback)
            self._listening = True
            self._thread = threading.Thread(target=self._run, name='post-release-plan-link', daemon=True)
            self._status.update(ready=True, phase='ready', error=None)
            self._thread.start()
        except BaseException:
            self.close()
            raise

    def begin_release(self, session_id, sequence):
        if not (isinstance(session_id, int) and 0 <= session_id <= 0xffffffff and
                isinstance(sequence, int) and 0 <= sequence <= 0xffff):
            raise ValueError('invalid release identity')
        with self._lock:
            if not self.status()['ready']:
                raise RuntimeError('Pi event planner is not prewarmed/healthy')
            self._generation += 1
            self._assembly = FragmentAssembler(SNAPSHOT, session_id, sequence, SNAPSHOT_BODY.size)
            self._snapshot_queued = self._result_seen = False
            self._ack_retry_queued = False
            self._sent_plan = None
            self._status = {'ready': True, 'phase': 'waiting_snapshot', 'error': None,
                            'session_id': session_id, 'sequence': sequence,
                            'generation': self._generation, 'release_begin_s': time.monotonic(),
                            'snapshot_ack_count': 0, 'commit_send_count': 0,
                            'snapshot_fragment_count': 0, 'snapshot_duplicate_count': 0}

    def status(self):
        with self._lock:
            result = dict(self._status)
            result['ready'] = bool(result.get('ready') and not self._stop.is_set()
                                   and self._process and self._process.is_alive()
                                   and self._thread and self._thread.is_alive())
            return result

    def _on_packet(self, packet):
        if packet.port != CRTPPort.SETPOINT_HL or packet.channel != 1:
            return
        data = bytes(packet.data)
        if not data:
            return
        with self._lock:
            assembly = self._assembly
            if assembly is None or self._stop.is_set():
                return
            generation = self._generation
            try:
                if data[0] == SNAPSHOT:
                    before = len(assembly.parts)
                    body = assembly.accept(data)
                    if len(assembly.parts) > before:
                        self._status['snapshot_fragment_count'] += 1
                    elif body is not None:
                        self._status['snapshot_duplicate_count'] += 1
                    if body is not None and not self._snapshot_queued:
                        decode_snapshot(body)
                        self._events.put_nowait(('snapshot', generation, body))
                        self._snapshot_queued = True
                    elif (body is not None and not self._ack_retry_queued and
                          self._status['snapshot_ack_count'] < 3):
                        self._events.put_nowait(('snapshot_retry', generation, None))
                        self._ack_retry_queued = True
                elif data[0] == PLAN_RESULT and len(data) == RESULT.size:
                    values = RESULT.unpack(data)
                    if (values[1] == VERSION and values[2] == assembly.sequence and
                            values[3] == assembly.session_id and values[4] == assembly.token):
                        self._events.put_nowait(('result', generation, values))
            except (ValueError, queue.Full) as exc:
                self._status.update(phase='snapshot_rejected', error=str(exc))

    def _send(self, payload):
        # Only queue while the state lock is held. cflib invokes packet-sent
        # hooks synchronously; doing its I/O under our lock could stall the
        # receive callback or form a lock cycle with another subscriber.
        self._outbound.append((self._generation, bytes(payload)))

    def _flush_io(self):
        with self._lock:
            outgoing, jobs = self._outbound, self._process_jobs
            self._outbound, self._process_jobs = [], []
        for generation, payload in outgoing:
            with self._lock:
                current = generation == self._generation and not self._stop.is_set()
            if current:
                self._send_packet(payload)
                with self._lock:
                    if generation == self._generation and payload[0] == PLAN_COMMIT:
                        sent_s = time.monotonic()
                        self._status['last_commit_send_s'] = sent_s
                        self._status.setdefault('first_commit_send_s', sent_s)
                    elif generation == self._generation and payload[0] == SNAPSHOT_ACK:
                        self._status['last_snapshot_ack_send_s'] = time.monotonic()
        for generation, payload in jobs:
            with self._lock:
                current = generation == self._generation and not self._stop.is_set()
            if current:
                self._connection.send((generation, payload))

    def _send_packet(self, payload):
        packet = CRTPPacket()
        packet.set_header(CRTPPort.SETPOINT_HL, 0)
        packet.data = payload
        self.cf.send_packet(packet)

    def _snapshot_ack(self):
        assembly = self._assembly
        if self._status['snapshot_ack_count'] >= 3:
            return
        self._send(ACK.pack(SNAPSHOT_ACK, VERSION, assembly.sequence,
                            assembly.session_id, assembly.token))
        self._status['snapshot_ack_count'] += 1

    def _handle_event(self, kind, generation, payload):
        if generation != self._generation:
            return
        if kind == 'snapshot':
            if self._assembly.rejected:
                return
            self._snapshot_ack()
            self._status.update(phase='planning', snapshot_token=self._assembly.token,
                                snapshot_received_s=self._assembly.first_receive_s)
            self._process_jobs.append((generation, payload))
        elif kind == 'snapshot_retry':
            self._ack_retry_queued = False
            self._snapshot_ack()
        elif kind == 'result':
            # An unsolicited result cannot complete an unsent plan, and a
            # duplicate must not restart/alter a completed transaction.
            if self._sent_plan is None or self._result_seen:
                return
            errno, accepted = payload[5:7]
            # Wire errno follows ARM/newlib (EAGAIN=11), not the host OS's
            # errno.EAGAIN (35 on macOS). A commit may overtake a missing
            # fragment: resend the IDENTICAL plan, still with its original
            # epoch and the existing three-send/deadline bound.
            if errno == 11:
                remaining_us = (self._sent_plan['start_delay_us'] -
                                (time.monotonic() - self._assembly.first_receive_s) * 1e6)
                if (self._status['commit_send_count'] < 3 and
                        remaining_us > LOCAL_SEND_MARGIN_US):
                    self._status.update(phase='awaiting_result', firmware_errno=errno,
                                        last_commit_send_s=time.monotonic() - .021,
                                        incomplete_plan_replies=self._status.get('incomplete_plan_replies', 0) + 1)
                    return
            if errno == 0 and accepted != self._sent_plan['start_us']:
                self._status.update(phase='result_rejected', error='FC accepted unexpected plan epoch')
                self._result_seen = True
                return
            self._result_seen = True
            self._status.update(phase='accepted' if errno == 0 else 'rejected',
                                firmware_errno=errno, accepted_start_us=accepted,
                                result_received_s=time.monotonic(),
                                error=None if errno == 0 else 'FC rejected plan (errno=%s)' % errno)

    def _handle_plan(self, generation, plan, error):
        if generation != self._generation:
            return
        if self._assembly is not None and self._assembly.rejected:
            self._status.update(phase='snapshot_rejected', error='snapshot assembly rejected')
            return
        if error:
            self._status.update(phase='plan_failed', error=error)
            return
        elapsed_us = (time.monotonic() - self._assembly.first_receive_s) * 1e6
        if elapsed_us + LOCAL_SEND_MARGIN_US >= plan['start_delay_us']:
            self._status.update(phase='plan_late', error='Pi compute/queue missed bounded send lead',
                                snapshot_to_plan_s=elapsed_us / 1e6)
            return
        self._sent_plan = plan
        self._status.update({key: value for key, value in plan.items() if key != 'body'})
        self._status.update(phase='awaiting_result', snapshot_to_plan_s=elapsed_us / 1e6,
                            error=None)
        self._send_plan()

    def _send_plan(self):
        assembly, plan = self._assembly, self._sent_plan
        for packet in encode_chunks(PLAN_CHUNK, assembly.session_id,
                                    assembly.sequence, assembly.token, plan['body']):
            self._send(packet)
        self._send(COMMIT.pack(PLAN_COMMIT, VERSION, assembly.sequence, assembly.session_id,
                               assembly.token, zlib.crc32(plan['body']) & 0xffffffff))
        self._status['commit_send_count'] += 1
        self._status['last_commit_send_s'] = time.monotonic()

    def _tick(self):
        now = time.monotonic()
        if self._status.get('phase') == 'waiting_snapshot' and now - self._status['release_begin_s'] > 1.0:
            self._status.update(phase='snapshot_timeout', error='no complete FC event snapshot within 1 s')
        if (self._status.get('phase') == 'planning' and
                now - self._assembly.first_receive_s > .5):
            self._status.update(phase='plan_timeout', error='Pi process exceeded bounded planning interval')
        if self._sent_plan is not None and not self._result_seen:
            age_us = (now - self._assembly.first_receive_s) * 1e6
            if age_us + LOCAL_SEND_MARGIN_US >= self._sent_plan['start_delay_us']:
                if self._status.get('phase') == 'awaiting_result':
                    self._status.update(phase='result_timeout', error='FC plan acceptance not received before start')
            elif (now - self._status['last_commit_send_s'] >= .02 and
                  self._status['commit_send_count'] < 3):
                self._send_plan()

    def _run(self):
        try:
            while not self._stop.is_set():
                try:
                    event = self._events.get(timeout=.002)
                except queue.Empty:
                    event = None
                result_message = self._connection.recv() if self._connection.poll() else None
                with self._lock:
                    if event is not None:
                        self._handle_event(*event)
                    if result_message is not None:
                        kind, result = result_message
                        if kind != 'result':
                            raise RuntimeError('unexpected planner process reply: %s' % kind)
                        self._handle_plan(*result)
                    self._tick()
                self._flush_io()
        except Exception as exc:
            with self._lock:
                self._status.update(ready=False, phase='service_failed', error=str(exc))

    def close(self):
        self._stop.set()
        if self._listening:
            self.cf.remove_port_callback(CRTPPort.SETPOINT_HL, self._callback)
            self._listening = False
        if self._thread and self._thread is not threading.current_thread():
            self._thread.join(timeout=1.)
        if self._connection:
            try:
                self._connection.send(None)
            except (EOFError, BrokenPipeError, OSError):
                pass
        if self._process and self._process.pid is not None:
            self._process.join(timeout=1.)
            if self._process.is_alive():
                self._process.terminate()
                self._process.join(timeout=1.)
        if self._connection:
            self._connection.close()
        with self._lock:
            self._status.update(ready=False, phase='closed')
