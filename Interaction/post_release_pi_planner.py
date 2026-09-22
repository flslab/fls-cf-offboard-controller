"""Opt-in, event-only FC snapshot -> independent Pi planner -> FC schedule.

No periodic state subscription is created. ``start`` compiles/warms the native
kernel and a separate spawn process before arming. Packet callbacks only
validate/assemble/queue; a service thread handles planning and sends.
The FC owns timing/late rejection and the bounded emergency level return.
``status`` is JSON-safe; ``ready``, ``phase`` and ``error`` are stable fields.
"""

from __future__ import annotations

import ctypes
from collections import deque
import math
import multiprocessing
from multiprocessing.connection import wait
from pathlib import Path
import queue
import secrets
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time
import zlib

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort

VERSION = 3
SNAPSHOT = 19
SNAPSHOT_ACK = 20
PLAN_CHUNK = 21
PLAN_COMMIT = 22
PLAN_RESULT = 23
MODEL_REQUEST = 24
MODEL_PART = 25
MODEL_BODY = struct.Struct('<9f')
MODEL_QUERY = struct.Struct('<BBI')
CHUNK = struct.Struct('<BBHIHBB')
ACK = struct.Struct('<BBHIH')
COMMIT = struct.Struct('<BBHIHI')
RESULT = struct.Struct('<BBHIHHIII')
SNAPSHOT_BODY = struct.Struct('<IIHH28f')
PLAN_BODY = struct.Struct('<If12f')
# Expanded bodies above are numerical-kernel interfaces, not v3 wire layouts.
WIRE_SNAPSHOT_BODY = struct.Struct('<IIHHI19f')
WIRE_PLAN_BODY = struct.Struct('<If6f')
DATA_BYTES = 18
DEFAULT_START_DELAY_US = 40_000
MIN_START_DELAY_US = 20_000
LOCAL_SEND_MARGIN_US = 10_000
MAX_SNAPSHOT_WINDOW_US = 1_000_000


def validate_model(body):
    if len(body) != MODEL_BODY.size:
        raise ValueError('model size mismatch')
    values = MODEL_BODY.unpack(body)
    if (not all(math.isfinite(v) for v in values) or
            not all(0 < v <= 20 for v in values[:3]) or
            not all(abs(v) <= 20 for v in values[3:6]) or
            not all(struct.unpack('<f', struct.pack('<f', .01))[0] <= v <=
                    struct.unpack('<f', struct.pack('<f', .20))[0] for v in values[6:])):
        raise ValueError('invalid cached model')
    return zlib.crc32(body) & 0xffffffff


def compact_snapshot(body):
    """Fixture/helper encoder matching FC; no quantization of dynamic state."""
    epoch, latest, va, aa, *v = SNAPSHOT_BODY.unpack(body)
    model = MODEL_BODY.pack(*v[14:23])
    return WIRE_SNAPSHOT_BODY.pack(epoch, latest, va, aa, validate_model(model),
                                   *(v[:14] + v[23:]))


def expand_snapshot(body, cached_model):
    epoch, latest, va, aa, model_id, *v = WIRE_SNAPSHOT_BODY.unpack(body)
    if cached_model is None or validate_model(cached_model) != model_id:
        raise ValueError('snapshot model changed or was not synchronized before arm')
    expanded = SNAPSHOT_BODY.pack(epoch, latest, va, aa,
        *(v[:14] + list(MODEL_BODY.unpack(cached_model)) + v[14:]))
    decode_snapshot(expanded)
    return expanded


def compact_plan(body):
    delay, duration, *v = PLAN_BODY.unpack(body)
    return WIRE_PLAN_BODY.pack(delay, duration, *v[6:])


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
            # Each assembler owns one immutable prefix token. The service
            # replaces it explicitly when a newer FC prefix supersedes it.
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
    elapsed, duration = values[26:28]
    if (elapsed < 0 or duration < 0 or duration > 1.6 or
            (duration == 0 and elapsed != 0) or
            (duration > 0 and (duration < .08 or elapsed > duration + .5))):
        raise ValueError('invalid local reference prefix')
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
            warm = [0.] * 28
            warm[8] = 1.
            warm[14:17] = [6., 7.1, 6.]
            warm[17:20] = [1., 1., 1.]
            warm[20:23] = [.05, .07, .08]
            self._solve((ctypes.c_float * 28)(*warm), .04, (ctypes.c_float * 17)())
        except BaseException:
            self._directory.cleanup()
            raise

    def solve(self, body):
        epoch, latest, state_age_us, attitude_age_us, values = decode_snapshot(body)
        delay_us = min(DEFAULT_START_DELAY_US, forward_delta_us(latest, epoch))
        out = (ctypes.c_float * 17)()
        started = time.monotonic()
        solved = self._solve((ctypes.c_float * 28)(*values), delay_us / 1e6, out)
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
                'prefix_elapsed_s': values[26], 'prefix_duration_s': values[27],
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
    """Event-only service; a new FC prefix token supersedes the previous job."""

    def __init__(self, cf):
        self.cf = cf
        self._lock = threading.RLock()
        self._stop = threading.Event()
        self._events = queue.Queue(maxsize=16)
        self._process = self._thread = self._connection = None
        self._wake_read = self._wake_write = None
        self._callback = self._on_packet
        self._listening = False
        self._generation = 0
        self._assembly = None
        self._snapshot_queued = False
        self._result_seen = False
        self._sent_plan = None
        self._outbound = []
        self._process_jobs = []
        self._status = {'ready': False, 'phase': 'not_started', 'error': None}
        self._model_body = None
        self._model_assembly = None
        self._model_event = threading.Event()
        self._model_error = None
        self._diagnostics = deque(maxlen=128)
        self._diagnostics_dropped = 0

    def _record_diagnostic(self, event, **fields):
        # Caller holds _lock. No file/network I/O on the timing-critical path.
        if len(self._diagnostics) == self._diagnostics.maxlen:
            self._diagnostics_dropped += 1
        self._diagnostics.append(dict(schema='pi_plan_evidence_v1', event=event,
            pi_monotonic_ns=time.monotonic_ns(), generation=self._generation,
            session_id=self._status.get('session_id'), sequence=self._status.get('sequence'),
            token=self._assembly.token if self._assembly else None, **fields))

    def drain_diagnostics(self):
        with self._lock:
            rows = list(self._diagnostics)
            self._diagnostics.clear()
            if self._diagnostics_dropped:
                rows.insert(0, dict(schema='pi_plan_evidence_v1', event='buffer_overflow',
                                    dropped=self._diagnostics_dropped))
                self._diagnostics_dropped = 0
            return rows

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
            self._wake_read, self._wake_write = socket.socketpair()
            self._wake_read.setblocking(False)
            self._wake_write.setblocking(False)
            self.cf.add_port_callback(CRTPPort.SETPOINT_HL, self._callback)
            self._listening = True
            self._thread = threading.Thread(target=self._run, name='post-release-plan-link', daemon=True)
            self._status.update(ready=True, phase='ready', error=None)
            self._thread.start()
            self.sync_model()
        except BaseException:
            self.close()
            raise

    def sync_model(self, timeout_s=3.):
        """Startup-only bounded model read. No release or controller authority."""
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('invalid model synchronization timeout')
        with self._lock:
            if self._assembly is not None:
                raise RuntimeError('model synchronization is only allowed before release')
            nonce = secrets.randbits(32)
            self._model_body = None
            self._model_error = None
            self._model_event.clear()
            self._model_assembly = FragmentAssembler(MODEL_PART, nonce, 0, MODEL_BODY.size)
        deadline = time.monotonic() + timeout_s
        try:
            for _ in range(6):
                self._send_packet(MODEL_QUERY.pack(MODEL_REQUEST, VERSION, nonce))
                if self._model_event.wait(max(0., min(.5, deadline-time.monotonic()))):
                    break
                if time.monotonic() >= deadline:
                    break
            with self._lock:
                if self._model_error or self._model_body is None:
                    raise RuntimeError('startup model sync failed: ' + (self._model_error or 'timeout'))
                model_id = validate_model(self._model_body)
                self._status['model_id'] = model_id
                return model_id
        finally:
            with self._lock:
                self._model_assembly = None

    def begin_release(self, session_id, sequence):
        if not (isinstance(session_id, int) and 0 <= session_id <= 0xffffffff and
                isinstance(sequence, int) and 0 <= sequence <= 0xffff):
            raise ValueError('invalid release identity')
        with self._lock:
            if not self.status()['ready']:
                raise RuntimeError('Pi event planner is not prewarmed/healthy')
            self._generation += 1
            self._assembly = FragmentAssembler(SNAPSHOT, session_id, sequence, WIRE_SNAPSHOT_BODY.size)
            self._snapshot_queued = self._result_seen = False
            self._sent_plan = None
            self._status = {'ready': True, 'phase': 'waiting_snapshot', 'error': None,
                            'session_id': session_id, 'sequence': sequence,
                            'generation': self._generation, 'release_begin_s': time.monotonic(),
                            'plan_send_count': 0, 'superseded_snapshot_count': 0,
                            'snapshot_fragment_count': 0, 'snapshot_duplicate_count': 0}

    def status(self):
        with self._lock:
            result = dict(self._status)
            result['model_id'] = (validate_model(self._model_body)
                                  if self._model_body is not None else None)
            result['ready'] = bool(result.get('ready') and not self._stop.is_set()
                                   and self._process and self._process.is_alive()
                                   and self._thread and self._thread.is_alive()
                                   and self._model_body is not None)
            return result

    def _on_packet(self, packet):
        if packet.port != CRTPPort.SETPOINT_HL or packet.channel != 1:
            return
        data = bytes(packet.data)
        if not data:
            return
        with self._lock:
            if data[0] == MODEL_PART:
                if self._model_assembly is not None:
                    try:
                        body = self._model_assembly.accept(data)
                        if body is not None:
                            validate_model(body)
                            self._model_body = body
                            self._model_event.set()
                    except ValueError as exc:
                        self._model_error = str(exc)
                        self._model_event.set()
                return
            assembly = self._assembly
            if assembly is None or self._stop.is_set():
                return
            generation = self._generation
            try:
                if data[0] == SNAPSHOT:
                    if len(data) >= CHUNK.size:
                        _, version, seq, session, token, _, _ = CHUNK.unpack_from(data)
                        if (version != VERSION or seq != assembly.sequence or
                                session != assembly.session_id):
                            return
                        if assembly.token is not None and token != assembly.token:
                            if not 0 < ((token - assembly.token) & 0xffff) < 0x8000:
                                return  # delayed fragment from a superseded prefix
                            self._generation += 1
                            generation = self._generation
                            self._assembly = assembly = FragmentAssembler(
                                SNAPSHOT, session, seq, WIRE_SNAPSHOT_BODY.size)
                            self._snapshot_queued = self._result_seen = False
                            self._sent_plan = None
                            self._status = {
                                'ready': self._status.get('ready', False),
                                'phase': 'waiting_snapshot', 'error': None,
                                'session_id': session, 'sequence': seq,
                                'generation': generation,
                                'release_begin_s': self._status['release_begin_s'],
                                'superseded_snapshot_count': self._status.get('superseded_snapshot_count', 0) + 1,
                                'plan_send_count': 0, 'snapshot_fragment_count': 0,
                                'snapshot_duplicate_count': 0,
                            }
                    before = len(assembly.parts)
                    body = assembly.accept(data)
                    if len(assembly.parts) > before:
                        self._status['snapshot_fragment_count'] += 1
                    elif body is not None:
                        self._status['snapshot_duplicate_count'] += 1
                    if body is not None and not self._snapshot_queued:
                        wire_body = body
                        body = expand_snapshot(wire_body, self._model_body)
                        self._record_diagnostic('snapshot', protocol_version=VERSION,
                            wire_body_hex=wire_body.hex(), expanded_body_hex=body.hex(),
                            model_body_hex=self._model_body.hex(),
                            model_id=validate_model(self._model_body),
                            first_fragment_pi_monotonic_s=assembly.first_receive_s)
                        self._queue_event(('snapshot', generation, body))
                        self._snapshot_queued = True
                elif data[0] == PLAN_RESULT and len(data) == RESULT.size:
                    values = RESULT.unpack(data)
                    if (values[1] == VERSION and values[2] == assembly.sequence and
                            values[3] == assembly.session_id and values[4] == assembly.token):
                        self._queue_event(('result', generation, values))
            except (ValueError, queue.Full) as exc:
                self._status.update(phase='snapshot_rejected', error=str(exc))

    def _notify_work(self):
        writer = self._wake_write
        if writer is not None:
            try:
                writer.send(b'\0')
            except BlockingIOError:
                # A full socket already has an unread wakeup. The queue, not
                # the number of wake bytes, owns the pending events.
                pass
            except OSError:
                if not self._stop.is_set():
                    raise

    def _queue_event(self, event):
        self._events.put_nowait(event)
        self._notify_work()

    def _wait_for_work(self, timeout_s=.002):
        # Wait on BOTH the process result pipe and callback wake socket. A
        # completed plan used to wait for Queue.get(timeout=.002) to expire.
        # Keep that timeout for unchanged retry/deadline maintenance, but
        # process results and FC packets now interrupt the wait immediately.
        if not self._events.empty() or self._stop.is_set():
            return
        ready = wait([self._connection, self._wake_read], timeout=timeout_s)
        if self._wake_read in ready:
            try:
                self._wake_read.recv(4096)
            except BlockingIOError:
                pass

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
                    if (generation == self._generation and payload[0] == PLAN_CHUNK
                            and payload[10] == payload[11] - 1):
                        sent_s = time.monotonic()
                        self._status['last_plan_send_s'] = sent_s
                        self._status.setdefault('first_plan_send_s', sent_s)
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

    def _handle_event(self, kind, generation, payload):
        if generation != self._generation:
            return
        if kind == 'snapshot':
            if self._assembly.rejected:
                return
            self._status.update(phase='planning', snapshot_token=self._assembly.token,
                                snapshot_received_s=self._assembly.first_receive_s)
            self._process_jobs.append((generation, payload))
        elif kind == 'result':
            # An unsolicited result cannot complete an unsent plan, and a
            # duplicate must not restart/alter a completed transaction.
            if self._sent_plan is None:
                return
            self._record_diagnostic('firmware_result', result_body_hex=RESULT.pack(*payload).hex())
            errno, accepted = payload[5:7]
            received_us, executed_us = payload[7:9]
            # Execution is a second asynchronous notice, not another handshake.
            # An execution notice can arrive before its acceptance notice.
            if self._result_seen and self._status.get('phase') not in ('accepted', 'executing'):
                return
            if self._result_seen and (errno != 0 or not executed_us):
                return
            if self._status.get('fc_first_curve_us'):
                return  # first execution timestamp is immutable
            # Wire errno follows ARM/newlib (EAGAIN=11), not the host OS's
            # errno.EAGAIN (35 on macOS). A commit may overtake a missing
            # fragment: resend the IDENTICAL plan, still with its original
            # epoch and the existing three-send/deadline bound.
            if errno == 11:
                remaining_us = (self._sent_plan['start_delay_us'] -
                                (time.monotonic() - self._assembly.first_receive_s) * 1e6)
                if (self._status['plan_send_count'] < 3 and
                        remaining_us > LOCAL_SEND_MARGIN_US):
                    self._status.update(phase='awaiting_result', firmware_errno=errno,
                                        last_plan_send_s=time.monotonic() - .021,
                                        incomplete_plan_replies=self._status.get('incomplete_plan_replies', 0) + 1)
                    return
            if errno == 0 and accepted != self._sent_plan['start_us']:
                self._status.update(phase='result_rejected', error='FC accepted unexpected plan epoch')
                self._result_seen = True
                return
            self._result_seen = True
            self._status.update(phase=('executing' if executed_us else 'accepted') if errno == 0 else 'rejected',
                                firmware_errno=errno, accepted_start_us=accepted,
                                fc_plan_received_us=received_us, fc_first_curve_us=executed_us,
                                result_received_s=time.monotonic(),
                                error=None if errno == 0 else 'FC rejected plan (errno=%s)' % errno)
            if errno == 0 and self._sent_plan.get('snapshot_us') is not None:
                self._status['fc_snapshot_to_plan_ms'] = forward_delta_us(
                    received_us, self._sent_plan['snapshot_us']) / 1000.
                if executed_us:
                    self._status['fc_snapshot_to_first_curve_ms'] = forward_delta_us(
                        executed_us, self._sent_plan['snapshot_us']) / 1000.
                    self._status['fc_plan_to_first_curve_ms'] = forward_delta_us(
                        executed_us, received_us) / 1000.

    def _handle_plan(self, generation, plan, error):
        if generation != self._generation:
            return
        self._record_diagnostic('planner_result', error=error,
            expanded_plan_hex=plan['body'].hex() if plan and 'body' in plan else None,
            wire_plan_hex=compact_plan(plan['body']).hex() if plan and 'body' in plan else None,
            summary={key: value for key, value in (plan or {}).items() if key != 'body'})
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
                                    assembly.sequence, assembly.token, compact_plan(plan['body'])):
            self._send(packet)
        # FC reconstructs/schedules as soon as all CRC-checked parts arrive.
        self._status['plan_send_count'] += 1
        self._status['last_plan_send_s'] = time.monotonic()

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
            elif (now - self._status['last_plan_send_s'] >= .02 and
                  self._status['plan_send_count'] < 3):
                self._send_plan()

    def _run(self):
        try:
            while not self._stop.is_set():
                self._wait_for_work()
                if self._stop.is_set():
                    break
                try:
                    event = self._events.get_nowait()
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
        self._notify_work()
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
        for wake_socket in (self._wake_read, self._wake_write):
            if wake_socket is not None:
                wake_socket.close()
        self._wake_read = self._wake_write = None
        with self._lock:
            self._status.update(ready=False, phase='closed')
