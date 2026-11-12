"""Capture-clock to MP4 timing for rpicam/libcamera's libav encoder.

The encoder's verbose first-frame timestamp is its input capture timestamp in
microseconds. MP4 presentation timestamps retain offsets from that timestamp.
Never use Popen/ack receipt time as a substitute for the first captured frame.
"""
import hashlib
import json
import math
from pathlib import Path
import re
import subprocess
import threading
import time


FIRST_FRAME = re.compile(r'Video start timestamp\s*:\s*(\d+)\s+us')


def epoch_clock_status():
    try:
        result = subprocess.run(['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'],
                                capture_output=True, text=True, timeout=2, check=True)
        return dict(ntp_synchronized=result.stdout.strip() == 'yes')
    except (OSError, subprocess.SubprocessError):
        return dict(ntp_synchronized=None)


def clock_sample():
    before = time.monotonic_ns()
    epoch = time.time_ns()
    boot = time.clock_gettime_ns(time.CLOCK_BOOTTIME) if hasattr(time, 'CLOCK_BOOTTIME') else None
    after = time.monotonic_ns()
    return dict(epoch_ns=epoch, monotonic_ns=(before + after)//2,
                boottime_ns=boot if boot is not None else (before+after)//2, sample_span_ns=after-before)


def capture_epoch(capture_us, samples):
    """Map capture timestamp using the nearest paired host-clock observation.

    FrameWallClock builds supply epoch timestamps; older builds supply kernel
    monotonic timestamps. Accept only a domain consistent with this recording.
    Suspend-separated monotonic/boottime clocks are deliberately not guessed.
    """
    ns = int(capture_us)*1000
    offsets = [s['epoch_ns']-s['monotonic_ns'] for s in samples]
    drift_s = (max(offsets)-min(offsets))/1e9
    if drift_s > .005:
        raise ValueError('wall clock stepped/drifted by more than 5 ms during recording')
    sample = min(samples, key=lambda s: min(abs(ns-s['epoch_ns']), abs(ns-s['monotonic_ns'])))
    uncertainty = sample['sample_span_ns']/1e9 + drift_s
    if abs(ns-sample['epoch_ns']) < 60*1e9:
        return ns/1e9, 'encoder_epoch', uncertainty
    if abs(ns-sample['monotonic_ns']) > 60*1e9:
        raise ValueError('encoder capture clock does not match sampled host clocks')
    if abs(sample['boottime_ns']-sample['monotonic_ns']) > 1_000_000:
        raise ValueError('monotonic/boottime differ after suspend; capture clock domain is ambiguous')
    return (ns + sample['epoch_ns']-sample['monotonic_ns'])/1e9, 'encoder_monotonic', uncertainty


class RecordingTiming:
    def __init__(self, video, *, camera_id, run_id, request_id, command):
        self.video = Path(video)
        self.metadata_path = self.video.with_suffix('.timing.json')
        self.frames_path = self.video.with_suffix('.frames.jsonl')
        self.encoder_path = self.video.with_suffix('.encoder.log')
        self.samples = [clock_sample()]
        self.first_capture_us = None
        self.ready = threading.Event()
        self.stop = threading.Event()
        self.lock = threading.Lock()
        self.events = []
        self.metadata = dict(schema='lightbender-video-timing', schema_version=1,
                             camera_id=camera_id, run_id=run_id, request_id=request_id,
                             video_file=self.video.name, command=command, valid=False,
                             clock_assumption='camera and drone epoch clocks synchronized by NTP',
                             first_frame_source='libav encoder input capture timestamp',
                             frame_time_source='MP4 presentation timestamps from ffprobe')
        self.metadata['epoch_clock'] = epoch_clock_status()
        # Truncate stale sidecars before launching a new recording.
        self.frames_path.write_text('')
        self.encoder_log = self.encoder_path.open('w')
        self.save()

    def save(self):
        snapshot = dict(self.metadata, clock_samples=self.samples, events=self.events)
        temporary = self.metadata_path.with_suffix('.json.tmp')
        temporary.write_text(json.dumps(snapshot, indent=2, allow_nan=False)+'\n')
        temporary.replace(self.metadata_path)

    def attach(self, process):
        self.stream = process.stderr
        self.reader = threading.Thread(target=self._read, args=(process.stderr,), daemon=True)
        self.sampler = threading.Thread(target=self._sample, daemon=True)
        self.reader.start(); self.sampler.start()

    def _sample(self):
        while not self.stop.wait(.5):
            with self.lock:
                self.samples.append(clock_sample())

    def _read(self, stream):
        for line in iter(stream.readline, ''):
            self.encoder_log.write(line); self.encoder_log.flush()
            match = FIRST_FRAME.search(line)
            if match and self.first_capture_us is None:
                with self.lock:
                    self.first_capture_us = int(match[1])
                    self.samples.append(clock_sample())
                self.ready.set()

    def event(self, name, **values):
        with self.lock:
            self.events.append(dict(name=name, clock=clock_sample(), **values))
            self.save()

    def wait_first_frame(self, timeout):
        if not self.ready.wait(timeout):
            raise RuntimeError('encoder did not report first-frame capture timestamp')
        with self.lock:
            epoch, domain, uncertainty = capture_epoch(self.first_capture_us, self.samples)
            values = dict(first_frame_epoch_s=epoch, capture_clock=domain,
                          host_clock_mapping_uncertainty_s=uncertainty)
            self.metadata.update(values, first_capture_timestamp_us=self.first_capture_us)
        return values

    def finish(self, returncode):
        self.stop.set()
        if hasattr(self, 'sampler'):
            self.sampler.join(timeout=2)
        if hasattr(self, 'reader'):
            self.reader.join(timeout=3)
            if self.reader.is_alive():
                raise RuntimeError('encoder stderr reader did not stop')
        self.encoder_log.close()
        if hasattr(self, 'reader'):
            self.process_stream_close()
        self.samples.append(clock_sample())
        self.metadata['encoder_returncode'] = returncode
        try:
            if self.first_capture_us is None:
                raise ValueError('missing encoder capture timestamp')
            if returncode not in (0, -2):
                raise ValueError(f'encoder exited with status {returncode}')
            epoch, domain, uncertainty = capture_epoch(self.first_capture_us, self.samples)
            result = subprocess.run(['ffprobe', '-v', 'error', '-select_streams', 'v:0',
                                     '-show_frames', '-show_entries', 'frame=best_effort_timestamp_time',
                                     '-of', 'json', str(self.video)], capture_output=True, text=True,
                                    check=True, timeout=120)
            pts = [float(f['best_effort_timestamp_time']) for f in json.loads(result.stdout)['frames']]
            if (not pts or not all(math.isfinite(p) for p in pts)
                    or any(b <= a for a, b in zip(pts, pts[1:])) or abs(pts[0]) > .001):
                raise ValueError('MP4 timestamps must start at zero and increase in display order')
            with self.frames_path.open('w') as stream:
                for index, value in enumerate(pts):
                    stream.write(json.dumps(dict(frame_index=index, video_time_s=value,
                                                 epoch_time_s=epoch+value))+'\n')
            digest = hashlib.sha256()
            with self.video.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024*1024), b''):
                    digest.update(chunk)
            self.metadata.update(valid=True, first_capture_timestamp_us=self.first_capture_us,
                                 video_start_epoch_s=epoch, capture_clock=domain,
                                 host_clock_mapping_uncertainty_s=uncertainty,
                                 frame_count=len(pts), video_sha256=digest.hexdigest())
        except Exception as error:
            self.metadata.update(valid=False, error=str(error))
            raise
        finally:
            self.save()

    def process_stream_close(self):
        if getattr(self, 'stream', None):
            self.stream.close()
