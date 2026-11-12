"""Live MJPEG observations for the pre-illumination servo calibration barrier."""
import math
import os
import json
import re
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np


def wrap(angle):
    return (angle + 180.) % 360. - 180.


def camera_rotation(position, look_at):
    forward = np.asarray(look_at, float) - position
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, [0., 0., 1.])
    right /= np.linalg.norm(right)
    rotation = np.column_stack((right, np.cross(forward, right), forward))
    if not np.isfinite(rotation).all():
        raise ValueError('invalid calibration camera look-at')
    return rotation


def measure_rods(image, calibration, reference, config, diagnostics=None):
    """Recover directed physical angles by intersecting rays with the rod plane.

    SFL angles are in degrees in the body YZ plane: (0, cos(a), -sin(a)).
    Two colors disambiguate hinge -> tip, including the second servo's 360
    crossing. Perspective and lens distortion are removed before measuring.
    Ambiguous/missing/short/edge-on observations are returned as None.
    """
    if tuple(image.shape[1::-1]) != tuple(calibration.image_size):
        raise ValueError('calibration stream size differs from camera intrinsics')
    position = np.asarray(reference['camera'], float)
    rotation = camera_rotation(position, reference['look_at'])
    target = np.asarray(reference['target'][:3], float)
    yaw = float(reference['yaw'])
    c, s = math.cos(yaw), math.sin(yaw)
    body_y = np.array([-s, c, 0.])
    normal = np.array([c, s, 0.])
    mount = np.asarray(reference.get('light_module_offset', [0., 0., 0.]), float)
    hinge = target + np.array([c*mount[0]-s*mount[1], s*mount[0]+c*mount[1], mount[2]])
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Hue intervals for red, cyan, green, blue; inactive white is excluded.
    hue_ranges = (((0, 10), (170, 179)), ((80, 100),), ((45, 75),), ((105, 135),))
    result = []
    if diagnostics is not None:
        diagnostics['rods'] = []
    for servo, angle in enumerate(reference['angles']):
        detail = dict(servo_index=servo, target_deg=angle, status='missing_color')
        if diagnostics is not None:
            diagnostics['rods'].append(detail)
        direction = body_y*math.cos(math.radians(angle)) - np.array([0., 0., math.sin(math.radians(angle))])
        length = reference['led_count']//2 * config.get('led_pitch_m', .00624)
        world = hinge + np.array([.25, .75])[:, None]*length*direction
        camera_points = (world-position) @ rotation
        if np.any(camera_points[:, 2] <= .01):
            detail['status'] = 'behind_camera'
            result.append(None)
            continue
        expected = cv2.projectPoints(camera_points, np.zeros(3), np.zeros(3),
                                     calibration.camera_matrix, calibration.distortion_coefficients)[0].reshape(2, 2)
        centers = []
        color_components = []
        for half, color_index in enumerate((servo*2, servo*2+1)):
            mask = np.zeros(image.shape[:2], np.uint8)
            for low, high in hue_ranges[color_index]:
                mask |= cv2.inRange(hsv, (low, 100, 55), (high, 255, 255))
            count, labels, stats, centroids = cv2.connectedComponentsWithStats(mask)
            candidates = [i for i in range(1, count)
                          if stats[i, cv2.CC_STAT_AREA] >= 2
                          and np.linalg.norm(centroids[i]-expected[half]) <= config.get('association_radius_px', 80.)]
            if not candidates or sum(stats[i, cv2.CC_STAT_AREA] for i in candidates) < config.get('min_color_pixels', 6):
                detail.update(status='missing_color', color_index=color_index)
                break
            # Bright LED cores clip to white, leaving disconnected colored
            # halos on both sides. Tiny compression/sparkle blobs must not
            # veto those legitimate components or pull the color centroid.
            largest = max(stats[i, cv2.CC_STAT_AREA] for i in candidates)
            candidates = [i for i in candidates if stats[i, cv2.CC_STAT_AREA] >= max(2., .1*largest)]
            areas = stats[candidates, cv2.CC_STAT_AREA]
            center = np.average(centroids[candidates], axis=0, weights=areas)
            centers.append(center)
            color_components.append(centroids[candidates])
        if len(centers) != 2 or np.linalg.norm(centers[1]-centers[0]) < config.get('min_span_px', 8.):
            if len(centers) == 2:
                detail['status'] = 'too_short'
            result.append(None)
            continue
        # Fit the complete directed rod, not either short color half alone:
        # side halos can be wider than one color half, rotating that half's
        # PCA axis by 90 degrees. Keep the ambiguity check relative to the
        # full span, so off-rod distractors still reject the observation.
        axis = centers[1]-centers[0]
        span = np.linalg.norm(axis)
        detail.update(span_px=float(span), centers_px=[c.tolist() for c in centers])
        perpendicular = np.array([-axis[1], axis[0]])/span
        components = np.concatenate(color_components)
        if np.max(np.abs((components-centers[0]) @ perpendicular)) > max(3., .25*span):
            detail['status'] = 'off_rod_color_components'
            result.append(None)
            continue
        if any(np.max(np.abs((group-center) @ (axis/span))) > .5*span+2.
               for group, center in zip(color_components, centers)):
            detail['status'] = 'color_group_too_long'
            result.append(None)
            continue
        rays = cv2.undistortPoints(np.array(centers).reshape(-1, 1, 2),
                                  calibration.camera_matrix, calibration.distortion_coefficients).reshape(2, 2)
        rays = np.column_stack((rays, np.ones(2))) @ rotation.T
        denominator = rays @ normal
        if np.min(np.abs(denominator)) < .15 or denominator[0]*denominator[1] <= 0:
            detail['status'] = 'edge_on'
            result.append(None)
            continue
        vector = rays[1]/denominator[1] - rays[0]/denominator[0]
        # The common plane distance may be negative when normal faces away.
        vector *= np.sign((hinge-position) @ normal)
        measured = math.degrees(math.atan2(-vector[2], vector @ body_y))
        error = wrap(measured-angle)
        detail['status'] = 'detected' if abs(error) <= config.get('max_correction_deg', 15.)+10. else 'angle_out_of_range'
        result.append(float(angle+error) if abs(error) <= config.get('max_correction_deg', 15.)+10. else None)
    return result


def validate_capture(parameters, config):
    from camera_pose import load_camera_calibration
    calibration = load_camera_calibration(config['calibration_file'])
    if calibration.image_size is None or calibration.lens_position is None:
        raise ValueError('servo calibration requires image_size and capture.lens_position')
    lens = next((parameters[i+1] for i in range(len(parameters)-1)
                 if parameters[i] == '--lens-position'), None)
    if lens is None or not math.isclose(float(lens), calibration.lens_position, abs_tol=1e-6):
        raise ValueError('servo calibration lens position differs from recording settings')
    return calibration


def calibration_recording_paths(run_id):
    if not isinstance(run_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]+', run_id):
        raise ValueError('calibration recording requires a valid camera run ID')
    video = Path('logs') / 'servo_calibration' / f'{run_id}.mjpeg'
    return video, video.with_suffix('.frames.jsonl')


def finalize_recording(run_id):
    """Encode after camera release; retain the original stream for recovery."""
    video, _ = calibration_recording_paths(run_id)
    output = video.with_suffix('.mp4')
    temporary = video.with_suffix('.partial.mp4')
    subprocess.run(['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error', '-y',
                    '-f', 'mjpeg', '-framerate', '30', '-i', str(video),
                    '-c:v', 'libx264', '-preset', 'ultrafast', '-crf', '20',
                    '-pix_fmt', 'yuv420p', '-movflags', '+faststart', str(temporary)],
                   check=True, timeout=60, stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    temporary.replace(output)
    return output


class CalibrationStream:
    """Nonblocking camera reader; the camera node retains command/emergency polling."""
    def __init__(self, binary, parameters, config, *, run_id):
        self.config = config
        self.calibration = validate_capture(parameters, config)
        width, height = self.calibration.image_size
        command = [binary, '--nopreview', '-t', '0', '--codec', 'mjpeg',
                   '--width', str(width), '--height', str(height), '-o', '-', *parameters]
        self.video_path, self.timing_path = calibration_recording_paths(run_id)
        self.video_path.parent.mkdir(parents=True, exist_ok=True)
        # Preserve original JPEG bytes, including frames not selected by the
        # detector. Appending also retains footage if capture restarts in a run.
        self.video_file = self.video_path.open('ab')
        try:
            self.timing_file = self.timing_path.open('a')
        except BaseException:
            self.video_file.close()
            raise
        self.buffer = b''
        self.buffer_offset = self.video_file.tell()
        self.frame_index = 0
        self.job = None
        self.last_request = time.monotonic()
        self.stream_started = self.last_request
        try:
            self.process = subprocess.Popen(command, stdout=subprocess.PIPE,
                                            stderr=subprocess.DEVNULL, start_new_session=True)
            os.set_blocking(self.process.stdout.fileno(), False)
        except BaseException:
            if hasattr(self, 'process'):
                self.process.kill()
                self.process.wait(timeout=2)
                self.process.stdout.close()
            self.video_file.close()
            self.timing_file.close()
            raise

    def _read_frames(self):
        for _ in range(32):
            try:
                data = os.read(self.process.stdout.fileno(), 65536)
            except BlockingIOError:
                break
            if not data:
                break
            self.video_file.write(data)
            self.buffer += data
        frames = []
        received = time.monotonic()
        while True:
            start = self.buffer.find(b'\xff\xd8')
            end = self.buffer.find(b'\xff\xd9', max(0, start))
            if start < 0 or end < 0:
                break
            frames.append(self.buffer[start:end+2])
            self.timing_file.write(json.dumps(dict(
                frame_index=self.frame_index, byte_offset=self.buffer_offset+start,
                byte_length=end+2-start, received_monotonic_s=received,
                stream_started_monotonic_s=self.stream_started,
                request_id=self.job.get('request_id') if self.job else None))+'\n')
            self.frame_index += 1
            self.buffer = self.buffer[end+2:]
            self.buffer_offset += end+2
        if len(self.buffer) > 8*1024*1024:
            raise RuntimeError('invalid MJPEG calibration stream')
        return frames

    def request(self, message):
        if self.job and self.job['request_id'] == message['request_id']:
            return  # retries do not reset the observation deadline
        # Archive pre-request frames, but do not use them for the new estimate.
        self._read_frames()
        self.job = dict(message, samples=[[], []], not_before=time.monotonic()+.25)
        self.last_request = time.monotonic()
        self.deadline = self.last_request + self.config.get('measurement_timeout_s', 5.)

    def poll(self):
        frames = self._read_frames()
        if self.process.poll() is not None:
            raise RuntimeError('servo calibration camera stream exited')
        if time.monotonic()-self.last_request > self.config.get('timeout_s', 60.)+10.:
            raise RuntimeError('servo calibration camera idle timeout')
        job = self.job
        if job is None:
            return None
        if frames and time.monotonic() >= job['not_before']:
            # Use only the newest frame; buffered older frames cannot masquerade
            # as temporal consensus after a servo movement.
            frame = cv2.imdecode(np.frombuffer(frames[-1], np.uint8), cv2.IMREAD_COLOR)
            if frame is not None:
                job['diagnostics'] = {}
                angles = measure_rods(frame, self.calibration, job['reference'], self.config,
                                      diagnostics=job['diagnostics'])
                for samples, angle in zip(job['samples'], angles):
                    if angle is None:
                        samples.clear()
                    elif samples and abs(angle-np.median(samples)) > self.config.get('max_spread_deg', 1.5):
                        samples[:] = [angle]
                    else:
                        samples.append(angle)
        required = self.config.get('consensus_frames', 3)
        complete = all(len(samples) >= required for samples in job['samples'])
        if complete or time.monotonic() >= self.deadline:
            self.job = None
            return dict(status='SERVO_MEASURED', request_id=job['request_id'],
                        angles=[float(np.median(v)) if len(v) >= required else None for v in job['samples']],
                        diagnostics=dict(job.get('diagnostics', {}), consensus_counts=[len(v) for v in job['samples']]))
        return None

    def close(self):
        try:
            if self.process.poll() is None:
                self.process.terminate()
                try:
                    self.process.wait(timeout=2)
                except subprocess.TimeoutExpired:
                    self.process.kill()
                    self.process.wait(timeout=2)
            # Keep bytes queued at stop, including the final partial JPEG if
            # capture was interrupted. Earlier complete frames remain playable.
            self._read_frames()
        finally:
            try:
                self.process.stdout.close()
            finally:
                try:
                    self.video_file.close()
                finally:
                    self.timing_file.close()
