import sys

import yaml
import zmq
import subprocess
import signal
import os
import json
import hashlib
import math
import shutil
import tempfile
import time
from dataclasses import fields
from log import LoggerFactory


MANIFEST_FILE = 'swarm_manifest.yaml'
OUTPUT_FILENAME = 'mission_footage.mp4'
POSE_OUTPUT_FILENAME = 'camera_pose_estimate.json'
POSE_OVERLAY_FILENAME = 'camera_pose_overlay.jpeg'
POSE_FAILED_FRAME_FILENAME = 'camera_pose_failed.jpeg'


class CaptureError(RuntimeError):
    """A transient failure while acquiring an RGB pose frame."""


def load_manifest():
    with open(MANIFEST_FILE, 'r') as f:
        return yaml.safe_load(f)


class CameraNode:
    def __init__(self, manifest, args):
        self.manifest = manifest
        self.args = args
        self.ctrl = manifest['controller']
        self.context = zmq.Context()
        self.recording_process = None
        self.recording_binary = None
        self.logger = LoggerFactory("Camera").get_logger()
        self.pose_config = manifest.get('camera_node', {}).get(
            'pose_estimation', {}
        )
        self.run_id = manifest.get('camera_node', {}).get('run_id')

        # Subscribe to Commands (START, STOP_CAMERA, EMERGENCY)
        self.sub_socket = self.context.socket(zmq.SUB)
        self.sub_socket.connect(f"tcp://{self.ctrl['ip']}:{self.ctrl['zmq_cmd_port']}")
        self.sub_socket.setsockopt_string(zmq.SUBSCRIBE, "")

        # Push Status (READY)
        self.push_socket = self.context.socket(zmq.PUSH)
        self.push_socket.connect(f"tcp://{self.ctrl['ip']}:{self.ctrl['zmq_ack_port']}")

    @staticmethod
    def _typed_config(config_type, value, name):
        """Build one solver config while rejecting misspelled manifest keys."""
        if value is None:
            return config_type()
        if not isinstance(value, dict):
            raise ValueError(f"{name} must be a mapping")
        allowed = {item.name for item in fields(config_type)}
        unknown = sorted(set(value) - allowed)
        if unknown:
            raise ValueError(
                f"unknown {name} option(s): {', '.join(unknown)}"
            )
        return config_type(**value)

    @staticmethod
    def _parameter_value(parameters, option):
        """Return the last command-line value following *option*, if present."""
        result = None
        for index, value in enumerate(parameters[:-1]):
            if value == option:
                result = parameters[index + 1]
        return result

    @staticmethod
    def _sha256_file(path):
        digest = hashlib.sha256()
        with open(path, 'rb') as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b''):
                digest.update(block)
        return digest.hexdigest()

    def _status_message(self, status, **values):
        message = {'id': 'CAM', 'status': status}
        run_id = getattr(self, 'run_id', None)
        if run_id is not None:
            message['run_id'] = run_id
        message.update(values)
        return message

    def _capture_pose_image(self, calibration, timeout_s=None):
        """Capture a settled frame through the same pipeline used for video."""
        import cv2

        configured_image = self.pose_config.get('image_file')
        if configured_image:
            if not self.pose_config.get('offline_test_mode', False):
                raise ValueError(
                    "pose_estimation.image_file is allowed only with "
                    "offline_test_mode: true"
                )
            image_path = os.fspath(configured_image)
            image = cv2.imread(image_path, cv2.IMREAD_COLOR)
            if image is None:
                raise RuntimeError(f"could not read pose image {image_path}")
            return image

        capture_binary = (
            self.pose_config.get('capture_binary')
            or getattr(self, 'recording_binary', None)
            or shutil.which('rpicam-vid')
            or shutil.which('libcamera-vid')
        )
        if not capture_binary:
            raise RuntimeError(
                "neither rpicam-vid nor libcamera-vid is installed"
            )

        if calibration.image_size is not None:
            width, height = calibration.image_size
        else:
            width = int(self.pose_config.get('width', 1920))
            height = int(self.pose_config.get('height', 1080))
        if width <= 0 or height <= 0:
            raise ValueError("pose capture width and height must be positive")

        # Motion-JPEG uses the video configuration rather than switching to a
        # still configuration with a potentially different sensor crop.  A
        # one-millisecond segment boundary emits one JPEG per frame; use the
        # final frame after the configured settling interval.
        with tempfile.TemporaryDirectory(prefix='camera-pose-') as directory:
            image_pattern = os.path.join(directory, 'frame-%06d.jpeg')
            command = [
                capture_binary,
                '--nopreview',
                '--timeout', str(int(self.pose_config.get('warmup_ms', 2500))),
                '--width', str(width),
                '--height', str(height),
                '--codec', 'mjpeg',
                '--segment', '1',
                '-o', image_pattern,
            ]
            command.extend(self.args['params'])
            self.logger.info(f"Capturing HyperGrid pose image: {command}")
            capture_timeout = float(
                self.pose_config.get('capture_timeout_s', 10.0)
            )
            if not math.isfinite(capture_timeout) or capture_timeout <= 0:
                raise ValueError("capture_timeout_s must be positive and finite")
            if timeout_s is not None:
                capture_timeout = min(capture_timeout, max(0.001, timeout_s))
            try:
                result = subprocess.run(
                    command,
                    check=False,
                    timeout=capture_timeout,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.PIPE,
                    text=True,
                )
            except subprocess.TimeoutExpired as error:
                raise CaptureError(
                    f"pose image capture timed out after {capture_timeout:.3f}s"
                ) from error
            if result.returncode != 0:
                detail = (result.stderr or '').strip()
                raise CaptureError(
                    f"pose image capture exited {result.returncode}: {detail}"
                )
            frame_paths = sorted(
                os.path.join(directory, name)
                for name in os.listdir(directory)
                if name.startswith('frame-') and name.endswith('.jpeg')
            )
            if not frame_paths:
                raise CaptureError("pose video capture produced no JPEG frames")
            image = cv2.imread(frame_paths[-1], cv2.IMREAD_COLOR)
            if image is None:
                raise CaptureError(
                    "pose video capture produced no readable final frame"
                )
            return image

    def _load_pose_prior(self, config):
        """Use a previous accepted pose only with matching grid and optics."""
        if not os.path.isfile(POSE_OUTPUT_FILENAME):
            return None
        try:
            with open(POSE_OUTPUT_FILENAME) as stream:
                prior = json.load(stream)
            provenance = prior['provenance']
            if (
                provenance['grid_sha256']
                != self._sha256_file(config['grid_file'])
                or provenance['calibration_sha256']
                != self._sha256_file(config['calibration_file'])
            ):
                return None
            return prior
        except (OSError, ValueError, KeyError, TypeError):
            return None

    def estimate_pose(self):
        """Estimate the external RGB camera pose from the always-on lattice."""
        from camera_pose import (
            MarkerDetectionConfig,
            PoseEstimationError,
            PoseSolverConfig,
            estimate_camera_pose,
            load_camera_calibration,
            load_hypergrid,
            render_camera_pose_overlay,
        )

        config = self.pose_config
        calibration = load_camera_calibration(config['calibration_file'])
        if calibration.image_size is None:
            raise ValueError(
                "external RGB camera calibration must include image_size"
            )
        if calibration.lens_position is None:
            raise ValueError(
                "external RGB camera calibration must include "
                "capture.lens_position"
            )
        configured_lens_position = self._parameter_value(
            self.args['params'], '--lens-position'
        )
        if configured_lens_position is None:
            raise ValueError(
                "camera node must use a fixed --lens-position for pose estimation"
            )
        try:
            configured_lens_position = float(configured_lens_position)
        except (TypeError, ValueError) as error:
            raise ValueError("--lens-position must be numeric") from error
        if not math.isclose(
            configured_lens_position,
            calibration.lens_position,
            rel_tol=0.0,
            abs_tol=1e-6,
        ):
            raise ValueError(
                "camera --lens-position does not match calibration: "
                f"{configured_lens_position} != {calibration.lens_position}"
            )
        detector_config = self._typed_config(
            MarkerDetectionConfig, config.get('detector'),
            'camera_node.pose_estimation.detector',
        )
        solver_config = self._typed_config(
            PoseSolverConfig, config.get('solver'),
            'camera_node.pose_estimation.solver',
        )
        prior_camera_pose = self._load_pose_prior(config)

        # A newly opened camera can need more than one frame for exposure and
        # scaler state to settle. Retry transient capture/pattern failures;
        # bad calibration or malformed configuration remains fail-fast.
        orchestrator_timeout = float(config.get('timeout_s', 20.0))
        if not math.isfinite(orchestrator_timeout) or orchestrator_timeout <= 0:
            raise ValueError("timeout_s must be positive and finite")
        # Leave the orchestrator time to receive and validate the result, but
        # allow four normal captures within the default 20-second preflight.
        # The first capture is commonly rejected while exposure settles.
        default_estimation_timeout = 0.9 * orchestrator_timeout
        estimation_timeout = float(config.get(
            'estimation_timeout_s', default_estimation_timeout
        ))
        if (
            not math.isfinite(estimation_timeout)
            or not 0 < estimation_timeout < orchestrator_timeout
        ):
            raise ValueError(
                "estimation_timeout_s must be positive and less than timeout_s"
            )

        consensus_frames = config.get('consensus_frames', 3)
        if (
            isinstance(consensus_frames, bool)
            or not isinstance(consensus_frames, int)
            or consensus_frames < 1
        ):
            raise ValueError("consensus_frames must be a positive integer")
        if config.get('image_file') and consensus_frames != 1:
            raise ValueError(
                "offline image_file estimation requires consensus_frames: 1"
            )
        max_consensus_position_spread = float(config.get(
            'max_consensus_position_spread_m', 0.025
        ))
        max_consensus_yaw_spread = float(config.get(
            'max_consensus_yaw_spread_rad', 0.03
        ))
        if (
            not math.isfinite(max_consensus_position_spread)
            or max_consensus_position_spread <= 0
            or not math.isfinite(max_consensus_yaw_spread)
            or max_consensus_yaw_spread <= 0
        ):
            raise ValueError("consensus spread limits must be positive and finite")

        deadline = time.monotonic() + estimation_timeout
        attempt = 0
        consensus = []
        images_by_estimate = {}
        last_retry_error = None
        while len(consensus) < consensus_frames:
            remaining = deadline - time.monotonic()
            warmup_s = float(config.get('warmup_ms', 2500)) / 1000.0
            startup_margin_s = float(config.get(
                'capture_startup_margin_s', 1.0
            ))
            minimum_capture_budget = warmup_s + startup_margin_s
            if remaining <= minimum_capture_budget:
                detail = (
                    f"; last rejection: {last_retry_error}"
                    if last_retry_error is not None else ""
                )
                raise PoseEstimationError(
                    "camera pose did not reach temporal consensus before "
                    f"timeout{detail}"
                ) from last_retry_error
            attempt += 1
            image = None
            try:
                image = self._capture_pose_image(
                    calibration,
                    timeout_s=remaining,
                )
                image_size = (int(image.shape[1]), int(image.shape[0]))
                if (
                    calibration.image_size is not None
                    and calibration.image_size != image_size
                ):
                    raise ValueError(
                        "pose image size "
                        f"{image_size} does not match calibration "
                        f"{calibration.image_size}"
                    )
                estimate = estimate_camera_pose(
                    image,
                    config['grid_file'],
                    calibration.camera_matrix,
                    calibration.distortion_coefficients,
                    config['rough_position_xyz'],
                    config['desired_position_xyz'],
                    rough_camera_yaw=config.get('rough_yaw_rad'),
                    desired_camera_yaw=config.get('desired_yaw_rad'),
                    look_at=config.get('look_at_xyz'),
                    detector_config=detector_config,
                    solver_config=solver_config,
                    prior_camera_pose=prior_camera_pose,
                )
            except (CaptureError, PoseEstimationError) as error:
                if image is not None:
                    import cv2
                    if cv2.imwrite(POSE_FAILED_FRAME_FILENAME, image):
                        self.logger.warning(
                            'Saved rejected RGB pose frame: %s',
                            POSE_FAILED_FRAME_FILENAME,
                        )
                if config.get('image_file') or time.monotonic() >= deadline:
                    raise
                last_retry_error = error
                self.logger.warning(
                    "RGB camera pose attempt %d rejected: %s", attempt, error
                )
                delay = max(0.0, float(config.get('retry_delay_s', 0.2)))
                time.sleep(min(delay, max(0.0, deadline - time.monotonic())))
                continue

            candidate_consensus = consensus + [estimate]
            images_by_estimate[id(estimate)] = image
            position_spread = max(
                (
                    math.dist(first.position_xyz, second.position_xyz)
                    for index, first in enumerate(candidate_consensus)
                    for second in candidate_consensus[index + 1:]
                ),
                default=0.0,
            )
            yaw_spread = max(
                (
                    abs(math.atan2(
                        math.sin(first.yaw_rad - second.yaw_rad),
                        math.cos(first.yaw_rad - second.yaw_rad),
                    ))
                    for index, first in enumerate(candidate_consensus)
                    for second in candidate_consensus[index + 1:]
                ),
                default=0.0,
            )
            if (
                position_spread > max_consensus_position_spread
                or yaw_spread > max_consensus_yaw_spread
            ):
                self.logger.warning(
                    "RGB camera pose consensus reset at attempt %d: "
                    "position spread %.4fm, yaw spread %.4frad",
                    attempt,
                    position_spread,
                    yaw_spread,
                )
                consensus = [estimate]
            else:
                consensus = candidate_consensus

        estimate = min(
            consensus,
            key=lambda item: item.quality.reprojection_rms_px,
        )
        position_spread = max(
            (
                math.dist(first.position_xyz, second.position_xyz)
                for index, first in enumerate(consensus)
                for second in consensus[index + 1:]
            ),
            default=0.0,
        )
        yaw_spread = max(
            (
                abs(math.atan2(
                    math.sin(first.yaw_rad - second.yaw_rad),
                    math.cos(first.yaw_rad - second.yaw_rad),
                ))
                for index, first in enumerate(consensus)
                for second in consensus[index + 1:]
            ),
            default=0.0,
        )

        payload = estimate.to_dict()
        payload.update({
            'protocol_version': 1,
            'id': 'CAM',
            'status': 'CAMERA_POSE',
        })
        run_id = getattr(self, 'run_id', None)
        if run_id is not None:
            payload['run_id'] = run_id
        payload['quality']['attempt'] = attempt
        payload['quality']['consensus_frames'] = len(consensus)
        payload['quality']['consensus_position_spread_m'] = position_spread
        payload['quality']['consensus_yaw_spread_rad'] = yaw_spread
        payload['provenance'] = {
            'grid_file': os.fspath(config['grid_file']),
            'grid_sha256': self._sha256_file(config['grid_file']),
            'calibration_file': os.fspath(config['calibration_file']),
            'calibration_sha256': self._sha256_file(
                config['calibration_file']
            ),
            'image_size': list(calibration.image_size),
            'lens_position': calibration.lens_position,
            'camera_model': calibration.camera_model,
        }
        import cv2
        overlay = render_camera_pose_overlay(
            images_by_estimate[id(estimate)], estimate,
            load_hypergrid(config['grid_file']),
            calibration.camera_matrix, calibration.distortion_coefficients,
        )
        if not cv2.imwrite(POSE_OVERLAY_FILENAME, overlay):
            raise RuntimeError(
                f"could not save camera pose overlay {POSE_OVERLAY_FILENAME}"
            )
        payload['overlay_file'] = POSE_OVERLAY_FILENAME
        with open(POSE_OUTPUT_FILENAME, 'w') as stream:
            json.dump(payload, stream, indent=2)
        return payload

    def start_recording(self, request_id=None, acknowledge=True):
        if self.recording_process is None:
            # -t 0: Record indefinitely until signal.
            # --inline: Improves compatibility for streaming/concatenation
            cmd = [
                self.recording_binary,
                "-t", "0",
                "-o", OUTPUT_FILENAME,
                "--width", "1920",
                "--height", "1080",
                "--nopreview"
            ]
            cmd += self.args["params"]
            self.logger.info(f"Starting Recording with these settings: {cmd}")

            try:
                # Start process in new process group for clean termination
                self.recording_process = subprocess.Popen(
                    cmd,
                    preexec_fn=os.setsid
                )
                time.sleep(0.25)
                return_code = self.recording_process.poll()
                if return_code is not None:
                    self.recording_process = None
                    raise RuntimeError(
                        f"video process exited immediately with {return_code}"
                    )
                if acknowledge:
                    values = {}
                    if request_id is not None:
                        values['request_id'] = request_id
                    self.push_socket.send_json(
                        self._status_message('RECORDING', **values)
                    )
                return True
            except Exception as error:
                self.recording_process = None
                self.logger.exception(f"Could not start recording: {error}")
                values = {'error': str(error)}
                if request_id is not None:
                    values['request_id'] = request_id
                self.push_socket.send_json(self._status_message(
                    'CAMERA_RECORDING_FAILED', **values
                ))
                return False
        else:
            self.logger.info("Already recording.")
            if acknowledge:
                values = {}
                if request_id is not None:
                    values['request_id'] = request_id
                self.push_socket.send_json(
                    self._status_message('RECORDING', **values)
                )
            return True

    def stop_recording(self):
        if self.recording_process:
            self.logger.info("Stopping Recording and Saving...")
            # Send SIGINT (Ctrl+C) to the process group to close file gracefully
            os.killpg(os.getpgid(self.recording_process.pid), signal.SIGINT)
            try:
                self.recording_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.recording_process.kill()

            self.recording_process = None
            self.logger.info(f"[Camera] Saved to {OUTPUT_FILENAME}")
        else:
            self.logger.error("[Camera] No active recording to stop.")

    def run(self):
        self.logger.info("Node Online. Waiting for commands.")

        configured_video_binary = self.manifest.get('camera_node', {}).get(
            'video_binary'
        )
        if configured_video_binary:
            self.recording_binary = shutil.which(configured_video_binary)
        else:
            self.recording_binary = (
                shutil.which('rpicam-vid')
                or shutil.which('libcamera-vid')
            )
        if not self.recording_binary:
            error = "neither rpicam-vid nor libcamera-vid is installed"
            self.logger.error(error)
            self.push_socket.send_json(
                self._status_message('FAILED', error=error)
            )
            return

        if self.pose_config.get('enabled', False):
            try:
                pose = self.estimate_pose()
                self.logger.info(
                    "RGB camera pose xyz=%s yaw=%.6f rad, offset=%s "
                    "yaw_offset=%.6f rad, quality=%s",
                    pose['position_xyz'], pose['yaw_rad'],
                    pose['position_offset_xyz'], pose['yaw_offset_rad'],
                    pose['quality'],
                )
                self.push_socket.send_json(pose)
            except Exception as error:
                self.logger.exception(f"RGB camera pose estimation failed: {error}")
                self.push_socket.send_json(self._status_message(
                    'CAMERA_POSE_FAILED',
                    protocol_version=1,
                    error=str(error),
                ))
                return

        # Notify Laptop only after optional pose estimation succeeds.
        self.push_socket.send_json(self._status_message('READY'))

        try:
            while True:
                if self.recording_process is not None:
                    return_code = self.recording_process.poll()
                    if return_code is not None:
                        self.recording_process = None
                        error = (
                            "video process exited unexpectedly with "
                            f"{return_code}"
                        )
                        self.logger.error(error)
                        self.push_socket.send_json(self._status_message(
                            'CAMERA_RECORDING_FAILED', error=error
                        ))
                        return
                if self.sub_socket.poll(250) == 0:
                    continue
                msg = self.sub_socket.recv_json()
                cmd = msg.get('cmd')

                if cmd == 'START_CAMERA':
                    if (
                        self.run_id is not None
                        and msg.get('run_id') != self.run_id
                    ):
                        continue
                    if not self.start_recording(
                        request_id=msg.get('request_id')
                    ):
                        break

                elif cmd == 'START':
                    # Compatibility command. Explicit START_CAMERA performs
                    # the nonce-bound health acknowledgement; do not emit a
                    # duplicate RECORDING status here.
                    if not self.start_recording(acknowledge=False):
                        break

                elif cmd == 'STOP_CAMERA':
                    self.stop_recording()
                    break  # Exit loop to allow script to finish (and download to happen)

                elif cmd == 'EMERGENCY':
                    self.logger.warning("Emergency received. Stopping recording.")
                    self.stop_recording()
                    break

                elif cmd == 'SHUTDOWN':
                    self.stop_recording()
                    break

        except KeyboardInterrupt:
            self.stop_recording()
        except Exception as e:
            self.logger.error(f"Error: {e}")
            self.stop_recording()


if __name__ == "__main__":
    args = {"params": sys.argv[1:]}
    manifest = load_manifest()
    node = CameraNode(manifest, args)
    node.run()
