"""Calibration protocol/vision checks without Raspberry Pi hardware."""
import ast
import json
import math
import threading
import tempfile
import unittest
from types import SimpleNamespace
from pathlib import Path
from unittest.mock import Mock, patch

import cv2
import numpy as np

from servo_calibration import calibration_colors, handle_command
from servo_calibration_camera import measure_rods, camera_rotation, CalibrationStream
from camera_node import CameraNode


def scene(angles, yaw=0., camera=(2., 0., 1.), look_at=(0., 0., 1.), discrete=True):
    calibration = SimpleNamespace(image_size=(960, 720),
        camera_matrix=np.array([[1400., 0., 480.], [0., 1400., 360.], [0., 0., 1.]]),
        distortion_coefficients=np.array([.12, -.04, .001, -.002, 0.]))
    reference = dict(camera=camera, look_at=look_at, target=[0., 0., 1., yaw],
                     yaw=yaw, angles=angles, led_count=50)
    rotation = camera_rotation(np.array(camera), look_at)
    image = np.zeros((720, 960, 3), np.uint8)
    colors = calibration_colors(50, True)
    for servo, angle in enumerate(angles):
        direction = np.array([-math.sin(yaw)*math.cos(math.radians(angle)),
                              math.cos(yaw)*math.cos(math.radians(angle)), -math.sin(math.radians(angle))])
        indices = list(range(24, -1, -1)) if servo == 0 else list(range(25, 50))
        for step, index in enumerate(indices):
            point = np.array([0., 0., 1.])+(step+.5)*.00624*direction
            camera_point = (point-np.array(camera)) @ rotation
            pixel = cv2.projectPoints(camera_point.reshape(1, 3), np.zeros(3), np.zeros(3),
                                      calibration.camera_matrix, calibration.distortion_coefficients)[0].reshape(2)
            cv2.circle(image, tuple(np.rint(pixel).astype(int)), 1 if discrete else 3,
                       tuple(reversed(colors[index])), -1)
    return image, calibration, reference


class VisionTests(unittest.TestCase):
    def test_recorded_white_cores_and_split_halos_are_detected(self):
        from camera_pose import load_camera_calibration
        root = Path(__file__).with_name('testdata')/'servo_calibration'
        fixture = json.loads((root/'bloom_reference.json').read_text())
        calibration = load_camera_calibration(root/'bloom_reference.json')
        for image in (root/f'bloom_{index}.jpg' for index in (10, 100, 600)):
            with self.subTest(image=image.name):
                diagnostics = {}
                measured = measure_rods(cv2.imread(str(image)), calibration, fixture['reference'], {}, diagnostics)
                self.assertTrue(all(value is not None for value in measured), diagnostics)
                np.testing.assert_allclose(measured, [90., 270.], atol=3.)
                self.assertEqual([rod['status'] for rod in diagnostics['rods']], ['detected', 'detected'])

    def test_directed_angles_perspective_distortion_and_disconnected_leds(self):
        for angles in ([8., 188.], [98., 278.], [180., 360.], [255., 435.]):
            for yaw in (0., .4):
                with self.subTest(angles=angles, yaw=yaw):
                    image, calibration, reference = scene(angles, yaw, camera=(2., -.6, 1.3))
                    measured = measure_rods(image, calibration, reference, {})
                    self.assertTrue(all(v is not None for v in measured), measured)
                    np.testing.assert_allclose(measured, angles, atol=1.1)

    def test_blank_short_and_ambiguous_images_are_rejected(self):
        image, calibration, reference = scene([90., 270.])
        self.assertEqual(measure_rods(image*0, calibration, reference, {}), [None, None])
        # A colored distractor close to the rod, off its line, is ambiguous.
        cv2.circle(image, (500, 325), 4, (0, 0, 180), -1)
        self.assertIsNone(measure_rods(image, calibration, reference, {})[0])
        self.assertEqual(measure_rods(image, calibration, reference, dict(min_span_px=1000)), [None, None])

    def test_image_size_is_checked(self):
        image, calibration, reference = scene([90., 270.])
        with self.assertRaises(ValueError):
            measure_rods(image[:100], calibration, reference, {})

    def test_camera_size_and_focus_preflight(self):
        from servo_calibration_camera import validate_capture
        calibration = SimpleNamespace(image_size=(960, 720), lens_position=.5)
        with patch('camera_pose.load_camera_calibration', return_value=calibration):
            with self.assertRaises(ValueError):
                validate_capture(['--lens-position', '1.'], {'calibration_file': 'test'})
            self.assertIs(validate_capture(['--lens-position', '.5'], {'calibration_file': 'test'}), calibration)


class ProtocolTests(unittest.TestCase):
    def controller(self):
        return SimpleNamespace(args=SimpleNamespace(drone_id='lb1', tag='run', servo=True,
            led=True, servo_count=2, servo_type='V', led_count=50, servo_offsets=[0., 0.]),
            manifest={'servo_calibration': {'enabled': True}},
            missions=[{'drones': {'lb1': {'servos': [[180., 360.], [265., 445.]]}}}],
            _servo_calibration_phase=0, push_socket=Mock(), servo=Mock(), led=Mock(),
            smooth_controller=SimpleNamespace(lock=threading.RLock(), groups={'servos': {}}))

    def command(self, **kwargs):
        return dict(cmd='SERVO_CALIBRATE', tag='run', drone_id='lb1', mission_index=0,
                    request_id='request', angles=[180., 360.], offsets=[2., -3.], selected=True, **kwargs)

    def test_offsets_apply_once_to_hardware_and_future_targets(self):
        controller = self.controller()
        command = self.command()
        handle_command(controller, command)
        handle_command(controller, command)  # retry is absolute/idempotent
        controller.servo.set_all.assert_called_with([182., 357.])
        self.assertEqual(controller.smooth_controller.groups['servos']['offsets'], [2., -3.])
        self.assertEqual(controller.args.servo_offsets, [2., -3.])
        self.assertEqual(controller.push_socket.send_json.call_args.args[0]['status'], 'SERVO_APPLIED')
        self.assertEqual(set(controller.led.set_colors.call_args.args[0]),
                         {(180, 0, 0), (0, 180, 180), (0, 180, 0), (0, 0, 180)})

    def test_limits_include_future_sfl_samples_and_no_partial_mutation(self):
        controller = self.controller()
        command = self.command()
        command['offsets'] = [6., 0.]
        handle_command(controller, command)
        self.assertEqual(controller.push_socket.send_json.call_args.args[0]['status'], 'SERVO_APPLY_FAILED')
        controller.servo.set_all.assert_not_called()

    def test_wrong_run_target_phase_and_invalid_values(self):
        for field, value in [('tag', 'old'), ('drone_id', 'lb2'), ('mission_index', 1),
                             ('offsets', [float('nan'), 0.])]:
            controller = self.controller()
            command = self.command()
            command[field] = value
            handle_command(controller, command)
            controller.servo.set_all.assert_not_called()
        controller = self.controller()
        controller._servo_calibration_phase = None
        handle_command(controller, self.command())
        controller.servo.set_all.assert_not_called()

    def test_hardware_failure_is_reported_and_finish_clears_leds(self):
        controller = self.controller()
        controller.servo.set_all.side_effect = RuntimeError('PWM failed')
        handle_command(controller, self.command())
        self.assertEqual(controller.push_socket.send_json.call_args.args[0]['status'], 'SERVO_APPLY_FAILED')
        controller.servo.set_all.side_effect = None
        handle_command(controller, self.command(finish=True))
        controller.led.set_colors.assert_called_with([(0, 0, 0)]*50)

    def test_camera_request_nonce_retry_and_release(self):
        node = CameraNode.__new__(CameraNode)
        node.run_id = 'camera-run'
        node.manifest = {'servo_calibration': {'enabled': True}, 'camera_node': {'id': 'FRONT'}}
        node.recording_process = None
        node.push_socket = Mock()
        node._servo_camera = Mock()
        command = dict(cmd='MEASURE_SERVOS', run_id='old', request_id='request')
        node._servo_calibration_command(command)
        node._servo_camera.request.assert_not_called()
        command['run_id'] = 'camera-run'
        node._servo_calibration_command(command)
        node._servo_camera.poll.return_value = dict(status='SERVO_MEASURED', request_id='request', angles=[180., 360.])
        node._poll_servo_camera()
        self.assertEqual(node.push_socket.send_json.call_args.args[0]['id'], 'FRONT')
        node._servo_calibration_command(command)
        self.assertEqual(node._servo_camera.request.call_count, 1)
        stream = node._servo_camera
        node._servo_calibration_command(dict(command, cmd='STOP_SERVO_CAMERA'))
        stream.close.assert_called_once()
        self.assertIsNone(node._servo_camera)

    def test_stream_missing_frames_times_out_and_retry_does_not_reset(self):
        stream = CalibrationStream.__new__(CalibrationStream)
        stream.process = Mock()
        stream.process.poll.return_value = None
        stream.config = dict(measurement_timeout_s=1.)
        stream.job = None
        stream.buffer = b''
        with patch('servo_calibration_camera.os.read', side_effect=BlockingIOError), patch(
                'servo_calibration_camera.time.monotonic', return_value=10.):
            stream.request(dict(request_id='a'))
            self.assertEqual(stream.deadline, 11.)
        with patch('servo_calibration_camera.os.read', side_effect=BlockingIOError), patch(
                'servo_calibration_camera.time.monotonic', return_value=12.):
            stream.request(dict(request_id='a'))
            self.assertEqual(stream.deadline, 11.)
            self.assertEqual(stream.poll()['angles'], [None, None])

    def test_calibration_camera_ignores_takeoff_start_until_explicit_recording(self):
        node = CameraNode.__new__(CameraNode)
        node.run_id = 'camera-run'
        node.manifest = {'camera_node': {'video_binary': 'true'},
                         'servo_calibration': {'enabled': True}}
        node.args = {'params': []}
        node.pose_config = {}
        node.recording_process = None
        node.logger = Mock()
        node.push_socket = Mock()
        node.sub_socket = Mock()
        node.sub_socket.poll.return_value = True
        node.sub_socket.recv_json.side_effect = [dict(cmd='START'),
            dict(cmd='START_CAMERA', run_id='camera-run', request_id='record'), dict(cmd='SHUTDOWN')]
        node.start_recording = Mock(return_value=True)
        node.stop_recording = Mock()
        with patch('camera_node.shutil.which', return_value='/binary'), patch(
                'servo_calibration_camera.validate_capture'):
            node.run()
        node.start_recording.assert_called_once_with(request_id='record')

    def test_drone_watchdog_starts_with_calibration_and_expires_into_landing(self):
        tree = ast.parse(Path(__file__).with_name('controller.py').read_text())
        controller_class = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == 'Controller')
        handshake = next(node for node in controller_class.body if isinstance(node, ast.FunctionDef) and node.name == 'handshake')
        clock = SimpleNamespace(monotonic=Mock(return_value=2.), sleep=Mock())
        namespace = dict(time=clock, logger=Mock())
        exec(compile(ast.Module(body=[handshake], type_ignores=[]), 'controller.py', 'exec'), namespace)
        controller = self.controller()
        controller.args.orchestrated = True
        controller.voltage = 4.
        controller.manifest['mission'] = dict(delta_t=0.)
        controller._prepare_for_emergency_landing = Mock()
        # Waiting for operator confirmation alone never starts the watchdog.
        controller._safe_sleep = Mock(side_effect=[False, False, True])
        namespace['handshake'](controller, calibration_mission_index=0)
        clock.monotonic.assert_not_called()

        def start_calibration(_duration):
            controller._servo_calibration_deadline = 1.
            return False

        controller._safe_sleep = start_calibration
        with self.assertRaisesRegex(RuntimeError, 'handshake timed out'):
            namespace['handshake'](controller, calibration_mission_index=0)
        controller._prepare_for_emergency_landing.assert_called_once()
        self.assertIsNone(controller._servo_calibration_phase)


class CalibrationRecordingTests(unittest.TestCase):
    def create_stream(self, directory):
        paths = Path(directory)/'run.mjpeg', Path(directory)/'run.frames.jsonl'
        process = Mock()
        process.poll.return_value = None
        with patch('servo_calibration_camera.validate_capture', return_value=SimpleNamespace(image_size=(32, 32))), patch(
                'servo_calibration_camera.calibration_recording_paths', return_value=paths), patch(
                'servo_calibration_camera.subprocess.Popen', return_value=process), patch(
                'servo_calibration_camera.os.set_blocking'):
            stream = CalibrationStream('/camera', [], {}, run_id='run')
        return stream, paths

    @staticmethod
    def jpeg(value):
        return cv2.imencode('.jpg', np.full((32, 32, 3), value, np.uint8))[1].tobytes()

    def test_saves_exact_frames_before_during_and_after_measurement_and_at_stop(self):
        frames = [self.jpeg(value) for value in (10, 50, 100, 150, 200)]
        with tempfile.TemporaryDirectory() as directory:
            stream, (video, timing) = self.create_stream(directory)
            # Preserve a JPEG split across reads, including the pre-request drain.
            with patch('servo_calibration_camera.os.read', side_effect=[frames[0][:10], BlockingIOError]):
                stream.request(dict(request_id='measurement', reference={}))
            stream.job['not_before'] = 0.
            with patch('servo_calibration_camera.os.read', side_effect=[frames[0][10:]+frames[1]+frames[2], BlockingIOError]), patch(
                    'servo_calibration_camera.measure_rods', return_value=[180., 360.]) as detector:
                stream.config['consensus_frames'] = 1
                self.assertEqual(stream.poll()['angles'], [180., 360.])
                self.assertAlmostEqual(float(detector.call_args.args[0].mean()), 100., delta=1.)
            # Frames between estimates and still queued at shutdown are retained.
            with patch('servo_calibration_camera.os.read', side_effect=[frames[3], BlockingIOError]):
                self.assertIsNone(stream.poll())
            stream.process.poll.return_value = 0
            with patch('servo_calibration_camera.os.read', side_effect=[frames[4], b'']):
                stream.close()
            self.assertTrue(stream.video_file.closed)
            self.assertTrue(stream.timing_file.closed)
            self.assertEqual(video.read_bytes(), b''.join(frames))
            index = [json.loads(line) for line in timing.read_text().splitlines()]
            self.assertEqual([item['frame_index'] for item in index], list(range(5)))
            for item, frame in zip(index, frames):
                self.assertEqual(item['byte_length'], len(frame))
                data = video.read_bytes()
                self.assertEqual(data[item['byte_offset']:item['byte_offset']+item['byte_length']], frame)
            # Verify the saved stream can actually be played by a video reader.
            reader = cv2.VideoCapture(str(video))
            count = 0
            try:
                while reader.read()[0]:
                    count += 1
            finally:
                reader.release()
            self.assertEqual(count, 5)

    def test_failed_capture_preserves_partial_footage_and_restarts_append(self):
        frame = self.jpeg(50)
        with tempfile.TemporaryDirectory() as directory:
            stream, (video, timing) = self.create_stream(directory)
            stream.process.poll.return_value = 7
            with patch('servo_calibration_camera.os.read', side_effect=[frame, BlockingIOError]):
                with self.assertRaisesRegex(RuntimeError, 'stream exited'):
                    stream.poll()
            with patch('servo_calibration_camera.os.read', side_effect=BlockingIOError):
                stream.close()
            self.assertEqual(video.read_bytes(), frame)
            restarted, _ = self.create_stream(directory)
            restarted.process.poll.return_value = 0
            with patch('servo_calibration_camera.os.read', side_effect=[frame, b'']):
                restarted.close()
            self.assertEqual(video.read_bytes(), frame*2)
            index = [json.loads(line) for line in timing.read_text().splitlines()]
            self.assertEqual([item['byte_offset'] for item in index], [0, len(frame)])


if __name__ == '__main__':
    unittest.main()
