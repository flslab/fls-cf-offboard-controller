"""Addressed calibration commands, accepted only at the illumination barrier.

Offsets here have the same sign/units as --servo-offsets (degrees added to
SFL angles). Hardware mounting offsets remain in Servo, not in this protocol.
"""
import math
import time


def finite_pair(values):
    if (not isinstance(values, (list, tuple)) or len(values) != 2 or
            any(isinstance(v, bool) or not isinstance(v, (float, int)) or
                not math.isfinite(v) for v in values)):
        raise ValueError('expected two finite servo values')
    return list(values)


def calibration_colors(count, selected):
    """Both rods stay lit; the selected drone has directed two-color rods.

    Strip indices run tip -> hinge -> tip. Red/green mark the inner halves,
    cyan/blue the outer halves. Other drones remain dim, unsaturated white.
    """
    if not selected:
        return [(24, 24, 24)] * count
    middle = count // 2
    return [((0, 180, 180) if i < middle // 2 else (180, 0, 0))
            if i < middle else
            ((0, 180, 0) if i < middle + (count-middle)//2 else (0, 0, 180))
            for i in range(count)]


def handle_command(controller, message):
    """Apply an absolute, retry-safe update and acknowledge actual application."""
    if message.get('drone_id') != controller.args.drone_id:
        return
    if message.get('tag') != controller.args.tag:
        return
    reply = dict(id=controller.args.drone_id, status='SERVO_APPLIED',
                 tag=message.get('tag'), request_id=message.get('request_id'))
    try:
        config = controller.manifest.get('servo_calibration', {})
        if (not config.get('enabled') or
                getattr(controller, '_servo_calibration_phase', None) != message.get('mission_index') or
                message.get('mission_index') is None):
            raise ValueError('servo calibration is outside its mission handshake')
        if not controller.args.servo or not controller.args.led or controller.args.servo_count != 2:
            raise ValueError('servo calibration requires two servos and addressable LEDs')
        angles = finite_pair(message.get('angles'))
        offsets = finite_pair(message.get('offsets'))
        base = [0, 180] if controller.args.servo_type == 'H' else [90, 270]
        # Check every sample that will use these session-wide offsets, not just
        # the current target. SmoothController otherwise silently clips them.
        rows = [angles]
        for mission in controller.missions:
            rows.extend(mission.get('drones', {}).get(controller.args.drone_id, {}).get('servos', []))
        for row in rows:
            row = finite_pair(row)
            if any(not lo <= a+o <= lo+180 for lo, a, o in zip(base, row, offsets)):
                raise ValueError('servo limit reached by calibration offset')
        if getattr(controller, '_servo_calibration_deadline', None) is None:
            cleanup_budget = config.get('command_timeout_s', 3.) * (len(controller.manifest.get('drones', []))+1)
            controller._servo_calibration_deadline = time.monotonic()+config.get('timeout_s', 60.)+cleanup_budget+10.
        with controller.smooth_controller.lock:
            group = controller.smooth_controller.groups['servos']
            group['offsets'] = list(offsets)
            # Force a hardware write even on retries/unchanged smooth targets;
            # an ACK must also expose a failed PWM/LED callback.
            corrected = [a+o for a, o in zip(angles, offsets)]
            controller.servo.set_all(corrected)
            for field in ('values', 'targets', 'starts'):
                group[field] = list(corrected)
            group['durations'] = [0., 0.]
            controller.led.set_colors(
                [(0, 0, 0)] * controller.args.led_count if message.get('finish')
                else calibration_colors(controller.args.led_count, message.get('selected', False)))
            controller.args.servo_offsets = list(offsets)
        reply['offsets'] = offsets
    except Exception as error:
        reply.update(status='SERVO_APPLY_FAILED', error=str(error))
    controller.push_socket.send_json(reply)
