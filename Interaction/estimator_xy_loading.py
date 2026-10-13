"""Transactional loading of a frozen XY fit; never changes the active estimator."""
from copy import deepcopy
import time
from uuid import uuid4

import numpy as np

from Interaction.estimator_hover_initialization import apply_hover_roll_pitch
from Interaction.firmware_parameter_confirmation import confirm_firmware_mode_parameters

API = 1
FIELDS = ('sx', 'sy', 'bx', 'by')
REQUIRED = ('api', 'req', 'ack', 'on', 'status', *FIELDS)


def firmware_xy_coefficients(candidate, fit, fingerprint):
    effective = apply_hover_roll_pitch(candidate, fit, fingerprint)
    if candidate.get('fit_passed') is not True:
        raise ValueError('XY loading requires an accepted static calibration')
    A = np.asarray(effective['measured_from_reference'], float)
    if (A.shape != (3, 3) or not np.isfinite(A).all()
            or not np.allclose(A, np.diag(np.diag(A)), atol=1e-9, rtol=0)):
        raise ValueError('onboard XY loading requires diagonal gravity-norm calibration')
    b = np.asarray(effective['accel_bias_m_s2'], float)
    result = dict(sx=1./A[0, 0], sy=1./A[1, 1], bx=b[0]/A[0, 0], by=b[1]/A[1, 1])
    if (not all(np.isfinite(v) for v in result.values())
            or not .8 <= result['sx'] <= 1.2 or not .8 <= result['sy'] <= 1.2
            or max(abs(result['bx']), abs(result['by'])) > .75):
        raise ValueError('onboard XY calibration exceeds firmware bounds')
    return result


def onboard_xy_candidate(coefficients):
    """Exact replay representation: raw Z and gyro, calibrated X/Y only."""
    c = coefficients
    return dict(fit_passed=True,
        measured_from_reference=np.diag([1./c['sx'], 1./c['sy'], 1.]).tolist(),
        accel_bias_m_s2=[c['bx']/c['sx'], c['by']/c['sy'], 0.],
        gyro_residual_bias_rad_s=[0., 0., 0.])


def prepare_xy_loading(cf):
    missing = set(REQUIRED)-set(cf.param.toc.toc.get('eskfXY', {}))
    if missing:
        raise RuntimeError('firmware lacks estimator-3 XY loading API: '+', '.join(sorted(missing)))
    cf.param.set_value('eskfXY.req', '0')
    return confirm_firmware_mode_parameters(cf.param, expected={
        'stabilizer.estimator':2, 'eskfXY.api':API, 'eskfXY.req':0,
        'eskfXY.ack':0, 'eskfXY.on':0, 'eskfXY.status':0})


def load_xy(cf, coefficients, *, timeout_s=2., commit_guard=None):
    """Run in a worker while the flight loop keeps sending position hold.

    Stage and freshly read all four floats before committing a generation.
    A different generation, timeout or rejection is never reported as success.
    Firmware rejects another commit for the rest of the armed flight.
    """
    confirm_firmware_mode_parameters(cf.param, expected={
        'stabilizer.estimator':2, 'eskfXY.api':API, 'eskfXY.on':0})
    expected = {'eskfXY.'+k:float(coefficients[k]) for k in FIELDS}
    for name, value in expected.items():
        cf.param.set_value(name, str(value))
    confirm_firmware_mode_parameters(cf.param, expected=expected)
    generation = (uuid4().int & 0x7fffffff) or 1
    commit = lambda: cf.param.set_value('eskfXY.req', str(generation))
    commit_guard(commit) if commit_guard is not None else commit()
    committed = {
        **expected, 'eskfXY.ack':generation, 'eskfXY.on':1, 'eskfXY.status':1,
        'stabilizer.estimator':2}
    deadline = time.monotonic()+timeout_s
    while True:
        try:
            # Read-only ACK variables change in the estimator task, so the
            # parameter server must be polled for fresh replies after commit.
            observed = confirm_firmware_mode_parameters(cf.param,
                timeout_s=min(.05, max(.001, deadline-time.monotonic())), expected=committed)
            break
        except RuntimeError as error:
            if time.monotonic() >= deadline:
                raise RuntimeError('estimator-3 XY calibration commit was not acknowledged: '+str(error)) from error
    return dict(api=API, generation=generation, coefficients=deepcopy(coefficients),
                firmware_applied=True, frozen=True, z_changed=False, gyro_changed=False,
                yaw_calibrated=False, readback=observed, applied_monotonic_s=time.monotonic())
