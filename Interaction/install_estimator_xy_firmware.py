"""Install the small XY interface into the paired classic firmware source.

Only source files are edited; no flashing or device access. Saves the exact
patch plus source fingerprints alongside the build for reproducibility.
"""
import argparse
import difflib
import hashlib
import json
from pathlib import Path
import shutil


def install(root, evidence):
    root, evidence = Path(root), Path(evidence)
    source = root/'src/modules/src/estimator/estimator_kalman.c'
    original = source.read_text()
    if 'eskfXY' in original:
        raise ValueError('XY interface already installed; do not reapply')
    if 'estimatorPostReleaseVicon15' not in original or 'scfEnabled' not in original:
        raise ValueError('not the paired classic estimator-3 firmware')
    updated = original.replace('#include "estimator.h"',
        '#include "estimator.h"\n#ifdef CONFIG_ESTIMATOR_KALMAN_POST_RELEASE_HARDWARE_BRAKE\n'
        '#include "kalman_core/estimator_xy_calibration.h"\n#endif', 1)
    updated = updated.replace('static scfObserver_t scfObserver;',
        'static estimatorXYCalibration_t estimatorXY = {.staged={1,1,0,0}};\n'
        'static uint8_t estimatorXYApi = 1;\nstatic scfObserver_t scfObserver;', 1)
    anchor = 'static void postReleaseShadowPrepareIteration(void) {'
    updated = updated.replace(anchor, anchor+'''
#ifdef CONFIG_ESTIMATOR_KALMAN_POST_RELEASE_HARDWARE_BRAKE
  taskENTER_CRITICAL();
  const bool xyLoaded = estimatorXYProcess(&estimatorXY, supervisorIsArmed(),
    stateEstimatorGetType() == StateEstimatorTypeKalman, postReleaseHardwareBrakeAccepted);
  taskEXIT_CRITICAL();
  if (xyLoaded) {
    // Discard uncorrected ESKF history before any authority switch. The
    // ordinary KF stays running and seeds a new, corrected propagation epoch.
    postReleaseShadowEnablePrevious = false;
    postReleaseShadowActive = 0;
    postReleaseShadowArmPending = true;
  }
#endif
''', 1)
    updated = updated.replace('const Axis3f specificForceMps2 = {', 'Axis3f specificForceMps2 = {', 1)
    anchor = '  postReleaseInertialEkfPropagate(\n    &postReleaseShadow, &postReleaseShadowParams,'
    if updated.count(anchor) != 1:
        raise ValueError('unexpected propagation site; inspect paired firmware')
    updated = updated.replace(anchor,
        '#ifdef CONFIG_ESTIMATOR_KALMAN_POST_RELEASE_HARDWARE_BRAKE\n'
        '  estimatorXYCorrect(&estimatorXY, specificForceMps2.axis);\n#endif\n'+anchor, 1)
    params = '''#ifdef CONFIG_ESTIMATOR_KALMAN_POST_RELEASE_HARDWARE_BRAKE
/* Staged XY calibration; req commits once, ACK identifies the frozen set.
 * Automatically cleared on disarm; no flash persistence or in-contact fit. */
PARAM_GROUP_START(eskfXY)
  PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, api, &estimatorXYApi)
  PARAM_ADD(PARAM_FLOAT, sx, &estimatorXY.staged.sx)
  PARAM_ADD(PARAM_FLOAT, sy, &estimatorXY.staged.sy)
  PARAM_ADD(PARAM_FLOAT, bx, &estimatorXY.staged.bx)
  PARAM_ADD(PARAM_FLOAT, by, &estimatorXY.staged.by)
  PARAM_ADD(PARAM_UINT32, req, &estimatorXY.request)
  PARAM_ADD(PARAM_UINT32 | PARAM_RONLY, ack, &estimatorXY.applied)
  PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, on, &estimatorXY.enabled)
  PARAM_ADD(PARAM_UINT8 | PARAM_RONLY, status, &estimatorXY.status)
PARAM_GROUP_STOP(eskfXY)
#endif

'''
    updated = updated.replace('PARAM_GROUP_START(kalmanPRel)', params+'PARAM_GROUP_START(kalmanPRel)', 1)
    evidence.mkdir(parents=True, exist_ok=True)
    patch = ''.join(difflib.unified_diff(original.splitlines(True), updated.splitlines(True),
                fromfile='a/src/modules/src/estimator/estimator_kalman.c',
                tofile='b/src/modules/src/estimator/estimator_kalman.c'))
    (evidence/'estimator_xy.patch').write_text(patch)
    source.write_text(updated)
    header = Path(__file__).with_name('native_estimator3')/'estimator_xy_calibration.h'
    shutil.copy2(header, root/'src/modules/interface/kalman_core'/header.name)
    (evidence/'source.json').write_text(json.dumps(dict(root=str(root.resolve()),
        source_before_sha256=hashlib.sha256(original.encode()).hexdigest(),
        source_after_sha256=hashlib.sha256(updated.encode()).hexdigest(),
        header_sha256=hashlib.sha256(header.read_bytes()).hexdigest(),
        flashed=False), indent=2)+'\n')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('firmware_root',type=Path)
    parser.add_argument('--evidence',required=True,type=Path)
    args=parser.parse_args()
    install(args.firmware_root,args.evidence)
