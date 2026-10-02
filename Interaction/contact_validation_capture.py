"""Opt-in independent contact validation: capture only, never a detector input.

The frozen experimental model is embedded for reproducible OFFLINE evaluation.
No fitting or model execution occurs in the controller or sensor callback.
"""
from copy import deepcopy
from dataclasses import asdict
import hashlib
import json
import logging
import math
from pathlib import Path

from Interaction.calibration_contact_logging import calibration_log_vars, capture_group_manifest

logger = logging.getLogger(__name__)


def capture_options(mission):
    options=(((mission or {}).get('Interaction') or {}).get('config') or {}).get('contact_validation_capture', {})
    if not isinstance(options, dict) or type(options.get('enabled', False)) is not bool:
        raise ValueError('contact_validation_capture.enabled must be boolean')
    if options.get('command_authority', False) is not False:
        raise ValueError('contact validation cannot have command authority')
    return options


def validate_capture_request(mission,args):
    options=capture_options(mission)
    if options.get('enabled',False) and (
            not getattr(args,'interaction',False) or not getattr(args,'sense',False)
            or not getattr(args,'log',False) or getattr(args,'droneless',False)
            or getattr(args,'calibrate',False)):
        raise ValueError('contact validation capture requires --interaction --sense --log, without --calibrate')
    return options


def validate_profile(profile, drone_id, mass):
    if (profile.get('schema_version') != 1 or profile.get('drone_id') != drone_id
            or profile.get('command_authority') is not False
            or profile.get('deployment_ready') is not False
            or not math.isclose(float(profile.get('mass_kg', 'nan')), mass, rel_tol=0, abs_tol=1e-9)):
        raise ValueError('frozen contact profile identity, mass or diagnostic-only status mismatch')
    for key, field, size, bound in [('baseline','specific_force_xy_m_s2',2,1.),
                                   ('rotational','effective_offset_m',3,.08)]:
        v=profile.get(key,{}).get(field)
        if (not isinstance(v,(list,tuple)) or len(v)!=size
                or not all(type(x) in (int,float) and math.isfinite(x) for x in v)
                or math.sqrt(sum(x*x for x in v))>bound):
            raise ValueError('invalid frozen contact profile coefficients')
    detector=profile.get('detector')
    if not isinstance(detector,dict):
        raise ValueError('frozen profile must include the detector thresholds')
    for key in ('component_thresholds','covariance_floor'):
        v=detector.get(key)
        if (not isinstance(v,list) or len(v)!=3
                or not all(type(x) in (int,float) and math.isfinite(x) and x>0 for x in v)):
            raise ValueError('invalid frozen detector thresholds')
    if profile.get('onset',{}).get('onset_mode')!='continuous':
        raise ValueError('unsupported frozen detector onset mode')


class PotentiometerSampleCapture:
    """Enqueue immutable samples to the existing async log writer."""
    def __init__(self, live_logger):
        self.live_logger=live_logger
        self.sequence=0

    def __call__(self, sample):
        data=asdict(sample)
        data.update(sample_sequence=self.sequence, time=sample.host_time,
                    command_authority=False)
        self.sequence+=1
        self.live_logger.write({'type':'potentiometer_raw','data':data})


def configure_contact_validation_capture(log_manager, cf, selected, mission, args):
    options=validate_capture_request(mission,args)
    if not options.get('enabled', False):
        return selected
    config=mission['Interaction']['config']
    mass=float(config['wrench_interaction']['mass'])
    path=Path(options['profile_path']).expanduser()
    contents=path.read_bytes()
    digest=hashlib.sha256(contents).hexdigest()
    if options.get('profile_sha256')!=digest:
        raise ValueError('frozen contact profile hash mismatch; refusing an unpinned model')
    profile=json.loads(contents)
    validate_profile(profile,args.drone_id,mass)
    selected=calibration_log_vars(selected)
    groups,count=capture_group_manifest(cf,selected,args.cf_log_period,label='contact validation')
    no_touch=float(options.get('initial_no_touch_s',10.))
    if not math.isfinite(no_touch) or no_touch<0:
        raise ValueError('initial_no_touch_s must be nonnegative')
    manifest=dict(schema_version=1,capture_only=True,protocol='independent_contact_validation',
        command_authority=False,profile_runtime_enabled=False,refit_enabled=False,
        drone_id=args.drone_id,mass_kg=mass,profile_path=str(path.resolve()),
        profile_sha256=digest,frozen_profile=profile,
        profile_object_sha256=hashlib.sha256(json.dumps(profile,sort_keys=True,
            separators=(',',':'),allow_nan=False).encode()).hexdigest(),
        initial_no_touch_s=no_touch,
        no_contact_is_operator_requirement_not_measured_ground_truth=True,
        primary_detector=config.get('detection_method','unchanged'),
        mission_snapshot=deepcopy(mission),groups=groups,variable_subscriptions=count,
        imu_requested_rate_hz=100,imu_timestamp_basis='24-bit FC CRTP log tick, not sensor epoch',
        imu_atomic_sensor_snapshot=False,
        potentiometer_capture='every valid UART sample, before control-loop decimation',
        potentiometer_reference='spring compression proxy, not independent physical-contact truth',
        sensor_settings={key:getattr(args,key,None) for key in
            ('sense_axis','sense_sign','sense_spring_constant','sense_max_extension')},
        requested_packets_per_s=sum(1000./g['requested_period_ms'] for g in groups.values()))
    log_manager.live_logger.write({'type':'contact_validation_capture','data':manifest})
    log_manager.capture_packet_timing=True
    log_manager.contact_validation_pot_callback=PotentiometerSampleCapture(log_manager.live_logger)
    logger.info('Independent contact validation capture enabled: frozen profile %s; raw IMU 100 Hz; '
                'raw potentiometer samples; primary detector unchanged. Do not touch for the first %.1f s '
                'after the initial contact detector arms.',digest[:12],no_touch)
    return selected
