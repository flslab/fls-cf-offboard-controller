"""One physical motion reference for marker-aided coasting, without radio I/O."""
import math
import numpy as np


def requested(config):
    options=config.get('level_coast') or {}
    brake=(config.get('wrench_interaction') or {}).get('firmware_auto_brake') or {}
    profile=brake.get('analytic_profile') or {}
    return (config.get('behavior')=='level_coast' and options.get('coasting_enabled',True)
            and 'marker_from_imu_m' in profile)


def parameters(value):
    if (not isinstance(value,(list,tuple)) or len(value)!=3 or
            any(isinstance(v,bool) or not isinstance(v,(int,float)) or not math.isfinite(v) for v in value)
            or np.linalg.norm(value)>.3):
        raise ValueError('marker_from_imu_m must be three finite metres with norm <= 0.3')
    return {'mocapMarker.api':1,'mocapMarker.enable':1,
            **{'mocapMarker.'+a:float(v) for a,v in zip('xyz',value)}}


def add_logs(selected,mission):
    config=(mission.get('Interaction') or {}).get('config') or {}
    if not requested(config):return selected
    return {**selected,'MOCAP_REFERENCE':{'log_period_ms':10,
        **{'mocapRef.'+v:{'type':'float','data':[]} for v in ('x','y','z','vx','vy','vz')},
        'mocapRef.age':{'type':'FP16','data':[]}}}


def motion_reference(log_manager,state_time,max_skew):
    packet,skew=log_manager.get_nearest_group_log_data('MOCAP_REFERENCE',state_time)
    if not packet or skew is None or not math.isfinite(skew) or abs(skew)>.035:
        raise ValueError('Corrected Vicon motion packet is missing or stale')
    stamp=packet.get('time')
    age=packet.get('mocapRef.age')
    if (not isinstance(stamp,(float,int)) or not math.isfinite(stamp)
            or not isinstance(age,(float,int)) or not math.isfinite(age) or not 0<=age<=.035):
        raise ValueError('Corrected Vicon source age is invalid or stale')
    capture=float(stamp)-age
    delta=float(state_time)-capture
    ceiling=.035 if max_skew is None else min(.035,max_skew)
    if not -.02<=delta<=ceiling:raise ValueError('Corrected Vicon observation epoch is stale')
    p=np.array([packet['mocapRef.'+a] for a in 'xyz'],float)
    v=np.array([packet['mocapRef.v'+a] for a in 'xyz'],float)
    if not np.isfinite(p).all() or not np.isfinite(v).all():
        raise ValueError('Corrected Vicon motion is nonfinite')
    # Predict position to the state epoch, never relabel the source epoch or
    # advance dwell on the basis of an extrapolated/duplicated observation.
    return p+delta*v,v,capture,capture-state_time
