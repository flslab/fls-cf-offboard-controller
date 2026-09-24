"""Read-only curve events and device-clock state recording; never sends setpoints."""
import collections
import logging
import math
from pathlib import Path
import struct
import threading
import time
import uuid
import zlib

from cflib.crtp.crtpstack import CRTPPacket, CRTPPort
from Interaction.live_logger import LiveLogger
from Interaction.compressed_state_logs import decode_kinematic_packet, validate_actuator_packet

VERSION = 2
SUPPORTED_VERSIONS = (1, 2)
PART, ACK, CHUNK = 0xd0, 0xd1, 22
HEADER = struct.Struct('<BBH10I')
FLOATS = struct.Struct('<60f')
TIMING = struct.Struct('<5I')
WIRE_SIZE_V1 = HEADER.size + FLOATS.size + 4
WIRE_SIZE = WIRE_SIZE_V1 + TIMING.size
PART_COUNT = (WIRE_SIZE + CHUNK - 1) // CHUNK
KINDS = {1:'initial', 2:'replan', 3:'hold', 4:'abort', 5:'interrupted'}
LOG = logging.getLogger(__name__)


def curve_protocol_version(cf):
    """Negotiate the wire format, not a firmware build-number allowlist."""
    version = int(cf.param.get_value('hlCommander.curveVer'))
    if version not in SUPPORTED_VERSIONS:
        raise RuntimeError(f'unsupported curve event logging protocol {version}')
    return version


def curve_log_config(mission):
    brake = mission.get('Interaction', {}).get('config', {}).get('wrench_interaction', {}).get('firmware_auto_brake', {})
    config = brake.get('curve_log', {}) or {}
    if not isinstance(config, dict) or type(config.get('enabled', False)) is not bool:
        raise ValueError('firmware_auto_brake.curve_log.enabled must be boolean')
    for key in ('user_id', 'trial_id'):
        value = config.get(key)
        if value is not None and (not isinstance(value, str) or not value.strip() or len(value)>128):
            raise ValueError('curve_log.'+key+' must be a nonempty string up to 128 characters')
    return config


def curve_state_log_vars(selected, *, events_enabled=True):
    from Interaction.config import FIRMWARE_KIN, FIRMWARE_ACT, ATT_DES
    # Reuse already-present compressed blocks; only command and health blocks
    # are new on the normal firmware-braking path.
    result = dict(selected)
    result.update(FIRMWARE_KIN=FIRMWARE_KIN, FIRMWARE_ACT=FIRMWARE_ACT,
                  ATT_DES={**ATT_DES, 'log_period_ms':10})
    if events_enabled:
        result['CURVE_STATUS'] = {'log_period_ms':100,
            'hlCommander.curveDrop':{'type':'uint32_t'},
            'hlCommander.curveId':{'type':'uint32_t'},
            'hlCommander.curveQ':{'type':'uint8_t'},
            'hlCommander.pRelGap0':{'type':'float'},
            'hlCommander.pRelStale0':{'type':'uint8_t'},
            'hlCommander.scQual':{'type':'uint8_t'},
            'hlCommander.scAge':{'type':'float'},
            'pRelVicon.readyErr':{'type':'uint8_t'}}
        result['CURVE_ESTIMATOR'] = {'log_period_ms':100,
            **{name: {'type':'float'} for name in (
                'pRelVicon.vx', 'pRelVicon.vy', 'kalmanPRel.vx',
                'kalmanPRel.vy', 'kalmanPRel.ax', 'kalmanPRel.ay')}}
    return result


def decode_event(wire):
    if not wire or wire[0] not in SUPPORTED_VERSIONS:
        raise ValueError('unsupported curve event version')
    size = WIRE_SIZE if wire[0] == 2 else WIRE_SIZE_V1
    if len(wire)!=size or zlib.crc32(wire[:-4]) != struct.unpack('<I',wire[-4:])[0]:
        raise ValueError('curve event length/CRC mismatch')
    version,kind,sequence,event,session,plan,replaces,applied,tick,origin,model,dropped,flags = HEADER.unpack_from(wire)
    v = FLOATS.unpack_from(wire, HEADER.size)
    if kind not in KINDS or not all(math.isfinite(x) for x in v):
        raise ValueError('unsupported or nonfinite curve event')
    axes=[]
    velocity_mode = bool(flags & 0x10000)
    position_reference = bool(flags & 0x20000)
    if velocity_mode and position_reference:
        raise ValueError('conflicting curve reference units')
    for i in range(2):
        axes.append(dict(duration_s=v[24+2*i], end_value=v[25+2*i],
            split=bool(flags & (1<<i)), coefficients=[list(v[28+16*i:36+16*i]),list(v[36+16*i:44+16*i])],
            response=dict(zip(('delay_s','wn_rad_s','zeta','gain','bias_rad'),v[14+5*i:19+5*i]))))
        # Keep the legacy attitude field only when its units really are radians.
        if not velocity_mode and not position_reference:
            axes[-1]['end_rad'] = v[25+2*i]
    if kind<=2 and any(a['duration_s']<=0 or (not velocity_mode and not position_reference and a['response']['gain']<=0) for a in axes):
        raise ValueError('invalid executable curve')
    result = dict(type='curve', wire_version=version, event=KINDS[kind], event_id=event, session_id=session,
        sequence=sequence, interaction_id=f'{session}:{sequence}', plan_id=plan,
        replaces_plan_id=replaces, applied_us_mod32=applied,
        cf_timestamp_ms=tick, plan_origin_us_mod32=origin, model_id=model,
        firmware_dropped=dropped, attitude_source={1:'ordinary',2:'post_release15',3:'post_release15_rp',
            4:'unified_vicon15',5:'simulation_truth'}.get((flags>>8)&0xff,'none'),
        command_mode='velocity' if velocity_mode else 'attitude',
        coefficient_units='m/s' if velocity_mode else 'rad',
        response_model_used=not velocity_mode,
        velocity_source='ordinary_firmware_state' if velocity_mode else 'scurve_imu_vicon_observer',
        acceleration_source='scurve_imu_vicon_observer',
        curve_axes=['along_velocity','cross_velocity'] if velocity_mode else ['along_equivalent_tilt','cross_equivalent_tilt'],
        direction_xy=list(v[:2]), release_yaw_rad=v[2], velocity_xy_m_s=list(v[3:5]),
        acceleration_xy_m_s2=list(v[5:7]), roll_pitch_deg=list(v[7:9]),
        euler_roll_pitch_rate_deg_s=list(v[9:11]), position_m=list(v[11:14]), axes=axes)
    if position_reference:
        source=(flags>>8)&0xff
        result.update(command_mode='attitude' if flags&0x40000 else 'position',
            reference_kind='position_polynomial',coefficient_units='m',
            response_model_used=False,curve_axes=['world_x','world_y'],
            velocity_source={1:'ordinary_firmware_state',3:'ordinary_firmware_state',
                4:'unified_vicon15',5:'simulation_truth'}.get(source,'unknown'),
            acceleration_source='tilt_approximation',
            reconstruction='reference_only; feedback_command_requires_state_and_runtime_parameters')
    if velocity_mode and kind==2:
        result['event']='phase_transition'
    execution_kind = (flags >> 19) & 3
    if execution_kind:
        result['command_mode'] = {1: 'velocity', 2: 'position', 3: 'attitude'}[execution_kind]
        result['execution_mode_explicit'] = True
        source = (flags >> 8) & 0xff
        result['velocity_source'] = {1: 'ordinary_firmware_state', 3: 'ordinary_firmware_state',
            4: 'unified_vicon15', 5: 'simulation_truth'}.get(source, 'unknown')
        result['response_model_used'] = False
    if flags & 0x200000:
        if not velocity_mode or execution_kind != 3:
            raise ValueError('response compensation requires explicit attitude velocity-reference execution')
        if kind <= 3 and (not model or any(a['response']['gain'] <= 0 or
                                          a['response']['wn_rad_s'] <= 0 for a in axes)):
            raise ValueError('response compensation model missing from executable curve')
        result.update(response_model_used=True,
            response_compensation='delay_state_prediction_and_tail_impulse_feedback',
            response_axes=['roll', 'pitch'],
            reconstruction='reference_only; command also requires state, causal command history and runtime parameters')
    result['timing'] = dict(clock='unavailable', plan_compute_us=None,
        control_step_max_us=None, hold_compute_us=None, control_steps=None)
    if version == 2:
        valid,plan_us,step_us,hold_us,steps = TIMING.unpack_from(wire, HEADER.size+FLOATS.size)
        result['timing'] = dict(
            clock={1:'mcu_elapsed',2:'sitl_host_elapsed'}.get((valid>>8)&0xff,'unavailable'),
            plan_compute_us=plan_us if valid&1 else None,
            control_step_max_us=step_us if valid&2 else None,
            hold_compute_us=hold_us if valid&4 else None,
            control_steps=steps if valid&2 else None,
            scope='bounded runtime; control-step maximum through snapshot, includes planning/handoff but excludes snapshot serialization')
    if kind in (4, 5):
        # Neither v1 nor the timing-only v2 extension carries state validity.
        # In particular failed state acquisition leaves zero-filled buffers.
        fields = ('velocity_xy_m_s', 'acceleration_xy_m_s2', 'position_m',
                  'roll_pitch_deg', 'euler_roll_pitch_rate_deg_s')
        result['raw_unverified_state'] = {key: result[key] for key in fields}
        result.update({key: None for key in fields})
        result['state_validity'] = f'unknown_terminal_state_wire_v{version}'
    else:
        result['state_validity'] = 'runtime_accepted'
    return result


class CurveAssembler:
    """Bounded fragment assembly. Retransmitted complete events are ACKed again."""
    def __init__(self):
        self.pending = collections.OrderedDict()
        self.complete = collections.OrderedDict()
        self.last_version = VERSION

    def feed(self, data):
        data=bytes(data)
        if len(data)<8 or data[0]!=PART or data[1] not in SUPPORTED_VERSIONS:
            raise ValueError('invalid curve fragment header')
        version=data[1];self.last_version=version
        wire_size=WIRE_SIZE if version==2 else WIRE_SIZE_V1
        event,index,count=struct.unpack_from('<IBB',data,2)
        expected=min(CHUNK,wire_size-index*CHUNK)
        if count!=(wire_size+CHUNK-1)//CHUNK or index>=count or len(data)!=8+expected:
            raise ValueError('invalid curve fragment size/index')
        if (version,event) in self.complete:
            return None,event
        if event not in self.pending and len(self.pending)>=8:
            lost,_=self.pending.popitem(last=False)
            raise ValueError(f'too many incomplete curve events; evicted {lost}')
        pending=self.pending.setdefault(event,{'version':version,'parts':{}})
        if pending['version']!=version:
            del self.pending[event]
            raise ValueError('conflicting curve fragment version')
        parts=pending['parts']
        if index in parts and parts[index]!=data[8:]:
            del self.pending[event]
            raise ValueError('conflicting curve fragment')
        parts[index]=data[8:]
        if len(parts)!=count:return None,None
        wire=b''.join(parts[i] for i in range(count))
        del self.pending[event]
        result=decode_event(wire)
        if result['event_id']!=event:raise ValueError('curve identity mismatch')
        return result,event

    def saved(self,event):
        self.complete[self.last_version,event]=True
        while len(self.complete)>128:self.complete.popitem(last=False)


def normalize_state(packet):
    d=packet.data;result=dict(type='state',group=packet.group,
        cf_timestamp_ms=packet.cf_timestamp_ms, host_receive_time_s=packet.host_receive_time_s,
        source_snapshot_atomic=packet.source_snapshot_atomic,raw=dict(d))
    if packet.group=='FIRMWARE_KIN':
        p,v,rpy,rates=decode_kinematic_packet(d)
        result.update(position_m=p.tolist(),velocity_m_s=v.tolist(),
            roll_pitch_deg=[math.degrees(x) for x in rpy[:2]],
            body_rates_legacy_deg_s=[math.degrees(x) for x in rates])
        phi,pitch=rpy[:2];p,qlegacy,r=rates
        if abs(math.cos(pitch))>.087:
            result['euler_roll_pitch_rate_deg_s']=[math.degrees(p+math.tan(-pitch)*(-math.sin(phi)*qlegacy+math.cos(phi)*r)),
                math.degrees(math.cos(phi)*qlegacy+math.sin(phi)*r)]
        else:result['euler_roll_pitch_rate_deg_s']=None
    elif packet.group=='FIRMWARE_ACT':
        validate_actuator_packet(d)
        a=[.001*d['stateEstimateZ.a'+axis] for axis in 'xyz']
        result['acceleration_world_m_s2']=[a[0],a[1],a[2]-9.81]
        result['acceleration_encoding']='stateEstimateZ: mm/s2, z includes +g'
    elif packet.group=='ATT_DES':
        result['command_roll_pitch_deg']=[d['controller.roll'],d['controller.pitch']]
    return result


class CurveRecorder:
    def __init__(self,cf,logger,*,directory,tag,user_id=None,trial_id=None,events_enabled=True,protocol_version=VERSION):
        self.cf=cf;self.logger=logger;self.events_enabled=events_enabled
        self.protocol_version=protocol_version
        self.ids=dict(run_id=uuid.uuid4().hex,user_id=user_id,trial_id=trial_id or tag,
                      drone_id=getattr(logger.args,'drone_id',None))
        directory=Path(directory);directory.mkdir(parents=True,exist_ok=True)
        self.events=LiveLogger(directory/(tag+'.curves.jsonl'),json_lines=True,exclusive=True)
        try:self.states=LiveLogger(directory/(tag+'.states.jsonl'),json_lines=True,exclusive=True)
        except Exception:self.events.close();raise
        self.lock=threading.RLock();self.assembler=CurveAssembler();self.closed=False
        self.last_event=None;self.gaps=0;self.errors=0;self.firmware_drops=0;self.firmware_last=0;self.queue_depth=0
        self.status_seen=False
        self.state_counts=collections.Counter();self.state_gaps=collections.Counter();self.state_last={}
        self.callback=self._packet
        metadata=dict(type='metadata',schema_version=VERSION,**self.ids,events_enabled=events_enabled,
            user_id_missing=user_id is None,polynomial_basis='ascending powers of normalized local segment time',
            curve_axes='per_event: inspect command_mode, curve_axes and coefficient_units',
            timestamp_basis='CRTP 24-bit tick ms; each curve includes paired usec32 and tick24 anchors',
            continuous_attitude_source='stateEstimateZ (published firmware state; inspect event attitude_source and runtime configuration)',
            curve_wire_version=protocol_version,
            compute_timing_available=events_enabled and protocol_version>=2,
            continuous_groups_atomic=False, requested_state_period_ms=10)
        self.events.write(metadata);self.states.write(metadata)
        if events_enabled and user_id is None:
            LOG.warning('Curve recording has no user_id; set a pseudonymous curve_log.user_id for between-user comparisons')
        if events_enabled and protocol_version<2:
            LOG.warning('Curve wire v1 records coefficients/states but has no per-curve computation timing')
        self.unsubscribe=logger.add_cf_packet_listener(self._state)
        if events_enabled:cf.add_port_callback(CRTPPort.SETPOINT_HL,self.callback)

    def _error(self,error):
        self.errors+=1;LOG.error('Curve log incomplete: %s',error)
        try:self.events.write(dict(type='integrity_error',message=str(error),**self.ids))
        except Exception:LOG.exception('Curve log error could not be written')

    def _packet(self,packet):
        if packet.channel!=2:return
        with self.lock:
            if self.closed:return
            try:
                result,event=self.assembler.feed(packet.data)
                if result:
                    if self.last_event is not None and ((event-self.last_event)&0xffffffff)!=1:
                        self.gaps+=1
                        self.events.write(dict(type='event_gap',after=self.last_event,before=event,**self.ids))
                    self.events.write(dict(result,**self.ids,host_receive_time_s=time.time()))
                    self.assembler.saved(event);self.last_event=event
                    self.firmware_drops=max(self.firmware_drops,result['firmware_dropped'])
                if event is not None:
                    ack=CRTPPacket();ack.set_header(CRTPPort.SETPOINT_HL,2)
                    ack.data=struct.pack('<BBI',ACK,packet.data[1],event);self.cf.send_packet(ack)
            except Exception as error:self._error(error)

    def _state(self,packet):
        if packet.group not in ('FIRMWARE_KIN','FIRMWARE_ACT','ATT_DES','CURVE_STATUS','CURVE_ESTIMATOR'):return
        with self.lock:
            if self.closed:return
            try:
                self.states.write(dict(normalize_state(packet),**self.ids))
                self.state_counts[packet.group]+=1
                previous=self.state_last.get(packet.group)
                if previous is not None and packet.group not in ('CURVE_STATUS','CURVE_ESTIMATOR'):
                    dt=(packet.cf_timestamp_ms-previous)&0xffffff
                    if dt>15:self.state_gaps[packet.group]+=1
                self.state_last[packet.group]=packet.cf_timestamp_ms
                if packet.group=='CURVE_STATUS':
                    self.status_seen=True
                    self.firmware_drops=max(self.firmware_drops,int(packet.data['hlCommander.curveDrop']))
                    self.firmware_last=int(packet.data['hlCommander.curveId'])
                    self.queue_depth=int(packet.data['hlCommander.curveQ'])
            except Exception as error:self._error(error)

    def check(self):
        for writer in (self.events,self.states):
            if writer.stats_snapshot()['error']:raise RuntimeError('curve/state log writer failed')
        if self.errors:raise RuntimeError('curve log has integrity errors')

    def close(self):
        with self.lock:
            if self.closed:return
        # Called after landing while the link remains open; allow queued
        # snapshots to finish, but never indefinitely wait on a missing packet.
        deadline=time.monotonic()+3
        while self.events_enabled and self.queue_depth and time.monotonic()<deadline:time.sleep(.02)
        self.unsubscribe()
        if self.events_enabled:self.cf.remove_port_callback(CRTPPort.SETPOINT_HL,self.callback)
        with self.lock:
            if self.closed:return
            self.closed=True
        # Finish the continuous file before claiming completeness in the event
        # file. A disk failure while draining must not leave a success summary.
        state_error=None
        try:self.states.close()
        except Exception as error:
            state_error=error
            self._error(error)
        try:
            missing_tail=self.events_enabled and (not self.status_seen or self.firmware_last!=(self.last_event or 0))
            writer_error=any(w.stats_snapshot()['error'] for w in (self.events,self.states))
            events_complete=not (self.errors or self.gaps or self.firmware_drops or self.assembler.pending or self.queue_depth or missing_tail or writer_error)
            states_complete=all(self.state_counts[g] for g in ('FIRMWARE_KIN','FIRMWARE_ACT','ATT_DES')) and not any(self.state_gaps.values()) and not state_error
            complete=events_complete and states_complete
            self.events.write(dict(type='summary',**self.ids,complete=complete,
                curve_events_complete=events_complete,continuous_states_complete=states_complete,
                last_event_id=self.last_event,firmware_last_event_id=self.firmware_last,
                firmware_dropped=self.firmware_drops,event_gaps=self.gaps,errors=self.errors,
                incomplete_events=list(self.assembler.pending),firmware_pending=self.queue_depth,
                status_seen=self.status_seen,missing_tail=missing_tail,
                state_counts=dict(self.state_counts),state_gap_counts=dict(self.state_gaps)))
        finally:self.events.close()
        if state_error:raise state_error
        if not complete:LOG.error('Curve log is incomplete; exclude this trial from complete-curve comparisons')
