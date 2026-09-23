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

VERSION = 1
PART, ACK, CHUNK = 0xd0, 0xd1, 22
HEADER = struct.Struct('<BBH10I')
FLOATS = struct.Struct('<60f')
WIRE_SIZE = HEADER.size + FLOATS.size + 4
PART_COUNT = (WIRE_SIZE + CHUNK - 1) // CHUNK
KINDS = {1:'initial', 2:'replan', 3:'hold', 4:'abort', 5:'interrupted'}
LOG = logging.getLogger(__name__)


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
            'kalmanPRel.readyErr':{'type':'uint8_t'}}
    return result


def decode_event(wire):
    if len(wire)!=WIRE_SIZE or zlib.crc32(wire[:-4]) != struct.unpack('<I',wire[-4:])[0]:
        raise ValueError('curve event length/CRC mismatch')
    version,kind,sequence,event,session,plan,replaces,applied,tick,origin,model,dropped,flags = HEADER.unpack_from(wire)
    v = FLOATS.unpack_from(wire, HEADER.size)
    if version!=VERSION or kind not in KINDS or not all(math.isfinite(x) for x in v):
        raise ValueError('unsupported or nonfinite curve event')
    axes=[]
    for i in range(2):
        axes.append(dict(duration_s=v[24+2*i], end_rad=v[25+2*i],
            split=bool(flags & (1<<i)), coefficients=[list(v[28+16*i:36+16*i]),list(v[36+16*i:44+16*i])],
            response=dict(zip(('delay_s','wn_rad_s','zeta','gain','bias_rad'),v[14+5*i:19+5*i]))))
    if kind<=2 and any(a['duration_s']<=0 or a['response']['gain']<=0 for a in axes):
        raise ValueError('invalid executable curve')
    return dict(type='curve', event=KINDS[kind], event_id=event, session_id=session,
        sequence=sequence, interaction_id=f'{session}:{sequence}', plan_id=plan,
        replaces_plan_id=replaces, applied_us_mod32=applied,
        cf_timestamp_ms=tick, plan_origin_us_mod32=origin, model_id=model,
        firmware_dropped=dropped, attitude_source={1:'ordinary',2:'post_release15'}.get(flags>>8,'none'),
        direction_xy=list(v[:2]), release_yaw_rad=v[2], velocity_xy_m_s=list(v[3:5]),
        acceleration_xy_m_s2=list(v[5:7]), roll_pitch_deg=list(v[7:9]),
        euler_roll_pitch_rate_deg_s=list(v[9:11]), position_m=list(v[11:14]), axes=axes)


class CurveAssembler:
    """Bounded fragment assembly. Retransmitted complete events are ACKed again."""
    def __init__(self):
        self.pending = collections.OrderedDict()
        self.complete = collections.OrderedDict()

    def feed(self, data):
        data=bytes(data)
        if len(data)<8 or data[:2]!=bytes((PART,VERSION)):
            raise ValueError('invalid curve fragment header')
        event,index,count=struct.unpack_from('<IBB',data,2)
        expected=min(CHUNK,WIRE_SIZE-index*CHUNK)
        if count!=PART_COUNT or index>=count or len(data)!=8+expected:
            raise ValueError('invalid curve fragment size/index')
        if event in self.complete:
            return None,event
        if event not in self.pending and len(self.pending)>=8:
            lost,_=self.pending.popitem(last=False)
            raise ValueError(f'too many incomplete curve events; evicted {lost}')
        parts=self.pending.setdefault(event,{})
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
        self.complete[event]=True
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
    def __init__(self,cf,logger,*,directory,tag,user_id=None,trial_id=None,events_enabled=True):
        self.cf=cf;self.logger=logger;self.events_enabled=events_enabled
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
            curve_axes=['along_equivalent_tilt_rad','cross_equivalent_tilt_rad'],
            timestamp_basis='CRTP 24-bit tick ms; each curve includes paired usec32 and tick24 anchors',
            continuous_attitude_source='stateEstimateZ (ordinary firmware state, not necessarily curve attitude source)',
            continuous_groups_atomic=False, requested_state_period_ms=10)
        self.events.write(metadata);self.states.write(metadata)
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
                    ack.data=struct.pack('<BBI',ACK,VERSION,event);self.cf.send_packet(ack)
            except Exception as error:self._error(error)

    def _state(self,packet):
        if packet.group not in ('FIRMWARE_KIN','FIRMWARE_ACT','ATT_DES','CURVE_STATUS'):return
        with self.lock:
            if self.closed:return
            try:
                self.states.write(dict(normalize_state(packet),**self.ids))
                self.state_counts[packet.group]+=1
                previous=self.state_last.get(packet.group)
                if previous is not None and packet.group!='CURVE_STATUS':
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
