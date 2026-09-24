"""Offline reconstruction and CSV export; no hardware or flight commands.

python -m Interaction.curve_log_export logs/TAG.curves.jsonl
"""
import argparse
import bisect
from collections import Counter
import csv
import json
import math
from pathlib import Path


def delta_ms(a,b):
    return ((int(a)-int(b)+(1<<23)) & 0xffffff)-(1<<23)


def reconstruct_command(event,tick_ms):
    """Exact reference convention (including model inverse and joint clamp)."""
    if event.get('coefficient_units')=='m/s' or event.get('command_mode')=='velocity' or event.get('reference_kind')=='position_polynomial':return None
    if event['event'] not in ('initial','replan'):return None
    age=((event['applied_us_mod32']-event['plan_origin_us_mod32'])&0xffffffff)*1e-6
    age+=delta_ms(tick_ms,event['cf_timestamp_ms'])*.001
    if age<0:return None
    command=[]
    for axis in event['axes']:
        T=axis['duration_s']
        if age>=T:x=axis['end_rad']
        else:
            h=T/2 if axis['split'] else T
            second=axis['split'] and age>=h
            u=(age-h if second else age)/h
            c=axis['coefficients'][int(second)]
            x=c[-1]
            for coefficient in reversed(c[:-1]):x=x*u+coefficient
        m=axis['response'];command.append(max(-.395,min(.395,(x-m['bias_rad'])/m['gain'])))
    norm=math.hypot(*command)
    if norm>.395:command=[x*.395/norm for x in command]
    dx,dy=event['direction_xy'];yaw=event['release_yaw_rad']
    ux=math.cos(yaw)*dx+math.sin(yaw)*dy;uy=-math.sin(yaw)*dx+math.cos(yaw)*dy
    along,cross=command
    return [math.degrees(-uy*along-ux*cross),math.degrees(-ux*along+uy*cross)]


def reconstruct_velocity(event, tick_ms):
    if event.get('reference_kind')=='position_polynomial':
        if event['event'] not in ('initial','replan'):return None
        age=((event['applied_us_mod32']-event['plan_origin_us_mod32'])&0xffffffff)*1e-6
        age+=delta_ms(tick_ms,event['cf_timestamp_ms'])*.001
        if age<0:return None
        values=[]
        for axis in event['axes']:
            if age>=axis['duration_s']:
                values.append(0.);continue
            h=axis['duration_s']/(2 if axis['split'] else 1)
            second=axis['split'] and age>=h
            u=(age-h if second else age)/h
            c=axis['coefficients'][int(second)]
            value=0.
            for i in range(7,0,-1):value=value*u+i*c[i]
            values.append(value/h)
        return values  # Already in world XY, not the release-direction frame.
    if event.get('coefficient_units') != 'm/s' or event['event'] not in ('initial','phase_transition'):
        return None
    age=((event['applied_us_mod32']-event['plan_origin_us_mod32'])&0xffffffff)*1e-6
    age+=delta_ms(tick_ms,event['cf_timestamp_ms'])*.001
    if age<0:return None
    values=[]
    for axis in event['axes']:
        if age>=axis['duration_s']:
            values.append(axis['end_value']);continue
        u=age/axis['duration_s'];c=axis['coefficients'][0];value=c[-1]
        for coefficient in reversed(c[:-1]):value=value*u+coefficient
        values.append(value)
    along=max(0.,values[0]);cross=values[1];dx,dy=event['direction_xy']
    return [dx*along-dy*cross,dy*along+dx*cross]


def _export_coefficients(curves, base, complete, events_complete):
    fields=['user_id','trial_id','run_id','drone_id','recording_complete','curve_events_complete',
        'interaction_id','event_id','event','plan_id','replaces_plan_id',
        'command_mode','coefficient_units','axis','segment','segment_duration_s',
        'axis_duration_s','polynomial_basis','curve_reference_valid','attitude_source','velocity_source',
        'timing_clock','plan_compute_us','control_step_max_us','hold_compute_us',
        'control_steps',*[f'c{i}' for i in range(8)]]
    with open(base+'.coefficients.csv','x',newline='') as output:
        writer=csv.DictWriter(output,fieldnames=fields);writer.writeheader()
        for event in curves:
            timing=event.get('timing',{})
            for index,axis in enumerate(event['axes']):
                segments=2 if axis['split'] else 1
                valid=axis['duration_s']>0 and event['event']!='interrupted'
                for segment in range(segments):
                    row={key:event.get(key) for key in fields if key in event}
                    row.update(recording_complete=complete,curve_events_complete=events_complete,
                        curve_reference_valid=valid,axis=event['curve_axes'][index],
                        segment=segment,segment_duration_s=axis['duration_s']/segments if valid else None,
                        axis_duration_s=axis['duration_s'] if valid else None,
                        polynomial_basis='c0+c1*u+...+c7*u^7; u=local_segment_time/segment_duration',
                        timing_clock=timing.get('clock'))
                    for key in ('plan_compute_us','control_step_max_us','hold_compute_us','control_steps'):
                        row[key]=timing.get(key)
                    if valid:row.update({f'c{i}':c for i,c in enumerate(axis['coefficients'][segment])})
                    writer.writerow(row)
    return base+'.coefficients.csv'


def export(curves_path, *, allow_incomplete=False, coefficients_only=False):
    curves_path=Path(curves_path)
    base=str(curves_path).removesuffix('.curves.jsonl')
    def read(path):
        with open(path) as stream:return [json.loads(line) for line in stream if line.strip()]
    events=read(curves_path);states=read(base+'.states.jsonl')
    event_meta=next((e for e in events if e['type']=='metadata'),{})
    state_meta=next((s for s in states if s['type']=='metadata'),{})
    run_id=event_meta.get('run_id')
    if (not run_id or state_meta.get('run_id')!=run_id
            or any(s.get('run_id')!=run_id for s in states)
            or any(e.get('run_id')!=run_id for e in events)):
        raise ValueError('curve/state files do not belong to the same recording')
    summaries=[e for e in events if e['type']=='summary']
    samples=[s for s in states if s['type']=='state']
    counts=dict(Counter(s['group'] for s in samples))
    complete=bool(summaries and summaries[-1]['complete'] and summaries[-1]['state_counts']==counts)
    events_complete=bool(summaries and summaries[-1].get('curve_events_complete',summaries[-1]['complete']))
    if coefficients_only:
        if not allow_incomplete and not events_complete:
            raise ValueError('curve event recording incomplete')
        return _export_coefficients([e for e in events if e['type']=='curve'],base,complete,events_complete)
    if not allow_incomplete and not complete:
        raise ValueError('curve recording incomplete; use --allow-incomplete only for diagnosis')
    if not samples:raise ValueError('no continuous state samples')
    # Both exports are exclusive; never silently overwrite a previous analysis.
    for suffix in ('.comparison.csv', '.coefficients.csv'):
        if Path(base+suffix).exists():raise FileExistsError(base+suffix)
    anchor=samples[0]['cf_timestamp_ms']
    groups={}
    for s in samples:
        s=dict(s,relative_ms=delta_ms(s['cf_timestamp_ms'],anchor))
        groups.setdefault(s['group'],[]).append(s)
    for rows in groups.values():rows.sort(key=lambda x:x['relative_ms'])
    # Runs must be shorter than half a log-clock wrap (~2.33 h). Do not align
    # by host receipt time: fragments may arrive far later than activation.
    curves=sorted((e for e in events if e['type']=='curve'),key=lambda e:delta_ms(e['cf_timestamp_ms'],anchor))
    ct=[delta_ms(e['cf_timestamp_ms'],anchor) for e in curves]
    lookup={g:[s['relative_ms'] for s in rows] for g,rows in groups.items()}
    def nearest(group,t):
        rows=groups.get(group,[]);ticks=lookup.get(group,[])
        i=bisect.bisect_left(ticks,t);candidates=rows[max(0,i-1):i+1]
        if not candidates:return None
        result=min(candidates,key=lambda x:abs(x['relative_ms']-t))
        return result if abs(result['relative_ms']-t)<=5 else None
    fields=['user_id','trial_id','run_id','recording_complete','interaction_id','plan_id','cf_relative_s','curve_start_relative_s',
        'vx','vy','vz','ax','ay','az','roll_deg','pitch_deg','roll_rate_deg_s','pitch_rate_deg_s',
        'body_p_deg_s','body_q_legacy_deg_s','command_roll_deg','command_pitch_deg',
        'reconstructed_roll_deg','reconstructed_pitch_deg','reference_vx_m_s','reference_vy_m_s',
        'command_mode','coefficient_units','attitude_source','velocity_source',
        'timing_clock','plan_compute_us','control_step_max_us','hold_compute_us',
        'timing_snapshot_event','accel_skew_ms','command_skew_ms']
    releases={e['interaction_id']:delta_ms(e['cf_timestamp_ms'],anchor) for e in curves if e['event']=='initial'}
    with open(base+'.comparison.csv','x',newline='') as output:
        writer=csv.DictWriter(output,fieldnames=fields);writer.writeheader()
        for s in groups.get('FIRMWARE_KIN',[]):
            t=s['relative_ms'];i=bisect.bisect_right(ct,t)-1
            event=curves[i] if i>=0 else None
            a=nearest('FIRMWARE_ACT',t);command=nearest('ATT_DES',t)
            row={k:s.get(k) for k in ('user_id','trial_id','run_id')}
            row.update(cf_relative_s=t*.001,recording_complete=complete)
            def put(keys,values):row.update(zip(keys,values or [None]*len(keys)))
            put(('vx','vy','vz'),s.get('velocity_m_s'));put(('ax','ay','az'),a.get('acceleration_world_m_s2') if a else None)
            put(('roll_deg','pitch_deg'),s.get('roll_pitch_deg'))
            put(('roll_rate_deg_s','pitch_rate_deg_s'),s.get('euler_roll_pitch_rate_deg_s'))
            put(('body_p_deg_s','body_q_legacy_deg_s'),s.get('body_rates_legacy_deg_s',[])[:2])
            put(('command_roll_deg','command_pitch_deg'),command.get('command_roll_pitch_deg') if command else None)
            if event:
                row.update(interaction_id=event['interaction_id'],plan_id=event['plan_id'])
                release=releases.get(event['interaction_id'])
                if release is not None:row['curve_start_relative_s']=(t-release)*.001
                put(('reconstructed_roll_deg','reconstructed_pitch_deg'),reconstruct_command(event,s['cf_timestamp_ms']))
                put(('reference_vx_m_s','reference_vy_m_s'),reconstruct_velocity(event,s['cf_timestamp_ms']))
                row['command_mode']=event.get('command_mode','attitude')
                for key in ('coefficient_units','attitude_source','velocity_source'):
                    row[key]=event.get(key)
                timing=event.get('timing',{})
                row.update(timing_clock=timing.get('clock'),timing_snapshot_event=event['event'])
                for key in ('plan_compute_us','control_step_max_us','hold_compute_us'):
                    row[key]=timing.get(key)
            row['accel_skew_ms']=a['relative_ms']-t if a else None
            row['command_skew_ms']=command['relative_ms']-t if command else None
            writer.writerow(row)
    # One row per event/axis/segment: coefficients are NOT mistaken for
    # physical-time powers. Duration and basis travel with every row.
    _export_coefficients(curves,base,complete,events_complete)
    return base+'.comparison.csv'


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('curves_jsonl');parser.add_argument('--allow-incomplete',action='store_true')
    parser.add_argument('--coefficients-only',action='store_true',help='Require complete curve events, independently of continuous-state gaps')
    args=parser.parse_args();output=export(args.curves_jsonl,allow_incomplete=args.allow_incomplete,coefficients_only=args.coefficients_only)
    print(output)
    if not args.coefficients_only:print(output.removesuffix('.comparison.csv')+'.coefficients.csv')
