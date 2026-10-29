"""Isolated localhost-only two-FLS170 Gazebo collision experiment.

Run with Interaction/run_virtual_collision_gazebo.sh. This is a simulation-only
outer tracker using Gazebo truth and the existing analytic S-curve generator.
It does not modify the production firmware S-curve or connect to hardware.
"""
import argparse, ast, hashlib, json, math, os, subprocess, sys, threading, time
import xml.etree.ElementTree as ET
from pathlib import Path

LAUNCH = Path('/workspace/crazyflie-firmware/tools/crazyflie-simulation/simulator_files/gazebo/launch')
sys.path[:0] = [str(LAUNCH), '/offboard']
gz = None
from dataclasses import asdict, dataclass, replace
from Interaction.virtual_collision_sim import CollisionConfig, plan_collision


@dataclass(frozen=True)
class CollisionExperimentOptions:
    """Explicit simulation experiments; baseline defaults and accuracy gates stay fixed."""
    reference_acceleration_m_s2: float = 2.0
    reference_jerk_m_s3: float = 8.0
    trigger_reference: str | None = None
    trigger_reserve_m: float = .06
    geometry_close_pass: bool = False
    jerk_feedforward_lead_s: float = .10
    command_acceleration_cap_m_s2: float = 3.0
    attitude_bias_compensation: bool = False
    predictive_unwind: bool = False
    unwind_impulse_time_s: float = .262
    unwind_velocity_gain: float = 5.0
    unwind_rate_damping_s: float = .20
    unwind_tilt_jerk_weight_s2: float = .0137
    minimum_surface_gap_m: float | None = None

    @property
    def resolved_minimum_surface_gap_m(self):
        """Conservative CAD-sphere clearance floor; not a measured-gap target."""
        return .03 if self.minimum_surface_gap_m is None else self.minimum_surface_gap_m

    def validate(self):
        values=(self.reference_acceleration_m_s2,self.reference_jerk_m_s3,self.trigger_reserve_m,self.jerk_feedforward_lead_s,self.command_acceleration_cap_m_s2,self.unwind_impulse_time_s,self.unwind_velocity_gain,self.unwind_rate_damping_s,self.unwind_tilt_jerk_weight_s2)
        if not all(math.isfinite(x) for x in values):raise ValueError('experiment options must be finite')
        if not 0<self.reference_acceleration_m_s2<=8:raise ValueError('reference acceleration must be in (0,8] m/s^2')
        if not 0<self.reference_jerk_m_s3<=80:raise ValueError('reference jerk must be in (0,80] m/s^3')
        if not 0<self.command_acceleration_cap_m_s2<=8:raise ValueError('collision command acceleration cap must be in (0,8] m/s^2')
        if self.reference_acceleration_m_s2>self.command_acceleration_cap_m_s2:raise ValueError('reference acceleration exceeds declared command cap')
        if not .10<=self.unwind_impulse_time_s<=.35 or not 1<=self.unwind_velocity_gain<=15 or not .03<=self.unwind_rate_damping_s<=.25 or not 0<=self.unwind_tilt_jerk_weight_s2<=.02:raise ValueError('predictive unwind options outside reviewed experimental domain')
        if not .04<=self.trigger_reserve_m<=.30:raise ValueError('empirical trigger reserve must be .04 to .30 m')
        if not .05<=self.jerk_feedforward_lead_s<=.35:raise ValueError('jerk feedforward lead must be .05 to .35 s in this simulation experiment')
        if self.minimum_surface_gap_m is not None:
            if not self.geometry_close_pass:raise ValueError('--minimum-surface-gap requires --geometry-close-pass')
            if not math.isfinite(self.minimum_surface_gap_m) or not 0<self.minimum_surface_gap_m<=.50:
                raise ValueError('minimum surface gap must be finite and in (0,0.50] m')
        if self.geometry_close_pass and self.trigger_reference is None:raise ValueError('CAD close pass requires an accepted empirical trigger reference')


def make_cad_safety_geometry(options, vehicle_bounds):
    """Build the same declared geometry floor used by planning and live/raw checks."""
    options.validate()
    if not options.geometry_close_pass:raise ValueError('CAD safety geometry requires close-pass mode')
    if not vehicle_bounds:raise ValueError('CAD safety geometry requires measured vehicle bounds')
    radii=[bound['bounding_sphere_radius_m'] for bound in vehicle_bounds]
    if not all(math.isfinite(radius) and radius>0 for radius in radii):raise ValueError('invalid CAD bounding sphere radius')
    radius=max(radii);surface_gap=options.resolved_minimum_surface_gap_m
    return dict(mode='cad_bounded_close_pass',minimum_center_separation_m=2*radius+surface_gap,
        minimum_surface_margin_m=surface_gap,requested_minimum_surface_gap_m=surface_gap,
        minimum_surface_gap_was_explicit=options.minimum_surface_gap_m is not None,
        surface_gap_semantics='Minimum conservative bounding-sphere clearance, not a guaranteed target for observed CAD surface distance.',
        bounding_sphere_radius_m=radius,vehicle_bounds=vehicle_bounds,
        scope='New simulation-only geometry scenario. Original .40 m baseline remains unchanged.')


def cad_surface_clearance_checks(sampled_clearance_m, adjusted_clearance_m, required_margin_m):
    """Apply one declared margin to both postprocessing surface-clearance checks."""
    if not math.isfinite(required_margin_m) or not 0<required_margin_m<=.50:raise ValueError('invalid required surface margin')
    return dict(cad_surface_clearance=math.isfinite(sampled_clearance_m) and sampled_clearance_m>=required_margin_m,
        cad_clearance_with_allowance=math.isfinite(adjusted_clearance_m) and adjusted_clearance_m>=required_margin_m)


def derive_geometry_guard(model_path, simulator_root):
    """Bound the newly generated body and arbitrary prop spin about model origin."""
    import numpy as np
    model_path=Path(model_path);model=ET.parse(model_path).find('model')
    visuals=model.findall('./link/visual')
    if {v.get('name') for v in visuals}!={'body_visual','m1_visual','m2_visual','m3_visual','m4_visual'}:
        raise ValueError('unreviewed visual geometry in CAD close-pass model')
    def xyz(node):return np.array([float(x) for x in node.findtext('pose','0 0 0 0 0 0').split()[:3]])
    hashes={model_path.name:hashlib.sha256(model_path.read_bytes()).hexdigest()};prop_bounds=[];body_radius=None
    ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
    for link in model.findall('link'):
        for visual in link.findall('visual'):
            mesh=visual.find('geometry/mesh')
            if mesh is None:raise ValueError('close-pass visual must be a reviewed mesh')
            scale=np.array([float(x) for x in mesh.findtext('scale','1 1 1').split()])
            if scale.shape!=(3,) or not np.isfinite(scale).all():raise ValueError('invalid visual scale')
            uri=mesh.findtext('uri');filename=Path(uri).name
            if visual.get('name')=='body_visual':
                path=model_path.with_name(filename);squared=[]
                for line in path.read_text().splitlines():
                    if line.startswith('v '):
                        point=np.array([float(x) for x in line.split()[1:4]])*scale
                        if not np.isfinite(point).all():raise ValueError('nonfinite CAD vertex')
                        squared.append(float(point@point))
                if not squared:raise ValueError('CAD body has no vertices')
                body_radius=float(np.linalg.norm(xyz(link))+np.linalg.norm(xyz(visual))+math.sqrt(max(squared)))
            else:
                if filename not in ('cw_prop.dae','ccw_prop.dae'):raise ValueError('unknown prop mesh')
                path=Path(simulator_root)/'meshes/collada_files'/filename;doc=ET.parse(path)
                nodes=doc.findall('.//c:visual_scene/c:node',ns)
                if len(nodes)!=1 or nodes[0].find('c:node',ns) is not None:raise ValueError('unreviewed prop hierarchy')
                node=nodes[0]
                if any(node.find('c:'+tag,ns) is not None for tag in ('translate','rotate','scale','lookat','skew')):
                    raise ValueError('unreviewed prop transform')
                matrix=np.array([float(x) for x in node.findtext('c:matrix',namespaces=ns).split()]).reshape(4,4)
                if not np.allclose(matrix[3],[0,0,0,1]):raise ValueError('nonaffine prop matrix')
                instances=node.findall('c:instance_geometry',ns)
                if len(instances)!=1:raise ValueError('unreviewed prop geometry count')
                gid=instances[0].get('url').lstrip('#');geom=doc.find(f'.//c:geometry[@id="{gid}"]',ns)
                source=geom.find('c:mesh/c:vertices/c:input[@semantic="POSITION"]',ns).get('source').lstrip('#')
                source=geom.find(f'c:mesh/c:source[@id="{source}"]',ns)
                accessor=source.find('c:technique_common/c:accessor',ns)
                stride=int(accessor.get('stride','3'));offset=int(accessor.get('offset','0'));count=int(accessor.get('count'))
                values=np.array([float(x) for x in source.findtext('c:float_array',namespaces=ns).split()])
                points=values[offset:offset+count*stride].reshape(count,stride)[:,:3]
                unit=doc.find('c:asset/c:unit',ns);unit=float(unit.get('meter','1')) if unit is not None else 1.
                points=(points@matrix[:3,:3].T+matrix[:3,3])*unit
                if not np.isfinite(points).all():raise ValueError('nonfinite prop geometry')
                mesh_radius=float(np.linalg.norm(points,axis=1).max())*float(np.abs(scale).max())
                prop_bounds.append(float(np.linalg.norm(xyz(link))+np.linalg.norm(xyz(visual))+mesh_radius))
            hashes[filename]=hashlib.sha256(path.read_bytes()).hexdigest()
    if body_radius is None or len(prop_bounds)!=4:raise ValueError('incomplete model geometry proof')
    radius=max(body_radius,*prop_bounds)
    if not .08<radius<.20:raise ValueError('derived geometry outside reviewed FLS170 domain')
    return dict(body_cad_radius_m=body_radius,animated_prop_radius_bounds_m=prop_bounds,
        bounding_sphere_radius_m=radius,source_sha256=hashes,
        annotation_removed='contact_side_orientation_marker',
        bound_method='Body vertex norm and triangle-inequality prop-link radius; prop mesh affine transform included; arbitrary spin and vehicle attitude preserve sphere bounds')


def load_trigger_reference(directory):
    """Measure forward closure from a prior physics trace, never ideal playback.

    This empirical predictor is a simulation hypothesis, not a safety proof.
    The current run still enforces its declared geometry live/raw separation gates.
    """
    if directory is None:return None
    base=Path(directory);source=base/'report.json';previous=json.loads(source.read_text())
    if previous.get('status')!='collision_complete':raise ValueError('trigger reference needs a completed physical simulation')
    cfg=previous['plan_config']
    if cfg['mass1_kg']!=.17 or cfg['mass2_kg']!=.17 or cfg['restitution']!=1.:
        raise ValueError('trigger reference must be the equal-mass FLS170 elastic case')
    incoming=previous.get('raw_odom_incoming_velocities_m_s',[cfg['velocity1_m_s'],cfg['velocity2_m_s']])
    relative=incoming[0]-incoming[1]
    if not math.isfinite(relative) or relative<=0:raise ValueError('invalid trigger reference velocity')
    start=round(previous['trigger_sim_time_s']*1e9);end=round(previous['capture_sim_time_s']*1e9)
    paths=[base/f'odom_{i}.jsonl' for i in (0,1)];positions=[]
    for path in paths:
        samples={}
        for line in path.read_text().splitlines():
            try:row=json.loads(line)
            except json.JSONDecodeError:continue
            if 'pose' not in row:continue
            stamp=row['header']['stamp'];ns=int(stamp.get('sec',0))*10**9+int(stamp.get('nsec',0))
            if start<=ns<=end:
                point=[float(row['pose']['position'].get(a,0)) for a in 'xyz']
                if not all(math.isfinite(x) for x in point):raise ValueError('nonfinite trigger reference pose')
                samples[ns]=point
        positions.append(samples)
    common=sorted(set(positions[0])&set(positions[1]))
    if len(common)<10 or common[0]-start>10_000_000:raise ValueError('trigger reference lacks aligned onset data')
    gaps=[positions[1][s][0]-positions[0][s][0] for s in common]
    closure=gaps[0]-min(gaps)
    if not math.isfinite(closure) or not 0<closure<2:raise ValueError('trigger reference closure outside domain')
    return dict(source_directory=str(base),source_sha256={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in [source,*paths]},
        acceptance_passed=previous.get('collision_passed') is True,
        incoming_relative_velocity_m_s=relative,measured_projected_closure_m=closure,
        reference_acceleration_m_s2=cfg['acceleration_limit_m_s2'],reference_jerk_m_s3=cfg['jerk_limit_m_s3'],
        tracker_gains=previous.get('outer_tracker_gains'),common_timestamp_count=len(common))


def select_trigger_distance(plan, empirical, reserve_m):
    """Use measured closure plus a fixed reserve, with conservative speed scaling."""
    if empirical is None:return plan.trigger_distance_m,dict(mode='analytic_baseline')
    relative=plan.config.velocity1_m_s-plan.config.velocity2_m_s
    ratio=relative/empirical['incoming_relative_velocity_m_s']
    if not .8<=ratio<=1.25:
        return plan.trigger_distance_m,dict(mode='analytic_outside_empirical_speed_range',speed_ratio=ratio)
    closure=empirical['measured_projected_closure_m']*max(ratio,ratio*ratio)
    closure=max(closure,plan.reference_closure_m)
    distance=plan.config.minimum_separation_m+closure+reserve_m
    return distance,dict(mode='measured_closure_with_reserve',predicted_closure_m=closure,
        speed_ratio=ratio,reserve_m=reserve_m,discrete_trigger_advance_m=.02,
        guarantee='empirical simulation predictor; current physical separation still checked')


def collision_experiment(connections, latest, rows, report, options, empirical, firmware_latest):
    """Ground-truth outer tracker; real simulated attitude PID/motors/physics."""
    cfg=CollisionConfig(mass1_kg=.17,mass2_kg=.17,initial_separation_m=3.2,
        acceleration_limit_m_s2=options.reference_acceleration_m_s2,jerk_limit_m_s3=options.reference_jerk_m_s3,
        minimum_separation_m=report['safety_geometry']['minimum_center_separation_m'],
        drone_radius_m=report['safety_geometry']['bounding_sphere_radius_m'])
    position_gain=2.;velocity_gain=3.
    report['outer_tracker_gains']=dict(position_approach_capture=position_gain,position_collision=0.,velocity_approach_capture=velocity_gain,velocity_collision=5.,jerk_feedforward_lead_s=options.jerk_feedforward_lead_s,tilt_rate_damping_s=.10,tilt_rate_filter_tau_s=.02)
    nominal=plan_collision(cfg)
    start=min(latest[i]['sim_time_ns'] for i in (0,1))/1e9
    report['approach_sim_time_s']=start
    initial=[list(latest[i]['position_m']) for i in (0,1)]
    last=-1;trigger=None;plan=None;offset=None;tail_finished=None;capture=None
    experiment=[];deadline=time.monotonic()+70;previous_truth=None;filtered_tilt_jerk=[0.,0.]
    filtered_pitch_bias=[0.,0.];unwind_started=None
    def tilt_accel(row):
        x,y,z,w=row['quaternion_xyzw']
        return 9.81*2*(x*z+w*y)/(1-2*(x*x+y*y))
    def tilt_angle(row):
        x,y,z,w=row['quaternion_xyzw']
        return math.degrees(math.acos(max(-1.,min(1.,1-2*(x*x+y*y)))))
    while True:
        if time.monotonic()>deadline:raise TimeoutError('collision scenario sim time stalled')
        truth=[dict(latest[i]) for i in (0,1)]
        sim=min(t['sim_time_ns'] for t in truth)/1e9
        if sim-last<.01-1e-6:time.sleep(.002);continue
        last=sim;t=sim-start
        if previous_truth is not None:
            for i in (0,1):
                step=(truth[i]['sim_time_ns']-previous_truth[i]['sim_time_ns'])/1e9
                if step>0:
                    observed=(tilt_accel(truth[i])-tilt_accel(previous_truth[i]))/step
                    filtered_tilt_jerk[i]+=step/(.02+step)*(observed-filtered_tilt_jerk[i])
        previous_truth=truth
        diagnostics=[firmware_latest.get(i) for i in (0,1)]
        bias_ages=[]
        for i,diagnostic in enumerate(diagnostics):
            age=None if diagnostic is None or diagnostic['approximate_truth_receipt_ns'] is None else (truth[i]['sim_time_ns']-diagnostic['approximate_truth_receipt_ns'])/1e9
            bias_ages.append(age)
            if options.attitude_bias_compensation:
                if age is None or not -.01<=age<=.05:raise RuntimeError('ground-truth bias telemetry stale or absent')
                raw_bias=diagnostic['values']['stabilizer.pitch']-diagnostic['truth_cf_pitch_deg_at_receipt']
                if abs(raw_bias)>10:raise RuntimeError('estimated-versus-true pitch discrepancy outside simulation observer domain')
                filtered_pitch_bias[i]+=.01/(.15+.01)*(raw_bias-filtered_pitch_bias[i])
                filtered_pitch_bias[i]=max(-3.,min(3.,filtered_pitch_bias[i]))
        gap=math.dist(truth[0]['position_m'],truth[1]['position_m'])
        if gap<cfg.minimum_separation_m:
            report.update(safety_abort=f'center separation below scenario floor {cfg.minimum_separation_m:.9f} m',minimum_separation_m=gap)
            raise RuntimeError('scenario geometric separation gate failed')
        if any(abs(x['position_m'][0])>4 or abs(x['position_m'][1])>.3 or x['position_m'][2]<.45 or x['position_m'][2]>1.2 for x in truth):
            raise RuntimeError('simulation position envelope failed')
        if any(tilt_angle(x)>45. for x in truth):raise RuntimeError('simulation actual tilt exceeded declared 45 degree envelope')
        if trigger is None:
            vel=[x['velocity_m_s'][0] for x in truth]
            if vel[0]>vel[1]+.05:
                candidate=plan_collision(replace(cfg,velocity1_m_s=vel[0],velocity2_m_s=vel[1]))
                trigger_distance,trigger_details=select_trigger_distance(candidate,empirical,options.trigger_reserve_m)
                if gap<=trigger_distance+.02:
                    trigger=sim;plan=candidate;first=plan.sample(0)
                    offset=[truth[i]['position_m'][0]-first[f'x{i+1}_m'] for i in (0,1)]
                    report.update(trigger_sim_time_s=sim,trigger_distance_actual_m=gap,plan_config=asdict(plan.config),
                        plan_duration_s=plan.duration_s,trigger_distance_planned_m=plan.trigger_distance_m,
                        trigger_distance_selected_m=trigger_distance,trigger_prediction=trigger_details,
                        target_velocities_m_s=[plan.target_velocity1_m_s,plan.target_velocity2_m_s])
        reference=[]
        if trigger is None:
            ramp=1.2;s=min(t/ramp,1)
            speed=.6*(3*s*s-2*s*s*s)
            accel=.6/ramp*(6*s-6*s*s) if t<ramp else 0.
            distance=.6*ramp*(s**3-.5*s**4)+.6*max(0,t-ramp)
            for i,sign in enumerate((1,-1)):
                reference.append(dict(x=initial[i][0]+sign*distance,v=sign*speed,a=sign*accel,j=0.))
            phase='approach'
            if t>8:raise RuntimeError('approach failed to reach collision trigger')
        elif sim-trigger < plan.duration_s+1.0:
            ref=plan.sample(sim-trigger)
            reference=[dict(x=ref[f'x{i+1}_m']+offset[i],v=ref[f'v{i+1}_m_s'],a=ref[f'a{i+1}_m_s2'],j=ref[f'j{i+1}_m_s3']) for i in (0,1)]
            phase='transition' if sim-trigger<plan.duration_s else 'separation'
        else:
            if tail_finished is None:
                tail_finished=sim
                capture=[x['position_m'][0]+.12*x['velocity_m_s'][0] for x in truth]
                outgoing=[x['velocity_m_s'][0] for x in truth]
                report.update(outgoing_velocities_m_s=outgoing,terminal_velocity_errors_m_s=[abs(outgoing[i]-[plan.target_velocity1_m_s,plan.target_velocity2_m_s][i]) for i in (0,1)])
            reference=[dict(x=capture[i],v=0.,a=0.,j=0.) for i in (0,1)];phase='capture'
        commands=[]
        predictions=[truth[i]['velocity_m_s'][0]+options.unwind_impulse_time_s*tilt_accel(truth[i])+options.unwind_tilt_jerk_weight_s2*filtered_tilt_jerk[i] for i in (0,1)]
        if options.predictive_unwind and trigger is not None and unwind_started is None and predictions[0]-predictions[1]<=plan.target_velocity1_m_s-plan.target_velocity2_m_s:
            unwind_started=sim
            report['predictive_unwind']=dict(start_after_trigger_s=sim-trigger,predicted_velocities_m_s=predictions,
                model='v_end=v+configured_tau*g_tilt+configured_beta*d(g_tilt)/dt; fitted delayed-plant hypothesis, not guarantee',
                impulse_time_s=options.unwind_impulse_time_s,tilt_jerk_weight_s2=options.unwind_tilt_jerk_weight_s2,
                blend_duration_s=.04,terminal_velocity_gain=options.unwind_velocity_gain,terminal_tilt_rate_damping_s=options.unwind_rate_damping_s)
        for i,scf in enumerate(connections):
            p=truth[i]['position_m'];v=truth[i]['velocity_m_s'];r=reference[i]
            x_position_gain=0. if phase in ('transition','separation') else position_gain
            x_velocity_gain=5. if phase in ('transition','separation') else velocity_gain
            ax=r['a']+x_position_gain*(r['x']-p[0])+x_velocity_gain*(r['v']-v[0])
            if phase in ('transition','separation'):ax+=options.jerk_feedforward_lead_s*r['j']-.10*filtered_tilt_jerk[i]
            if unwind_started is not None and phase in ('transition','separation'):
                s=max(0.,min(1.,(sim-unwind_started)/.04));blend=35*s**4-84*s**5+70*s**6-20*s**7
                target=[plan.target_velocity1_m_s,plan.target_velocity2_m_s][i]
                terminal=options.unwind_velocity_gain*(target-predictions[i])-options.unwind_rate_damping_s*filtered_tilt_jerk[i]
                ax=(1-blend)*ax+blend*terminal
            ay=position_gain*(initial[i][1]-p[1])-velocity_gain*v[1]
            precompensation_ax=ax;precompensation_ay=ay
            command_cap=options.command_acceleration_cap_m_s2 if phase in ('transition','separation') else 3.
            yaw=math.radians(gz._world_yaw_deg(truth[i]['quaternion_xyzw']))
            body_ax=ax*math.cos(yaw)+ay*math.sin(yaw)
            body_ay=-ax*math.sin(yaw)+ay*math.cos(yaw)
            applied_bias=filtered_pitch_bias[i] if options.attitude_bias_compensation and phase in ('transition','separation') else 0.
            body_ax=-9.81*math.tan(-math.atan2(body_ax,9.81)+math.radians(applied_bias))
            mag=math.hypot(body_ax,body_ay)
            if mag>command_cap:body_ax*=command_cap/mag;body_ay*=command_cap/mag
            ax=body_ax*math.cos(yaw)-body_ay*math.sin(yaw);ay=body_ax*math.sin(yaw)+body_ay*math.cos(yaw)
            # This firmware's PID uses negative roll for positive world Y.
            roll=-math.degrees(math.atan2(body_ay,9.81));pitch=-math.degrees(math.atan2(body_ax,9.81))
            command_tilt=math.degrees(math.acos(math.cos(math.radians(roll))*math.cos(math.radians(pitch))))
            if command_tilt>41.:raise RuntimeError('simulation command tilt exceeded declared 41 degree envelope')
            scf.cf.commander.send_zdistance_setpoint(roll,pitch,0,.8)
            commands.append(dict(ax_m_s2=ax,ay_m_s2=ay,precompensation_ax_m_s2=precompensation_ax,precompensation_ay_m_s2=precompensation_ay,roll_deg=roll,pitch_deg=pitch,tilt_deg=command_tilt,command_acceleration_cap_m_s2=command_cap,actual_tilt_deg=tilt_angle(truth[i]),filtered_tilt_jerk_m_s3=filtered_tilt_jerk[i],applied_pitch_bias_deg=applied_bias,bias_telemetry_age_s=bias_ages[i],predicted_terminal_velocity_m_s=predictions[i],predictive_unwind_active=unwind_started is not None))
        row=dict(elapsed_s=t,sim_time_s=sim,collision_elapsed_s=None if trigger is None else sim-trigger,phase=phase,drones=truth,reference=reference,commands=commands,separation_m=gap)
        experiment.append(row);rows.append(row)
        if tail_finished is not None and sim-tail_finished>=3:break
    active=[r for r in experiment if r['phase'] in ('transition','separation')]
    report['minimum_separation_m']=min(r['separation_m'] for r in experiment)
    actual_cfg=plan.config
    incoming_momentum=actual_cfg.mass1_kg*actual_cfg.velocity1_m_s+actual_cfg.mass2_kg*actual_cfg.velocity2_m_s
    report['incoming_momentum_kg_m_s']=incoming_momentum
    report['collision_momentum_peak_error_kg_m_s']=max(abs(actual_cfg.mass1_kg*r['drones'][0]['velocity_m_s'][0]+actual_cfg.mass2_kg*r['drones'][1]['velocity_m_s'][0]-incoming_momentum) for r in active)
    report['endpoint_momentum_error_kg_m_s']=actual_cfg.mass1_kg*report['outgoing_velocities_m_s'][0]+actual_cfg.mass2_kg*report['outgoing_velocities_m_s'][1]-incoming_momentum
    report['measured_restitution']=-(report['outgoing_velocities_m_s'][0]-report['outgoing_velocities_m_s'][1])/(actual_cfg.velocity1_m_s-actual_cfg.velocity2_m_s)
    report['max_state_pair_skew_s']=max(abs(r['drones'][0]['sim_time_ns']-r['drones'][1]['sim_time_ns'])/1e9 for r in experiment)
    report['max_command_sample_interval_s']=max(b['sim_time_s']-a['sim_time_s'] for a,b in zip(experiment,experiment[1:]))
    report['maximum_command_acceleration_m_s2']=max(math.hypot(c['ax_m_s2'],c['ay_m_s2']) for r in active for c in r['commands'])
    report['checks']=dict(separation=report['minimum_separation_m']>=cfg.minimum_separation_m,terminal_velocity=max(report['terminal_velocity_errors_m_s'])<=.10,
        transient_momentum=report['collision_momentum_peak_error_kg_m_s']<=.01,endpoint_momentum=abs(report['endpoint_momentum_error_kg_m_s'])<=.01,
        restitution=abs(report['measured_restitution']-actual_cfg.restitution)<=.10)
    report['collision_passed']=all(report['checks'].values())
    report['tracking_scope']='S-curve acceleration plus bounded PD from Gazebo truth; real frozen SITL attitude PID, altitude PID, mixer, motors and rigid-body physics. Not production onboard S-curve execution.'
    report['attitude_bias_compensation_scope']='Optional privileged simulation correction: CF estimated pitch minus quaternion-derived true CF pitch; receipt-aligned telemetry, .15s filter, +/-3deg bound, <=50ms freshness; final angles recapped after correction.'
    report['command_cap_scope']='Declared cap bounds collision-phase host equivalent-horizontal-acceleration command; approach/capture retain baseline3m/s². Actual acceleration is determined by attitude response, thrust and rigid-body dynamics and can exceed the command cap.'
    report['physical_envelope']=dict(command_acceleration_cap_m_s2=options.command_acceleration_cap_m_s2,
        commanded_tilt_limit_deg=41.,actual_tilt_abort_deg=45.,altitude_limits_m=[.45,1.2],
        maximum_commanded_tilt_deg=max(c['tilt_deg'] for r in active for c in r['commands']),
        maximum_actual_tilt_deg=max(c['actual_tilt_deg'] for r in experiment for c in r['commands']),
        altitude_minimum_m=min(t['position_m'][2] for r in experiment for t in r['drones']),
        altitude_maximum_m=max(t['position_m'][2] for r in experiment for t in r['drones']),
        static_level_hold_thrust_at_cap_N=.17*math.hypot(9.81,options.command_acceleration_cap_m_s2),
        frozen_plugin_full_pwm_thrust_ceiling_N=2.87747,frozen_z_pid_collective_ceiling_N=2.66812,
        thrust_scope='Static modeled envelope from loaded strict_sim PWM2OMEGA and frozen Z PID; no guarantee of transient mixer or attitude authority.')
    report['capture_sim_time_s']=tail_finished
    report['encounter_end_sim_time_s']=sim
    return experiment


def run(output, collision=False, options=CollisionExperimentOptions()):
    global gz
    if not Path('/.dockerenv').exists():
        raise RuntimeError('Container-only simulation; no hardware URI accepted')
    import contact_attitude_ekf_capture as gz
    options.validate();empirical=load_trigger_reference(options.trigger_reference)
    if options.geometry_close_pass and not empirical['acceptance_passed']:raise ValueError('CAD close pass requires a previously accepted physical simulation')
    output = Path(output); output.mkdir(parents=True, exist_ok=False)
    env = dict(os.environ, GZ_SIM_SYSTEM_PLUGIN_PATH='/workspace/.codex-fw-shadow-sitl/sitl_make/build-auto-170/build_crazysim_gz',
               GZ_SIM_RESOURCE_PATH=str(gz.GAZEBO_ROOT/'models')+':'+str(gz.GAZEBO_ROOT/'worlds'),
               LIBGL_ALWAYS_SOFTWARE='1', CF2_SIM_MODEL='gz_crazyflie')
    processes, streams, connections, workers, errors = [], [], [], [], []
    stop = threading.Event(); paths=[]; latest={}; events=[]; rows=[]; firmware_rows=[]; log_configs=[];firmware_latest={}
    report = {'scope':'two FLS170 Gazebo plants, frozen SITL PID/KF; not hardware or current onboard S-curve execution', 'mass_kg_each':.170, 'status':'starting'}
    report['experiment_options']=asdict(options);report['trigger_reference']=empirical
    report['safety_geometry']=dict(mode='baseline_040',minimum_center_separation_m=.4,bounding_sphere_radius_m=.12)
    vehicle_bounds=[]
    report['source_sha256']={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in (Path(__file__),Path('/offboard/Interaction/virtual_collision_sim.py'),Path('/offboard/Interaction/jerk_limited_braking.py'),Path('/master-sitl/sitl_make/build-selection/cf2'))}
    (output/'harness_used.py').write_text(Path(__file__).read_text())
    (output/'collision_profile_used.py').write_text(Path('/offboard/Interaction/virtual_collision_sim.py').read_text())
    try:
        tree=gz.ET.parse(gz.GAZEBO_ROOT/'worlds/contact_brake_shadow.sdf')
        world=tree.find('world')
        for plugin in list(world.findall('plugin')):
            if plugin.get('filename')=='gz_contact_wrench_scheduler':world.remove(plugin)
        world.find('physics/real_time_factor').text='1'
        tree.write(output/'world.sdf',encoding='unicode',xml_declaration=True)
        stream=(output/'gazebo.log').open('w');streams.append(stream)
        processes.append(subprocess.Popen(['stdbuf','-oL','-eL','gz','sim','-s',str(output/'world.sdf'),'-v','3'],env=env,stdout=stream,stderr=subprocess.STDOUT))
        gz.wait_for_service('/world/contact_brake_shadow/create',env)
        for i,x in enumerate((-1.6,1.6)):
            model=output/f'model_{i}.sdf'; firmport=19950+i; libport=19850+i
            gz.run_checked([sys.executable,str(LAUNCH/'jinja_gen.py'),str(gz.GAZEBO_ROOT/'models/crazyflie/model.sdf.jinja'),str(gz.GAZEBO_ROOT),
                '--cf_id',str(i),'--cffirm_udp_port',str(firmport),'--cflib_udp_port',str(libport),'--cf_name','cf',
                '--external-pose-mode','position_only','--airframe-profile','fls170','--fls-asset-root','/fls-assets','--output-file',str(model)],env=env)
            if options.geometry_close_pass:
                model_tree=ET.parse(model)
                for link in model_tree.findall('.//link'):
                    for visual in list(link.findall('visual')):
                        if visual.get('name')=='contact_side_orientation_marker':link.remove(visual)
                model_tree.write(model,encoding='unicode',xml_declaration=True)
                vehicle_bounds.append(derive_geometry_guard(model,LAUNCH.parents[2]))
            gz.run_checked(['gz','service','-s','/world/contact_brake_shadow/create','--reqtype','gz.msgs.EntityFactory','--reptype','gz.msgs.Boolean','--timeout','10000',
                '--req',f'sdf_filename: "{model}", name: "crazyflie_{i}", pose: {{position: {{x: {x}, z: 0.5}}}}'],env=env)
            path=output/f'odom_{i}.jsonl';paths.append(path)
            process,stream=gz.start_topic_capture(f'/cf_{i}/odom',path);processes.append(process);streams.append(stream)
            stream=(output/f'firmware_{i}.log').open('w');streams.append(stream)
            processes.append(subprocess.Popen(['stdbuf','-oL','-eL','/master-sitl/sitl_make/build-selection/cf2',str(firmport)],env=env,stdout=stream,stderr=subprocess.STDOUT))
            gz.wait_for_text(output/f'firmware_{i}.log','Connection established with gazebo',12)
            gz.probe_crtp(libport)
        if options.geometry_close_pass:
            report['safety_geometry']=make_cad_safety_geometry(options,vehicle_bounds)
        gz.world_control('contact_brake_shadow','pause: false',env)
        for i,path in enumerate(paths):
            gz.wait_for_text(output/f'firmware_{i}.log','Starting stabilizer loop',15)
            gz.wait_for_stationary_ground(path)
        import cflib.crtp
        from cflib.crazyflie import Crazyflie
        from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
        from cflib.crazyflie.log import LogConfig
        cflib.crtp.init_drivers()
        ctree=ast.parse(Path('/offboard/config.py').read_text())
        pid=ast.literal_eval(next(n.value for n in ctree.body if isinstance(n,ast.Assign) and any(isinstance(t,ast.Name) and t.id=='PID_VALUES' for t in n.targets)))
        for i,path in enumerate(paths):
            scf=SyncCrazyflie(f'udp://127.0.0.1:{19850+i}',cf=Crazyflie(rw_cache=str(output/f'cache_{i}')));scf.open_link();connections.append(scf);cf=scf.cf
            deadline=time.monotonic()+15
            while not cf.param.is_updated:
                if time.monotonic()>deadline:raise TimeoutError('parameter snapshot unavailable')
                time.sleep(.05)
            for key,value in {'commander.enHighLevel':1,'stabilizer.estimator':2,'stabilizer.controller':1,'locSrv.extPosStdDev':.001, **pid}.items():cf.param.set_value(key,str(value))
            for key in ('hlCommander.pRelAuto','kalmanPRel.enable','kalmanPRel.scEnable'):
                group,name=key.split('.')
                if name in cf.param.toc.toc.get(group,{}):cf.param.set_value(key,'0')
            def forward(index=i,client=cf,source=path):
                previous=None
                while not stop.is_set():
                    try:
                        truth=gz.latest_odom(source);latest[index]=truth
                        if truth['sim_time_ns']!=previous:client.extpos.send_extpos(*truth['position_m']);previous=truth['sim_time_ns']
                    except (ValueError,OSError):pass
                    except BaseException as e:errors.append(str(e));return
                    stop.wait(.003)
            worker=threading.Thread(target=forward,daemon=True);workers.append(worker);worker.start()
            variables=['stabilizer.roll','stabilizer.pitch','controller.cmd_pitch','stabilizer.thrust','pid_attitude.pitch_outI','pid_rate.pitch_outI']
            # Frozen firmware multiplies legacy period byte by1, cflib divides request by10.
            config=LogConfig(name='CollisionDiag',period_in_ms=200)
            available=[name for name in variables if cf.log.toc.get_element_by_complete_name(name) is not None]
            for name in available:config.add_variable(name)
            if available:
                cf.log.add_config(config)
                def log_data(timestamp,data,logconf,index=i):
                    current=latest.get(index,{})
                    q=current.get('quaternion_xyzw',[0.,0.,0.,1.]);x,y,z,w=q
                    diagnostic=dict(drone=index,firmware_timestamp_ms=timestamp,
                        approximate_truth_receipt_ns=current.get('sim_time_ns'),values=data,
                        truth_cf_pitch_deg_at_receipt=-math.degrees(math.asin(max(-1.,min(1.,2*(w*y-z*x))))))
                    firmware_rows.append(diagnostic)
                    if 'stabilizer.pitch' in data:firmware_latest[index]=diagnostic
                config.data_received_cb.add_callback(log_data);config.start();log_configs.append(config)
            report.setdefault('firmware_diagnostics_variables',[]).append(available)
            motors=LogConfig(name='CollisionMotors',period_in_ms=200)
            motor_variables=[name for name in ['controller.pitch','controller.pitchRate','motor.m1','motor.m2','motor.m3','motor.m4'] if cf.log.toc.get_element_by_complete_name(name) is not None]
            for name in motor_variables:motors.add_variable(name)
            if motor_variables:
                cf.log.add_config(motors);motors.data_received_cb.add_callback(log_data);motors.start();log_configs.append(motors)
            report.setdefault('firmware_motor_diagnostics_variables',[]).append(motor_variables)
        report['firmware_diagnostics_timing']=dict(cflib_requested_period_ms=200,legacy_wire_period=20,
            frozen_firmware_effective_period_ms=20,note='Frozen firmware legacy logStartBlock multiplies period byte by1 while cflib divides requested ms by10. Firmware clock is not Gazebo sim clock; receipt-aligned truth timestamp used, motion metrics use raw Gazebo time.')
        time.sleep(1)
        for scf in connections:scf.cf.platform.send_arming_request(True)
        time.sleep(.5)
        for scf in connections:scf.cf.high_level_commander.takeoff(.8,2.5)
        start=min(latest[i]['sim_time_ns'] for i in (0,1))/1e9
        deadline=time.monotonic()+45
        while min(latest[i]['sim_time_ns'] for i in (0,1))/1e9-start < 7:
            if time.monotonic()>deadline:raise TimeoutError('hover sim time stalled')
            rows.append({'elapsed_s':min(latest[i]['sim_time_ns'] for i in (0,1))/1e9-start,'drones':[dict(latest[i]) for i in (0,1)]})
            time.sleep(.02)
        report.update(status='hover_complete',hover_latest=[latest[i] for i in (0,1)],hover_passed=all(abs(latest[i]['position_m'][2]-.8)<.10 and math.dist(latest[i]['velocity_m_s'],[0,0,0])<.15 for i in (0,1)))
        if not report['hover_passed']:raise RuntimeError('paired hover gate failed')
        if collision:
            collision_experiment(connections,latest,rows,report,options,empirical,firmware_latest)
            from Interaction.commander_handoff import handoff_to_high_level
            for scf in connections:handoff_to_high_level(scf.cf.commander,scf.cf.high_level_commander,'land',.03,2.5)
            report['status']='collision_complete'
        else:
            for scf in connections:scf.cf.high_level_commander.land(.03,2.5)
        time.sleep(4)
        for scf in connections:scf.cf.platform.send_arming_request(False)
    except BaseException as error:
        report.update(status='failed',error=repr(error));raise
    finally:
        stop.set()
        for config in log_configs:
            try:config.stop()
            except Exception:pass
        for worker in workers:worker.join(timeout=1)
        for scf in connections:
            try:scf.cf.commander.send_stop_setpoint();scf.close_link()
            except Exception:pass
        for process in reversed(processes):gz.stop_process(process)
        for stream in streams:stream.close()
        report['forward_errors']=errors
        report.setdefault('checks', {}).update(raw_separation=False, raw_momentum=False,
            rebound_window_velocity=False, rebound_window_restitution=False)
        report['collision_passed']=False
        report['postprocessing']={'schema':'common_timestamp_collision_checks_v2',
            'source_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest()}
        try:
            if len(paths)==2:
                truth_sets=[{r['sim_time_ns']:r for r in gz.parse_odom(p)} for p in paths]
                common=set(truth_sets[0])&set(truth_sets[1])
                if common:
                    report['raw_odom_common_timestamp_count']=len(common)
                    report['raw_odom_minimum_separation_m']=min(math.dist(truth_sets[0][s]['position_m'],truth_sets[1][s]['position_m']) for s in common)
                    report['checks']['raw_separation']=report['raw_odom_minimum_separation_m']>=report['safety_geometry']['minimum_center_separation_m']
                    if options.geometry_close_pass:
                        begin=round(report['approach_sim_time_s']*1e9);finish=round(report['encounter_end_sim_time_s']*1e9)
                        ordered=sorted(s for s in common if begin<=s<=finish)
                        if len(ordered)<2:raise ValueError('no aligned encounter interval for clearance analysis')
                        max_dt=max((b-a)/1e9 for a,b in zip(ordered,ordered[1:]))
                        speed=max(math.dist(truth_sets[0][s]['velocity_m_s'],truth_sets[1][s]['velocity_m_s']) for s in ordered)
                        clearance=report['raw_odom_minimum_separation_m']-2*report['safety_geometry']['bounding_sphere_radius_m']
                        intervals=[]
                        for left,right in zip(ordered,ordered[1:]):
                            distances=[math.dist(truth_sets[0][s]['position_m'],truth_sets[1][s]['position_m']) for s in (left,right)]
                            speeds=[math.dist(truth_sets[0][s]['velocity_m_s'],truth_sets[1][s]['velocity_m_s']) for s in (left,right)]
                            allowance=max(speeds)*(right-left)/1e9/2
                            intervals.append((min(distances)-allowance,allowance))
                        adjusted=min(x[0] for x in intervals)-2*report['safety_geometry']['bounding_sphere_radius_m']
                        report['geometry_clearance']=dict(sampled_sphere_surface_clearance_m=clearance,
                            observed_relative_speed_max_m_s=speed,max_common_sample_interval_s=max_dt,
                            intersample_allowance_m=max(x[1] for x in intervals),clearance_after_observed_speed_allowance_m=adjusted,
                            velocity_scope='aligned approach through capture completion; initial spawn velocity artifacts excluded; full raw position minimum retained',
                            note='Interval-local observed-speed allowance only; not a proof of a velocity bound between samples or a hardware clearance guarantee')
                        surface_gap=report['safety_geometry']['minimum_surface_margin_m']
                        report['checks'].update(cad_surface_clearance_checks(clearance,adjusted,surface_gap))
                    if report.get('capture_sim_time_s') is not None:
                        start_ns=round(report['trigger_sim_time_s']*1e9)
                        end_ns=round(report['capture_sim_time_s']*1e9)
                        initial_ns=min(common,key=lambda s:abs(s-start_ns))
                        incoming=[truth_sets[i][initial_ns]['velocity_m_s'][0] for i in (0,1)]
                        initial_p=.17*sum(incoming)
                        active=sorted(s for s in common if start_ns<=s<=end_ns)
                        window=[s for s in active if end_ns-200_000_000<=s]
                        if len(window)<2 or (window[-1]-window[0])/1e9<.19:raise RuntimeError('invalid or incomplete common-stamp outgoing window')
                        velocity=[[truth_sets[i][s]['velocity_m_s'][0] for i in (0,1)] for s in window]
                        restitution=[-(v[0]-v[1])/(incoming[0]-incoming[1]) for v in velocity]
                        targets=report['target_velocities_m_s']
                        maxima=[max(abs(v[i]-targets[i]) for v in velocity) for i in (0,1)]
                        report['raw_odom_incoming_velocities_m_s']=incoming
                        report['raw_odom_momentum_peak_error_kg_m_s']=max(abs(.17*sum(truth_sets[i][s]['velocity_m_s'][0] for i in (0,1))-initial_p) for s in active)
                        report['outgoing_window']=dict(duration_s=(window[-1]-window[0])/1e9,samples=len(window),
                            minimum_velocities_m_s=[min(v[i] for v in velocity) for i in (0,1)],
                            maximum_velocities_m_s=[max(v[i] for v in velocity) for i in (0,1)],
                            maximum_speed_errors_m_s=maxima,minimum_restitution=min(restitution),maximum_restitution=max(restitution))
                        report['actual_zero_crossing_after_trigger_s']=[next(((s-start_ns)/1e9 for s in active if truth_sets[i][s]['velocity_m_s'][0]*incoming[i]<=0),None) for i in (0,1)]
                        report['outgoing_window']['mean_acceleration_m_s2']=[(velocity[-1][i]-velocity[0][i])/((window[-1]-window[0])/1e9) for i in (0,1)]
                        report['outgoing_window']['maximum_abs_acceleration_m_s2']=[max(abs((velocity[k+1][i]-velocity[k][i])/((window[k+1]-window[k])/1e9)) for k in range(len(window)-1)) for i in (0,1)]
                        active_v=[[truth_sets[i][s]['velocity_m_s'][0] for i in (0,1)] for s in active]
                        peak_out=[max(v[i]*math.copysign(1,targets[i]) for v in active_v) for i in (0,1)]
                        active_e=[-(v[0]-v[1])/(incoming[0]-incoming[1]) for v in active_v]
                        fidelity=[max(abs(v[i]-targets[i]) for i in (0,1))<=.1 and abs(e-1)<=.1 for v,e in zip(active_v,active_e)]
                        last_bad=max((k for k,good in enumerate(fidelity) if not good),default=-1)
                        settled=active[last_bad+1] if last_bad+1<len(active) else None
                        report['transient_rebound']=dict(peak_outgoing_speeds_m_s=peak_out,
                            peak_outgoing_overspeed_m_s=[max(0,peak_out[i]-abs(targets[i])) for i in (0,1)],
                            peak_restitution=max(active_e),
                            settled_after_trigger_s=None if settled is None else (settled-start_ns)/1e9,
                            settling_definition='First common timestamp after the final violation of per-drone speed error <=.10 m/s and |e-1|<=.10 through capture; finite recorded horizon',
                            peak_fidelity_passed=max(max(0,peak_out[i]-abs(targets[i])) for i in (0,1))<=.1 and max(active_e)<=1.1)
                        ratios=[peak_out[i]/abs(targets[i]) for i in (0,1)]
                        transitions=[]
                        def crossing(values,threshold):
                            for k in range(1,len(values)):
                                if values[k]<=threshold<values[k-1]:
                                    fraction=(values[k-1]-threshold)/(values[k-1]-values[k])
                                    return (active[k-1]-start_ns+fraction*(active[k]-active[k-1]))/1e9
                            return None
                        for i in (0,1):
                            direction=math.copysign(1,incoming[i]);along=[v[i]*direction for v in active_v]
                            first=crossing(along,.9*abs(incoming[i]));last=crossing(along,-.9*abs(targets[i]))
                            transitions.append(None if first is None or last is None else last-first)
                        report['transient_rebound'].update(peak_outgoing_speed_ratios=ratios,transition_90_to_minus90_s=transitions)
                        report['physical_envelope']['maximum_actual_x_acceleration_m_s2']=[max(abs((active_v[k+1][i]-active_v[k][i])/((active[k+1]-active[k])/1e9)) for k in range(len(active)-1)) for i in (0,1)]
                        report['fast_collision_checks']=dict(peak_speed_ratio=max(ratios)<=1.1,
                            peak_restitution=max(active_e)<=1.1,
                            transition_time=all(t is not None and t<.30 for t in transitions),
                            close_center=report['raw_odom_minimum_separation_m']<.32)
                        report['fast_collision_benchmark']=dict(maximum_center_distance_m_exclusive=.32,
                            geometry_floor_allows_close_center=report['safety_geometry']['minimum_center_separation_m']<.32,
                            scope='Fixed historical fast-collision benchmark; its <0.32 m center target is unchanged when a different minimum surface gap is requested.')
                        report['checks']['rebound_window_velocity']=max(maxima)<=.10 and (window[-1]-window[0])/1e9>=.19
                        report['checks']['rebound_window_restitution']=max(abs(e-1.) for e in restitution)<=.10
                        report['checks']['raw_momentum']=report['raw_odom_momentum_peak_error_kg_m_s']<=.01
                        report['collision_passed']=all(report['checks'].values())
        except (ValueError, OSError, KeyError, ZeroDivisionError, RuntimeError) as error:
            report['postprocessing_error']=repr(error)
            report['checks']['raw_separation']=False
        report['collision_passed']=bool(report.get('hover_passed') and
            report.get('status')=='collision_complete' and all(report['checks'].values()))
        report['fast_collision_passed']=bool(report['collision_passed'] and report.get('fast_collision_checks') and all(report['fast_collision_checks'].values()))
        (output/'report.json').write_text(json.dumps(report,indent=2))
        with (output/'trace.jsonl').open('w') as stream:
            for row in rows:stream.write(json.dumps(row)+'\n')
        with (output/'firmware_diagnostics.jsonl').open('w') as stream:
            for row in firmware_rows:stream.write(json.dumps(row)+'\n')
        print(json.dumps(report,indent=2),flush=True)
    return report

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',required=True);parser.add_argument('--collision',action='store_true')
    parser.add_argument('--reference-acceleration',type=float,default=2.)
    parser.add_argument('--reference-jerk',type=float,default=8.)
    parser.add_argument('--trigger-reference',help='Prior Gazebo result directory, absolute or relative to the repository')
    parser.add_argument('--trigger-reserve',type=float,default=.06)
    parser.add_argument('--geometry-close-pass',action='store_true',help='Separate CAD-derived noncontact scenario; preserves the baseline .40 m default')
    parser.add_argument('--minimum-surface-gap',type=float,default=None,metavar='METRES',help='Minimum conservative CAD bounding-sphere gap in (0,0.50] m; requires --geometry-close-pass; defaults to 0.03 m in that mode, not a target for observed CAD distance')
    parser.add_argument('--jerk-feedforward-lead',type=float,default=.10,help='Simulation-only reference-jerk lead; measured tilt-rate damping stays .10 s')
    parser.add_argument('--command-acceleration-cap',type=float,default=3.,help='Collision-phase equivalent acceleration command cap; baseline approach/capture retain3m/s²')
    parser.add_argument('--attitude-bias-compensation',action='store_true',help='Simulation-only filtered estimated-versus-true pitch correction')
    parser.add_argument('--predictive-unwind',action='store_true',help='Simulation-only remaining-attitude-impulse terminal controller')
    parser.add_argument('--unwind-impulse-time',type=float,default=.262)
    parser.add_argument('--unwind-velocity-gain',type=float,default=5.)
    parser.add_argument('--unwind-rate-damping',type=float,default=.20)
    parser.add_argument('--unwind-tilt-jerk-weight',type=float,default=.0137)
    args=parser.parse_args()
    options=CollisionExperimentOptions(args.reference_acceleration,args.reference_jerk,args.trigger_reference,args.trigger_reserve,args.geometry_close_pass,args.jerk_feedforward_lead,args.command_acceleration_cap,args.attitude_bias_compensation,args.predictive_unwind,args.unwind_impulse_time,args.unwind_velocity_gain,args.unwind_rate_damping,args.unwind_tilt_jerk_weight,args.minimum_surface_gap)
    try:options.validate()
    except ValueError as error:parser.error(str(error))
    result=run(args.output,args.collision,options)
    if args.collision and not result.get('collision_passed'):raise SystemExit(1)
