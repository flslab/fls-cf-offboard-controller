"""Read-only, common-time physics and CAD-clearance review of a Gazebo run.

The input is a completed run directory containing report.json, trace.jsonl,
odom_0/1.jsonl and the saved model/mesh files. This module never starts a
simulator or connects to a vehicle. CAD clearance is a support-plane lower
bound on surface distance, not an exact triangle-to-triangle distance.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial import ConvexHull


def quaternion_rotation(quaternion):
    q = np.asarray(quaternion, dtype=float)
    norm = math.sqrt(float(np.sum(q * q)))
    if not np.all(np.isfinite(q)) or norm < 1e-12:
        raise ValueError("invalid odometry quaternion")
    x, y, z, w = q / norm
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def read_odom(path):
    """Gazebo child-frame twist -> world velocity using its normalized pose."""
    records = {}
    for line in Path(path).read_text().splitlines():
        if not line.lstrip().startswith("{"):
            continue
        row = json.loads(line)
        stamp = row["header"]["stamp"]
        ns = int(stamp.get("sec", 0))*1_000_000_000 + int(stamp.get("nsec", 0))
        pose = row["pose"]
        rotation = quaternion_rotation([pose["orientation"].get(k, 0.) for k in "xyzw"])
        position = np.array([pose["position"].get(k, 0.) for k in "xyz"])
        body_velocity = np.array([row["twist"]["linear"].get(k, 0.) for k in "xyz"])
        velocity = np.einsum("ij,j->i", rotation, body_velocity)
        if not np.all(np.isfinite(np.r_[position, velocity])):
            raise ValueError(f"nonfinite odometry at {ns}")
        records[ns] = (position, velocity, rotation)
    return records


def pose_transform(text):
    x, y, z, roll, pitch, yaw = map(float, (text or "0 0 0 0 0 0").split())
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    rotation = np.array([[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                         [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr],
                         [-sp, cp*sr, cp*cr]])
    return np.array([x, y, z]), rotation


def read_cad(run, drone, cache):
    tree = ET.parse(run / f"model_{drone}.sdf")
    link = tree.find("model/link[@name='base_link']")
    if link is None:
        raise ValueError("saved model has no base_link")
    if np.any(np.array(list(map(float, (link.findtext("pose") or "0 0 0 0 0 0").split())))):
        raise ValueError("nonzero base_link pose requires explicit odometry-frame review")
    visual = link.find("visual[@name='body_visual']")
    mesh = visual.find("geometry/mesh")
    path = run / Path(mesh.findtext("uri")).name
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    if digest not in cache:
        vertices = np.array([list(map(float, line.split()[1:4]))
                             for line in path.open() if line.startswith("v ")])
        if not len(vertices) or not np.all(np.isfinite(vertices)):
            raise ValueError("invalid CAD vertices")
        hull = ConvexHull(vertices)
        cache[digest] = (vertices, vertices[hull.vertices])
    vertices, hull = cache[digest]
    scale = np.array(list(map(float, (mesh.findtext("scale") or "1 1 1").split())))
    translation, rotation = pose_transform(visual.findtext("pose"))
    transformed = np.einsum("ij,kj->ki", rotation, vertices*scale) + translation
    support_vertices = np.einsum("ij,kj->ki", rotation, hull*scale) + translation
    return support_vertices, dict(
        mesh_file=path.name, mesh_sha256=digest, vertex_count=len(vertices),
        convex_hull_vertex_count=len(hull), visual_pose=visual.findtext("pose"),
        mesh_scale=scale.tolist(), base_frame_bounds_min_m=transformed.min(axis=0).tolist(),
        base_frame_bounds_max_m=transformed.max(axis=0).tolist(),
        axis_aligned_size_m=np.ptp(transformed, axis=0).tolist(),
        origin_centered_bounding_sphere_radius_m=float(np.linalg.norm(transformed, axis=1).max()),
        collision_geometry=[ET.tostring(c.find("geometry"), encoding="unicode")
                            for c in link.findall("collision")],
        excluded_other_base_visual_names=[v.attrib.get("name") for v in link.findall("visual")
                                         if v is not visual],
        scope="Full saved body CAD, including its cage and static propeller parts; excludes separate annotation and animated-prop visuals.",
    )


def read_rotating_prop_bounds(run, drone):
    """Enclose the extra animated prop visuals for any rotation about their axes.

    Current FLS models use one Collada mesh and one matrix node per prop. Reject
    other layouts instead of silently dropping a transform. Run-local copies
    are preferred; the checked local CrazySim mesh is an explicit fallback.
    """
    root = ET.parse(run / f"model_{drone}.sdf")
    fallback = (Path(__file__).resolve().parents[2] / "CrazySim/crazyflie-firmware/tools/"
                "crazyflie-simulation/meshes/collada_files")
    bounds = []
    for link in root.findall("model/link"):
        if link.attrib.get("name") == "base_link":
            continue
        link_translation, _ = pose_transform(link.findtext("pose"))
        for visual in link.findall("visual"):
            mesh = visual.find("geometry/mesh")
            if mesh is None:
                raise ValueError("unreviewed animated visual primitive")
            name = Path(mesh.findtext("uri")).name
            candidates = [run / name, fallback / name]
            path = next((p for p in candidates if p.is_file()), None)
            if path is None:
                raise ValueError(f"cannot inspect rotating prop geometry {name}")
            collada = ET.parse(path)
            ns = {"c": collada.getroot().tag.split("}")[0][1:]}
            unit = float(collada.find("c:asset/c:unit", ns).attrib["meter"])
            sources = [source for source in collada.findall(".//c:source", ns)
                       if "position" in source.attrib.get("id", "").lower()]
            matrices = collada.findall(".//c:visual_scene//c:matrix", ns)
            if len(sources) != 1 or len(matrices) != 1:
                raise ValueError("unsupported Collada prop scene layout")
            vertices = np.fromstring(sources[0].find("c:float_array", ns).text,
                                     sep=" ").reshape(-1, 3)
            transform = np.fromstring(matrices[0].text, sep=" ").reshape(4, 4)
            transformed = (np.einsum("ij,kj->ki", transform[:3, :3], vertices)
                           + transform[:3, 3])*unit
            scale = np.array(list(map(float, (mesh.findtext("scale") or "1 1 1").split())))
            visual_translation, _ = pose_transform(visual.findtext("pose"))
            # Norm sums are conservative under unknown rotor rotation. The
            # visual translation also rotates with the joint and remains inside
            # this per-link sphere for every rotor angle.
            radius = (float(np.linalg.norm(link_translation))
                      + float(np.linalg.norm(visual_translation))
                      + float(np.linalg.norm(transformed*scale, axis=1).max()))
            bounds.append(dict(link=link.attrib.get("name"), mesh_file=name,
                source_path=str(path), source_sha256=hashlib.sha256(path.read_bytes()).hexdigest(),
                bounding_sphere_radius_m=radius))
    return bounds


def downward_crossing(times, values, threshold):
    for i in range(1, len(times)):
        if times[i] < 0:
            continue
        a, b = values[i-1]-threshold, values[i]-threshold
        if a >= 0 and b <= 0 and a != b:
            return float(times[i-1] + (times[i]-times[i-1])*a/(a-b))
    return None


def first_dwell(times, mask, duration=.02):
    for i, t in enumerate(times):
        if t < 0 or not mask[i]:
            continue
        j = np.searchsorted(times, t+duration)
        if j < len(times) and np.all(mask[i:j+1]):
            return float(t)
    return None


def attitude_diagnostics(run, raw, trace, start, end):
    """Compare received firmware telemetry with saved truth, without assuming sync.

    The firmware clock has no recorded exact mapping to Gazebo time. Receipt
    timestamps are approximate; repeat the comparison at +/-20 ms to expose
    sensitivity. Pitch sign is converted to Crazyflie's convention.
    """
    path = run / "firmware_diagnostics.jsonl"
    if not path.is_file():
        return None
    records = [json.loads(line) for line in path.read_text().splitlines()]
    commands = [r for r in trace if "commands" in r]
    command_times = np.array([r["sim_time_s"] for r in commands])
    result = []
    for drone in (0, 1):
        stamps = np.array(sorted(raw[drone]), dtype=np.int64)
        truth_times = stamps/1e9
        truth_pitch = np.array([-math.degrees(math.asin(np.clip(-raw[drone][t][2][2, 0], -1., 1.)))
                                for t in stamps])
        all_rows = [r for r in records if r["drone"] == drone
                    and r.get("approximate_truth_receipt_ns") is not None
                    and end-200_000_000 <= r["approximate_truth_receipt_ns"] <= end]
        # Log blocks can have independent field sets and receipt times. Only
        # pitch-bearing records may enter the estimator/truth comparison.
        rows = [r for r in all_rows if "stabilizer.pitch" in r["values"]]
        if not rows:
            result.append(dict(drone=drone, samples=0))
            continue
        receipt_times = np.array([r["approximate_truth_receipt_ns"]/1e9 for r in rows])
        names = sorted(set.union(*(set(r["values"]) for r in all_rows)))
        means = {name: float(np.mean([r["values"][name] for r in all_rows if name in r["values"]]))
                 for name in names}
        estimated = np.array([r["values"]["stabilizer.pitch"] for r in rows])
        aligned_truth = np.interp(receipt_times, truth_times, truth_pitch)
        requested = np.interp(receipt_times, command_times,
                              [r["commands"][drone]["pitch_deg"] for r in commands])
        shifted_errors = {str(shift): float(np.mean(estimated - np.interp(
            receipt_times+shift, truth_times, truth_pitch))) for shift in (-.02, 0., .02)}
        result.append(dict(drone=drone, samples=len(rows), firmware_values_mean=means,
            truth_cf_pitch_mean_deg=float(aligned_truth.mean()),
            host_pitch_command_mean_deg=float(requested.mean()),
            estimated_minus_truth_pitch_mean_deg=float((estimated-aligned_truth).mean()),
            host_command_minus_estimated_pitch_mean_deg=float((requested-estimated).mean()),
            estimated_minus_truth_mean_with_truth_time_shift_deg=shifted_errors))
    return dict(outgoing_window=result,
        alignment="Firmware telemetry receipt time matched to interpolated Gazebo truth; approximate, not synchronized sensor time. +/-20 ms shifts are a sensitivity diagnostic, not a guaranteed latency bound.",
        log_semantics="stabilizer.pitch is estimator pitch; controller.pitch is desired angle if logged; controller.cmd_pitch is actuator output, not an angle; pid_attitude.pitch_outI and pid_rate.pitch_outI have different controller units.")


def motor_diagnostics(run, start, end):
    path = run / "firmware_diagnostics.jsonl"
    if not path.is_file():
        return None
    records = [json.loads(line) for line in path.read_text().splitlines()]
    result = []
    names = [f"motor.m{i}" for i in range(1, 5)]
    for drone in (0, 1):
        rows = [r for r in records if r["drone"] == drone
                and r.get("approximate_truth_receipt_ns") is not None
                and start <= r["approximate_truth_receipt_ns"] <= end
                and all(name in r["values"] for name in names)]
        if not rows:
            result.append(dict(drone=drone, samples=0))
            continue
        values = np.array([[r["values"][name] for name in names] for r in rows])
        stamps = np.array([r["approximate_truth_receipt_ns"] for r in rows], dtype=np.int64)
        result.append(dict(drone=drone, samples=len(rows),
            minimum_motor_pwm=values.min(axis=0).tolist(),
            maximum_motor_pwm=values.max(axis=0).tolist(),
            sampled_motor_values_at_uint16_ceiling=int(np.count_nonzero(values >= 65535)),
            smallest_sampled_headroom_pwm=float(65535-values.max()),
            maximum_receipt_interval_s=float(np.diff(stamps).max()/1e9) if len(rows) > 1 else None))
    return dict(vehicles=result,
        scope="Received post-mixer PWM telemetry during trigger-to-capture; lack of sampled saturation does not exclude brief saturation between log records. PWM is not measured motor speed or thrust.")


def analyze(run):
    run = Path(run).resolve()
    report = json.loads((run / "report.json").read_text())
    trace = [json.loads(line) for line in (run / "trace.jsonl").read_text().splitlines()]
    start = round(next(r["sim_time_s"] for r in trace if r.get("phase") == "transition")*1e9)
    end = round(next(r["sim_time_s"] for r in trace if r.get("phase") == "capture")*1e9)
    raw = [read_odom(run / f"odom_{i}.jsonl") for i in (0, 1)]
    common = sorted(set(raw[0]) & set(raw[1]))
    if start not in common or end not in common:
        raise ValueError("trigger/capture lacks an exact common timestamp")
    active = np.array([t for t in common if start <= t <= end], dtype=np.int64)
    times = np.array([t for t in common if start-200_000_000 <= t <= end], dtype=np.int64)
    relative_time = (times-start)/1e9
    velocity = np.array([[raw[i][t][1][0] for i in (0, 1)] for t in times])
    acceleration = np.gradient(velocity, relative_time, axis=0)
    active_positions = np.array([[raw[i][t][0] for i in (0, 1)] for t in active])
    active_velocity_xyz = np.array([[raw[i][t][1] for i in (0, 1)] for t in active])
    interval_acceleration_xyz = np.diff(active_velocity_xyz, axis=0)/(np.diff(active)[:, None, None]/1e9)
    actual_tilt = np.array([[math.degrees(math.acos(np.clip(raw[i][t][2][2, 2], -1., 1.)))
                            for i in (0, 1)] for t in active])
    cfg = report["plan_config"]
    masses = np.array([cfg["mass1_kg"], cfg["mass2_kg"]])
    incoming = np.array([raw[i][start][1][0] for i in (0, 1)])
    planned_incoming = np.array([cfg["velocity1_m_s"], cfg["velocity2_m_s"]])
    targets = np.array(report["target_velocities_m_s"])
    relative_incoming = incoming[0]-incoming[1]
    if relative_incoming <= 0:
        raise ValueError("common-time incoming state is not approaching")
    restitution = (velocity[:, 1]-velocity[:, 0])/relative_incoming
    active_velocity = velocity[relative_time >= 0]
    active_momentum = np.einsum("ij,j->i", active_velocity, masses)
    raw_momentum_error = active_momentum-float(np.dot(incoming, masses))
    planned_momentum_error = active_momentum-float(np.dot(planned_incoming, masses))
    window_mask = times >= end-200_000_000
    window_times, window_velocity = times[window_mask], velocity[window_mask]
    window_error = window_velocity-targets
    window_restitution = restitution[window_mask]
    window_acceleration = np.diff(window_velocity, axis=0)/(np.diff(window_times)[:, None]/1e9)
    duration = float((window_times[-1]-window_times[0])/1e9)
    gaps = np.array([np.linalg.norm(raw[1][t][0]-raw[0][t][0]) for t in common])
    endpoint_error = active_velocity[-1]-targets
    endpoint_e = float((active_velocity[-1, 1]-active_velocity[-1, 0])/
                       (planned_incoming[0]-planned_incoming[1]))
    gate = cfg["minimum_separation_m"]
    independent_checks = dict(
        separation=bool(gaps.min() >= gate),
        terminal_velocity=bool(abs(endpoint_error).max() <= .10),
        transient_momentum=bool(abs(planned_momentum_error).max() <= .01),
        endpoint_momentum=bool(abs(planned_momentum_error[-1]) <= .01),
        restitution=abs(endpoint_e-cfg["restitution"]) <= .10,
        rebound_window_velocity=bool(abs(window_error).max() <= .10 and duration >= .19),
        rebound_window_restitution=bool(abs(window_restitution-cfg["restitution"]).max() <= .10),
        raw_momentum=bool(abs(raw_momentum_error).max() <= .01),
        raw_separation=bool(gaps.min() >= gate),
    )
    metrics = []
    for i in (0, 1):
        direction = float(np.sign(incoming[i]))
        along = velocity[:, i]*direction
        directional_a = acceleration[:, i]*direction
        t90in = downward_crossing(relative_time, along, .9*abs(incoming[i]))
        t90out = downward_crossing(relative_time, along, -.9*abs(targets[i]))
        outgoing_indices = np.flatnonzero(relative_time >= 0)
        peak_index = int(outgoing_indices[np.argmax(-along[outgoing_indices])])
        peak_speed = float(-along[peak_index])
        metrics.append(dict(
            drone=i, incoming_velocity_m_s=float(incoming[i]), target_velocity_m_s=float(targets[i]),
            braking_onset_gt_02_m_s2_for_20ms_s=first_dwell(relative_time, directional_a <= -.2),
            positive_90pct_time_s=t90in,
            zero_crossing_time_s=downward_crossing(relative_time, along, 0),
            negative_90pct_time_s=t90out,
            velocity_90_to_90_duration_s=t90out-t90in if t90in is not None and t90out is not None else None,
            peak_braking_acceleration_m_s2=float(-directional_a[relative_time >= 0].min()),
            peak_outgoing_speed_m_s=peak_speed,
            peak_outgoing_time_s=float(relative_time[peak_index]),
            peak_outgoing_speed_ratio=peak_speed/abs(targets[i]) if targets[i] != 0 else None,
            peak_outgoing_speed_excess_m_s=peak_speed-abs(targets[i]),
        ))
    stable = ((np.max(abs(velocity-targets), axis=1) <= .10)
              & (abs(restitution-cfg["restitution"]) <= .10)
              & (np.max(abs(acceleration), axis=1) <= .10))
    last_bad = np.flatnonzero((~stable) & (relative_time >= 0))
    settled = (float(relative_time[last_bad[-1]+1])
               if len(last_bad) and last_bad[-1]+1 < len(times) else None)
    if not len(last_bad):
        settled = 0.
    cache = {}
    cad = [read_cad(run, i, cache) for i in (0, 1)]
    geometry = []
    for t in active:
        delta = raw[1][t][0]-raw[0][t][0]
        distance = float(np.linalg.norm(delta))
        normal = delta/distance
        support = [float(np.max(np.einsum("ij,j->i", cad[i][0],
                        np.einsum("ji,j->i", raw[i][t][2], normal*(1 if i == 0 else -1)))))
                   for i in (0, 1)]
        geometry.append([distance-sum(support), distance, *support])
    geometry = np.array(geometry)
    closest = int(geometry[:, 0].argmin())
    energy_before = .5*float(np.dot(incoming*incoming, masses))
    window_energy = .5*np.einsum("ij,j->i", window_velocity**2, masses)
    active_energy = .5*np.einsum("ij,j->i", active_velocity**2, masses)
    actual_trigger_gap = float(np.linalg.norm(raw[1][start][0]-raw[0][start][0]))
    props = [read_rotating_prop_bounds(run, i) for i in (0, 1)]
    radii = [max(cad[i][1]["origin_centered_bounding_sphere_radius_m"],
                 *(v["bounding_sphere_radius_m"] for v in props[i])) for i in (0, 1)]
    stated_geometry = report.get("safety_geometry", {})
    geometry_mode = stated_geometry.get("mode", "baseline_unstated")
    geometry_validation = dict(mode=geometry_mode, stated_geometry=stated_geometry,
        independently_computed_vehicle_radius_m=radii, animated_prop_bounds=props,
        minimum_sampled_sphere_clearance_m=float(gaps.min()-sum(radii)),
        continuous_motion_clearance_guaranteed=False,
        geometry_implication="If center distance stays above the sphere-radius sum plus the requested margin at every instant, the enclosed rigid surfaces stay separated by that margin. Sampled odometry alone does not prove that premise between frames.")
    if geometry_mode == "cad_bounded_close_pass":
        margin = float(stated_geometry["minimum_surface_margin_m"])
        stated_floor = float(stated_geometry["minimum_center_separation_m"])
        required_floor = 2*max(radii)+margin
        if len(stated_geometry["vehicle_bounds"]) != 2:
            raise ValueError("geometry report must cover both vehicles")
        source_matches = True
        for i, stated in enumerate(stated_geometry["vehicle_bounds"]):
            actual_hashes = {cad[i][1]["mesh_file"]: cad[i][1]["mesh_sha256"],
                             f"model_{i}.sdf": hashlib.sha256((run/f"model_{i}.sdf").read_bytes()).hexdigest()}
            actual_hashes.update({p["mesh_file"]: p["source_sha256"] for p in props[i]})
            stated_hashes = {Path(name).name: digest for name, digest in stated["source_sha256"].items()}
            source_matches &= stated_hashes == actual_hashes
        no_annotation = all("contact_side_orientation_marker" not in
                            item[1]["excluded_other_base_visual_names"] for item in cad)
        geometry_checks = dict(
            reported_geometry_source_hashes_match=bool(source_matches),
            reported_radius_matches=math.isclose(float(stated_geometry["bounding_sphere_radius_m"]),
                                                max(radii), rel_tol=0., abs_tol=1e-9),
            reported_floor_matches=math.isclose(stated_floor, required_floor, rel_tol=0., abs_tol=1e-9),
            plan_floor_matches=math.isclose(gate, stated_floor, rel_tol=0., abs_tol=1e-9),
            annotation_removed=no_annotation,
            sampled_sphere_clearance=bool(gaps.min()-sum(radii) >= margin),
            sampled_projected_cad_clearance=bool(geometry[:, 0].min() >= margin),
        )
        observed_relative_speed = max(float(np.linalg.norm(raw[1][t][1]-raw[0][t][1]))
                                      for t in active)
        max_interval = float(np.diff(active).max()/1e9)
        allowance = observed_relative_speed*max_interval/2
        sphere_clearance = float(gaps.min()-sum(radii))
        independent_checks.update(cad_surface_clearance=sphere_clearance >= margin,
                                  cad_clearance_with_allowance=sphere_clearance-allowance >= margin)
        geometry_validation.update(requested_surface_margin_m=margin,
            required_center_floor_m=required_floor, checks=geometry_checks,
            sampled_geometry_passed=all(geometry_checks.values()),
            observed_active_motion_allowance=dict(relative_speed_max_m_s=observed_relative_speed,
                maximum_sample_interval_s=max_interval, allowance_m=allowance,
                sphere_clearance_after_allowance_m=sphere_clearance-allowance,
                scope="Diagnostic using observed speeds during trigger-to-capture only; not a verified intersample speed bound or a continuous-clearance guarantee."))
    requested_accelerations = [math.hypot(c["ax_m_s2"], c["ay_m_s2"])
                              for r in trace if r.get("phase") in ("transition", "separation")
                              for c in r["commands"]]
    cap = float(report.get("physical_envelope", {}).get("command_acceleration_cap_m_s2",
                report.get("experiment_options", {}).get("command_acceleration_cap_m_s2", 3.)))
    command_tilts = [math.degrees(math.acos(np.clip(
        math.cos(math.radians(c["roll_deg"]))*math.cos(math.radians(c["pitch_deg"])), -1., 1.)))
        for r in trace if r.get("phase") in ("transition", "separation") for c in r["commands"]]
    settled_duration = None if settled is None else float((end-start)/1e9-settled)
    fast_checks = dict(
        peak_speed_ratio=all(m["peak_outgoing_speed_ratio"] is not None and
                             m["peak_outgoing_speed_ratio"] <= 1.1 for m in metrics),
        peak_restitution=float(restitution[relative_time >= 0].max()) <= 1.1,
        transition_time=all(m["velocity_90_to_90_duration_s"] is not None and
                            m["velocity_90_to_90_duration_s"] < .30 for m in metrics),
        close_center=float(gaps.min()) < .32,
    )
    active_trace = [r for r in trace if r.get("phase") in ("transition", "separation")]
    bias_age = [c.get("bias_telemetry_age_s") for r in active_trace for c in r["commands"]]
    bias_age = [age for age in bias_age if age is not None]
    applied_bias = [c.get("applied_pitch_bias_deg", 0.) for r in active_trace for c in r["commands"]]
    hashes = report.get("source_sha256", {})
    source_matches = {}
    for reported_name, digest in hashes.items():
        basename = Path(reported_name).name
        local_name = {"simulate_virtual_collision_gazebo.py": "harness_used.py",
                      "virtual_collision_sim.py": "collision_profile_used.py"}.get(basename)
        if local_name and (run/local_name).is_file():
            source_matches[basename] = hashlib.sha256((run/local_name).read_bytes()).hexdigest() == digest
    return dict(
        schema="virtual_collision_realism_review_v1", source_run=str(run),
        scope="Read-only simulation analysis; no physical contact, motor calibration or flight validation inferred.",
        trigger_sim_time_s=start/1e9, capture_sim_time_s=end/1e9,
        common_timestamp_count=len(common), active_common_timestamp_count=len(active),
        maximum_active_common_timestamp_interval_s=float(np.diff(active).max()/1e9),
        maximum_collision_command_interval_s=float(np.diff([r["sim_time_s"] for r in active_trace]).max()),
        incoming_velocities_m_s=incoming.tolist(), incoming_relative_speed_m_s=float(relative_incoming),
        target_velocities_m_s=targets.tolist(), minimum_center_separation_m=float(gaps.min()),
        maximum_raw_momentum_error_kg_m_s=float(abs(raw_momentum_error).max()),
        endpoint_raw_momentum_error_kg_m_s=float(raw_momentum_error[-1]),
        independent_checks=independent_checks,
        independent_passed=all(independent_checks.values()) and geometry_validation.get("sampled_geometry_passed", True),
        report_collision_passed=report.get("collision_passed"),
        report_check_agreement={k: report.get("checks", {}).get(k) == v for k, v in independent_checks.items()},
        independent_fast_checks=fast_checks,
        independent_fast_passed=all(independent_checks.values()) and all(fast_checks.values())
            and geometry_validation.get("sampled_geometry_passed", True),
        report_fast_check_agreement={k: report.get("fast_collision_checks", {}).get(k) == v
                                    for k, v in fast_checks.items()},
        fast_check_thresholds=dict(peak_speed_ratio_maximum=1.1, peak_restitution_maximum=1.1,
                                  transition_time_exclusive_maximum_s=.30,
                                  center_distance_exclusive_maximum_m=.32),
        saved_source_hash_checks=source_matches,
        outgoing_window=dict(duration_s=duration, samples=len(window_times),
            maximum_sample_interval_s=float(np.diff(window_times).max()/1e9),
            maximum_speed_errors_m_s=abs(window_error).max(axis=0).tolist(),
            rms_speed_errors_m_s=np.sqrt((window_error**2).mean(axis=0)).tolist(),
            minimum_restitution=float(window_restitution.min()), maximum_restitution=float(window_restitution.max()),
            maximum_abs_acceleration_m_s2=abs(window_acceleration).max(axis=0).tolist(),
            kinetic_energy_ratio_range=[float(window_energy.min()/energy_before), float(window_energy.max()/energy_before)]),
        motion_metrics=metrics, combined_settled_time_after_trigger_s=settled,
        combined_settled_observed_duration_s=settled_duration,
        combined_settled_for_at_least_02s=settled_duration is not None and settled_duration >= .2,
        peak_transient_restitution=float(restitution[relative_time >= 0].max()),
        peak_transient_kinetic_energy_ratio=float(active_energy.max()/energy_before),
        settled_criteria="speed error <= .1 m/s, restitution error <= .1 and sampled |ax| <= .1 m/s2 at every observed timestamp through capture; finite recorded horizon",
        cad_geometry=[item[1] for item in cad],
        geometry_validation=geometry_validation,
        command_cap_review=dict(maximum_requested_acceleration_m_s2=max(requested_accelerations),
            requested_cap_m_s2=cap, command_cap_respected=max(requested_accelerations) <= cap+1e-9,
            actual_acceleration_bound_enforced=False,
            actual_peak_braking_acceleration_m_s2=[m["peak_braking_acceleration_m_s2"] for m in metrics]),
        observed_physical_envelope=dict(
            scope="Raw common-time Gazebo poses and finite-interval velocity derivatives from trigger through capture; sampled observations, not hard physical bounds.",
            maximum_actual_tilt_deg=actual_tilt.max(axis=0).tolist(),
            maximum_commanded_tilt_deg=max(command_tilts),
            altitude_minimum_m=active_positions[:, :, 2].min(axis=0).tolist(),
            altitude_maximum_m=active_positions[:, :, 2].max(axis=0).tolist(),
            altitude_at_trigger_m=active_positions[0, :, 2].tolist(),
            maximum_altitude_drop_from_trigger_m=(active_positions[0, :, 2]-active_positions[:, :, 2].min(axis=0)).tolist(),
            maximum_absolute_acceleration_xyz_m_s2=abs(interval_acceleration_xyz).max(axis=0).tolist(),
            maximum_horizontal_acceleration_m_s2=np.linalg.norm(interval_acceleration_xyz[:, :, :2], axis=2).max(axis=0).tolist(),
            maximum_vertical_speed_m_s=abs(active_velocity_xyz[:, :, 2]).max(axis=0).tolist()),
        firmware_attitude_diagnostics=attitude_diagnostics(run, raw, trace, start, end),
        firmware_motor_diagnostics=motor_diagnostics(run, start, end),
        bias_observer_review=dict(
            enabled=bool(report.get("experiment_options", {}).get("attitude_bias_compensation", False)),
            minimum_receipt_age_s=min(bias_age) if bias_age else None,
            maximum_receipt_age_s=max(bias_age) if bias_age else None,
            receipt_age_within_declared_bounds=bool(bias_age and min(bias_age) >= -.01 and max(bias_age) <= .05),
            maximum_applied_pitch_correction_deg=max(abs(v) for v in applied_bias),
            pitch_correction_within_declared_3_degree_bound=max(abs(v) for v in applied_bias) <= 3.,
            scope="Host correction uses firmware estimated pitch minus Gazebo truth pitch at approximate telemetry receipt; this requires ideal state access and is not an onboard estimator validation."),
        minimum_cad_projected_clearance=dict(clearance_m=float(geometry[closest, 0]),
            center_distance_m=float(geometry[closest, 1]),
            facing_support_extents_m=geometry[closest, 2:].tolist(),
            time_after_trigger_s=float((active[closest]-start)/1e9),
            interpretation="Positive centerline support-plane clearance lower-bounds full CAD surface distance; sampled poses only."),
        sphere_clearance_at_configured_gate_m=float(gate-sum(item[1]["origin_centered_bounding_sphere_radius_m"] for item in cad)),
        actual_trigger_center_distance_m=actual_trigger_gap,
        actual_relative_closure_m=actual_trigger_gap-float(gaps.min()),
        conditional_full_reversal_lower_bound_s_if_actual_acceleration_bounded_by_command_cap=(abs(incoming-targets)/cap).tolist(),
        acceleration_bound_caveat="The declared cap limits host equivalent-horizontal-acceleration commands, not actual acceleration; this time bound is conditional and not an enforced performance limit.",
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("run", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    result = analyze(args.run)
    output = args.output or args.run / "independent_realism_review.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2, allow_nan=False))
    print(json.dumps({k: result[k] for k in (
        "source_run", "incoming_velocities_m_s", "independent_passed", "independent_checks",
        "outgoing_window", "motion_metrics", "combined_settled_time_after_trigger_s",
        "minimum_cad_projected_clearance", "sphere_clearance_at_configured_gate_m",
        "command_cap_review", "geometry_validation")}, indent=2))


if __name__ == "__main__":
    main()
