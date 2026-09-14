"""Validated configuration for the three contact-attitude comparison flights.

The experiment selector changes estimator diagnostics and Vicon input routing
only.  It never grants the shadow observer command authority and never changes
commander, handoff, or setpoint ownership.
"""

from __future__ import annotations

from copy import deepcopy
import math
from typing import Mapping


ONBOARD_MIRROR = "onboard_mirror"
INERTIAL_POSITION = "inertial_position"
EXPERIMENT_RUNS = {
    1: {
        "shadow_mode": ONBOARD_MIRROR,
        "vicon_mode": "pointcloud",
        "vicon_orientation_forwarded": False,
    },
    2: {
        "shadow_mode": INERTIAL_POSITION,
        "vicon_mode": "rigidbody",
        "vicon_orientation_forwarded": False,
    },
    3: {
        "shadow_mode": INERTIAL_POSITION,
        "vicon_mode": "rigidbody",
        "vicon_orientation_forwarded": True,
    },
}


def experiment_run_config(run):
    """Return the immutable protocol fields for flight ``run`` (1, 2, or 3)."""
    if isinstance(run, bool):
        raise ValueError("contact attitude experiment run must be 1, 2, or 3")
    try:
        run = int(run)
    except (TypeError, ValueError):
        raise ValueError(
            "contact attitude experiment run must be 1, 2, or 3"
        ) from None
    if run not in EXPERIMENT_RUNS:
        raise ValueError("contact attitude experiment run must be 1, 2, or 3")
    return dict(EXPERIMENT_RUNS[run])


def _require_mapping(parent, key, qualified_name):
    value = parent.get(key) if isinstance(parent, Mapping) else None
    if not isinstance(value, Mapping):
        raise ValueError(
            f"contact attitude experiment requires {qualified_name} mapping"
        )
    return value


def _require_positive_finite(mapping, key, qualified_name):
    value = mapping.get(key) if isinstance(mapping, Mapping) else None
    if isinstance(value, bool):
        value = None
    try:
        numeric = float(value)
    except (TypeError, ValueError):
        numeric = math.nan
    if not math.isfinite(numeric) or numeric <= 0.0:
        raise ValueError(
            f"contact attitude experiment requires {qualified_name} to be "
            "finite and positive"
        )
    return numeric


def _validate_physical_lifecycle(interaction, wrench):
    """Reject missions that would observe a different contact lifecycle."""
    if interaction.get("detection_method") != "momentum_impulse":
        raise ValueError(
            "contact attitude experiment requires "
            "Interaction.config.detection_method=momentum_impulse"
        )
    if wrench.get("state_source") != "onboard":
        raise ValueError(
            "contact attitude experiment requires "
            "wrench_interaction.state_source=onboard"
        )

    initial_contact_arming = _require_mapping(
        wrench,
        "initial_contact_arming",
        "wrench_interaction.initial_contact_arming",
    )
    if initial_contact_arming.get("enabled") is not True:
        raise ValueError(
            "contact attitude experiment requires initial_contact_arming."
            "enabled=true"
        )
    if initial_contact_arming.get("apply_after_each_interaction") is not True:
        raise ValueError(
            "contact attitude experiment requires initial_contact_arming."
            "apply_after_each_interaction=true"
        )
    arming_speed = _require_positive_finite(
        initial_contact_arming,
        "max_xy_speed_m_s",
        "wrench_interaction.initial_contact_arming.max_xy_speed_m_s",
    )
    arming_dwell = _require_positive_finite(
        initial_contact_arming,
        "stationary_dwell_s",
        "wrench_interaction.initial_contact_arming.stationary_dwell_s",
    )
    arming_gap = _require_positive_finite(
        initial_contact_arming,
        "max_sample_gap_s",
        "wrench_interaction.initial_contact_arming.max_sample_gap_s",
    )
    if arming_speed > 0.03:
        raise ValueError(
            "contact attitude experiment requires initial_contact_arming."
            "max_xy_speed_m_s no greater than 0.03"
        )
    if arming_dwell < 0.50:
        raise ValueError(
            "contact attitude experiment requires initial_contact_arming."
            "stationary_dwell_s at least 0.50"
        )
    if arming_gap > 0.10:
        raise ValueError(
            "contact attitude experiment requires initial_contact_arming."
            "max_sample_gap_s no greater than 0.10"
        )

    virtual_object = _require_mapping(
        interaction, "virtual_object", "Interaction.config.virtual_object"
    )
    contact = _require_mapping(
        virtual_object,
        "contact_detection",
        "Interaction.config.virtual_object.contact_detection",
    )
    release = _require_mapping(
        virtual_object,
        "release_behavior",
        "Interaction.config.virtual_object.release_behavior",
    )
    if contact.get("source") != "potentiometer":
        raise ValueError(
            "contact attitude experiment requires "
            "virtual_object.contact_detection.source=potentiometer"
        )
    if release.get("mode") != "potentiometer_coast":
        raise ValueError(
            "contact attitude experiment requires "
            "virtual_object.release_behavior.mode=potentiometer_coast"
        )

    contact_force = _require_positive_finite(
        contact,
        "force_threshold_n",
        "virtual_object.contact_detection.force_threshold_n",
    )
    _require_positive_finite(
        contact,
        "onset_dwell_s",
        "virtual_object.contact_detection.onset_dwell_s",
    )
    force_drop = _require_positive_finite(
        release,
        "force_drop_n",
        "virtual_object.release_behavior.force_drop_n",
    )
    _require_positive_finite(
        release,
        "decrease_rate_n_s",
        "virtual_object.release_behavior.decrease_rate_n_s",
    )
    unloaded_force = _require_positive_finite(
        release,
        "unloaded_force_n",
        "virtual_object.release_behavior.unloaded_force_n",
    )
    _require_positive_finite(
        release,
        "unloaded_dwell_s",
        "virtual_object.release_behavior.unloaded_dwell_s",
    )
    _require_positive_finite(
        release,
        "max_sample_gap_s",
        "virtual_object.release_behavior.max_sample_gap_s",
    )
    _require_positive_finite(
        release,
        "candidate_stall_timeout_s",
        "virtual_object.release_behavior.candidate_stall_timeout_s",
    )
    _require_positive_finite(
        release,
        "candidate_sensor_stale_timeout_s",
        "virtual_object.release_behavior.candidate_sensor_stale_timeout_s",
    )
    candidate_lead = release.get("candidate_lead_drop_n")
    if candidate_lead is not None:
        candidate_lead = _require_positive_finite(
            release,
            "candidate_lead_drop_n",
            "virtual_object.release_behavior.candidate_lead_drop_n",
        )
        if candidate_lead > force_drop:
            raise ValueError(
                "contact attitude experiment requires candidate_lead_drop_n "
                "to be no greater than force_drop_n"
            )
    if unloaded_force >= contact_force:
        raise ValueError(
            "contact attitude experiment requires release unloaded_force_n "
            "to be below contact force_threshold_n"
        )


def prepare_contact_attitude_experiment_mission(mission, run):
    """Return a private mission copy with the requested shadow mode enabled."""
    if not isinstance(mission, dict):
        raise ValueError("contact attitude experiment requires a mission mapping")
    protocol = experiment_run_config(run)
    prepared = deepcopy(mission)
    interaction_root = _require_mapping(
        prepared, "Interaction", "Interaction"
    )
    if interaction_root.get('action') != 'translation':
        raise ValueError(
            'contact attitude experiment requires Interaction.action=translation'
        )
    if interaction_root.get('iteration', 1) != 1:
        raise ValueError(
            'contact attitude experiment requires Interaction.iteration=1'
        )
    interaction = _require_mapping(
        interaction_root, "config", "Interaction.config"
    )
    wrench = _require_mapping(
        interaction,
        "wrench_interaction",
        "Interaction.config.wrench_interaction",
    )
    if not isinstance(wrench, dict):
        raise ValueError(
            "contact attitude experiment requires mutable "
            "Interaction.config.wrench_interaction mapping"
        )
    embedded_run = wrench.get('contact_attitude_experiment_run')
    if embedded_run is not None:
        experiment_run_config(embedded_run)
        if int(embedded_run) != int(run):
            raise ValueError(
                'mission contact_attitude_experiment_run does not match the '
                'requested experiment run'
            )
    _validate_physical_lifecycle(interaction, wrench)
    wrench.update({
        "contact_attitude_shadow_enabled": True,
        "contact_attitude_shadow_mode": protocol["shadow_mode"],
        "contact_attitude_experiment_run": int(run),
        "contact_attitude_vicon_mode": protocol["vicon_mode"],
        "contact_attitude_vicon_orientation_forwarded": (
            protocol["vicon_orientation_forwarded"]
        ),
    })
    return prepared


def validate_contact_attitude_cli(args):
    """Fail before arming if CLI localization does not match the protocol."""
    run = getattr(args, "contact_attitude_run", None)
    if run is None:
        return None
    protocol = experiment_run_config(run)
    if not getattr(args, "interaction", False):
        raise ValueError("--contact-attitude-run requires --interaction")
    if not getattr(args, "sense", False):
        raise ValueError("--contact-attitude-run requires --sense")
    if not getattr(args, "vicon", False):
        raise ValueError("--contact-attitude-run requires --vicon")
    if not getattr(args, "log", False):
        raise ValueError("--contact-attitude-run requires --log")
    if getattr(args, "save_vicon", False):
        raise ValueError(
            "--contact-attitude-run cannot use --save-vicon because the "
            "position input must be forwarded"
        )
    if getattr(args, "ground_test", False) or getattr(args, "droneless", False):
        raise ValueError("--contact-attitude-run is an airborne hardware protocol")
    if getattr(args, "vicon_mode", None) != protocol["vicon_mode"]:
        raise ValueError(
            f"contact attitude run {run} requires --vicon-mode "
            f"{protocol['vicon_mode']}"
        )
    forwarded = bool(getattr(args, "vicon_full_pose", False))
    if forwarded != protocol["vicon_orientation_forwarded"]:
        required = "with" if protocol["vicon_orientation_forwarded"] else "without"
        raise ValueError(
            f"contact attitude run {run} must launch {required} "
            "--vicon-full-pose"
        )
    if protocol["vicon_mode"] == "rigidbody" and not getattr(args, "obj_name", None):
        raise ValueError(
            f"contact attitude run {run} requires --obj-name for the "
            "rigidbody orientation evaluation channel"
        )
    return protocol
