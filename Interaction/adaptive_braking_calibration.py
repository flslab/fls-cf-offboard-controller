"""Calibration-only adapter that can only shorten an existing brake pulse.

The first opposed pair remains the original identification maneuver. At each
later pair's first trial start, freeze the latest earlier-trial candidate. No
model update can change an in-progress pair. A missing candidate leaves that
whole pair baseline, with an explicit event. A failed prediction levels and
latches instead of continuing an obsolete braking instruction.

The caller must validate/start the online worker before enabling this adapter,
call modify only for an admitted control cycle, and call record_sent only after
the actual send succeeds. The adapter performs no I/O except its event callback.
The controller CLI enables it by default for ``--calibrate`` and provides
``--no-adaptive-braking-calibration`` for fixed-pulse baseline collection.  It
is never an interaction or POSITION-controller adapter.
"""
from __future__ import annotations

import copy
from dataclasses import replace
import math

import numpy as np

from Interaction.model_based_braking import ModelBasedBrakingController


DEFAULTS = {"enabled": False, "target_distance_m": .30, "model_based_braking": {}}


def _finite(value, name):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(name + " must be a finite number")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(name + " must be a finite number")
    return result


class AdaptiveBrakingCalibration:
    """Modify only brake -> level timing; preserve recovery and trial deadline.

    State uses ModelBasedBrakingController's schema and a common host clock.
    target_distance_m is an explicit experimental goal *ahead of the measured
    brake-start position*, not a learned coasting distance or a POSITION target.
    """

    @staticmethod
    def validate_online_worker_config(online_config):
        if not isinstance(online_config, dict) or online_config.get("enabled") is not True:
            raise ValueError("adaptive braking calibration requires an enabled online prediction worker")

    def __init__(self, config, plan, ctrl_rate, log_event):
        if config is not None and not isinstance(config, dict):
            raise ValueError("adaptive braking calibration config must be a mapping")
        self.config = copy.deepcopy(DEFAULTS)
        self.config.update(copy.deepcopy(config or {}))
        if type(self.config["enabled"]) is not bool:
            raise ValueError("adaptive braking calibration enabled must be boolean")
        self.enabled = self.config["enabled"]
        self.plan = plan
        self.log_event = log_event
        self._pair_models = {}
        self._episodes = {}
        self._histories = {}
        self._last_send = {}
        self._sent_brake_count = {}
        self._brake_observations = {}
        self._last_modify = None
        self._latched = set()
        if not self.enabled:
            return
        self.ctrl_rate = _finite(ctrl_rate, "ctrl_rate")
        if not 10 <= self.ctrl_rate <= 500:
            raise ValueError("adaptive braking calibration ctrl_rate must be in [10, 500] Hz")
        self.dt = 1. / self.ctrl_rate
        self.target_distance_m = _finite(self.config["target_distance_m"], "target_distance_m")
        if not 0 < self.target_distance_m <= min(1., float(plan.max_displacement_m)):
            raise ValueError("adaptive target distance must be positive and inside the existing displacement limit")
        directions = np.asarray(plan.trial_directions, dtype=float)
        if (not plan.enabled or directions.ndim != 2 or directions.shape[1] != 2
                or len(directions) < 4 or len(directions) % 2
                or not np.isfinite(directions).all()
                or np.any(np.abs(directions[:, 0]) > 1e-6)
                or np.any(np.abs(np.abs(directions[:, 1])-1.) > 1e-6)
                or any(not np.allclose(directions[i], -directions[i+1])
                       for i in range(0, len(directions), 2))):
            raise ValueError("adaptive braking calibration requires at least two opposed world-Y pairs")
        durations = np.r_[plan.trial_brake_s, plan.trial_accelerate_s,
                          plan.level_before_acceleration_s, plan.level_before_brake_s,
                          plan.level_after_brake_s]
        if not np.isfinite(durations).all() or np.any(durations < 3*self.dt):
            raise ValueError("adaptive phases require at least three control periods for identification")
        options = self.config.get("model_based_braking")
        if not isinstance(options, dict):
            raise ValueError("model_based_braking must be a mapping")
        self.model_config = copy.deepcopy(options)
        # Experimental use does not grant approval to use this model in normal
        # interaction. Neither limits nor brake amplitude may exceed the plan.
        self.model_config.update(enabled=True, experimental_calibration=True,
                                 calibration_one_way_latch=True,
                                 brake_tilt_deg=float(plan.tilt_deg))
        requested_speed = _finite(self.model_config.get("max_xy_speed_m_s", plan.max_xy_speed_m_s),
                                  "max_xy_speed_m_s")
        if requested_speed > plan.max_xy_speed_m_s:
            raise ValueError("adaptive model cannot increase the existing speed limit")
        self.model_config["max_xy_speed_m_s"] = requested_speed
        requested_target = _finite(self.model_config.get("max_target_distance_m", plan.max_displacement_m),
                                   "max_target_distance_m")
        if requested_target > plan.max_displacement_m or requested_target < self.target_distance_m:
            raise ValueError("adaptive model target envelope must contain its target and respect the plan")
        self.model_config["max_target_distance_m"] = requested_target
        # Validate numerical controller options before any maneuver. An empty
        # model is deliberately unusable; option validation is still executed.
        ModelBasedBrakingController({}, target_position_xy=[0., self.target_distance_m],
            direction_xy=[0., 1.], brake_deadline_s=1., config=self.model_config)

    def _emit(self, name, data):
        if self.log_event is not None:
            self.log_event(name, data)

    def _freeze_pair(self, command, report):
        segment = command.segment_id
        pair = segment // 2
        if pair in self._pair_models:
            return
        candidate = None
        reason = "initial_opposed_pair_identification"
        if pair > 0:
            reason = "candidate_unavailable_at_pair_start"
            if segment % 2 or command.phase != "level_before_acceleration":
                reason = "pair_start_not_observed"
            elif isinstance(report, dict) and isinstance(report.get("candidate"), dict):
                source = report["candidate"]
                training = source.get("training_segment_ids")
                model = source.get("model")
                version = source.get("version")
                if (isinstance(training, list) and len(training) >= 2 and not len(training) % 2
                        and all(type(value) is int for value in training)
                        and training == list(range(segment))
                        and isinstance(model, dict) and model.get("train_segment_ids") == training
                        and type(version) is int and version > 0):
                    eligible = source.get("control_eligible", True)
                    tolerance = float(self.model_config.get(
                        "terminal_velocity_tolerance_m_s", .05
                    ))
                    directional = model.get("directional_models", {})
                    margins = [
                        component.get(
                            "terminal_velocity_error_margin_m_s", 0.
                        )
                        for component in directional.values()
                        if isinstance(component, dict)
                    ]
                    if (model.get("candidate_status") not in
                            (None, "requires_held_out_validation")):
                        reason = "candidate_fit_not_identifiable_or_at_bounds"
                    elif directional and set(directional) != {
                            "positive_y", "negative_y"}:
                        reason = "candidate_directional_model_set_invalid"
                    elif eligible is not True:
                        reason = source.get(
                            "control_eligibility_reason",
                            "candidate_failed_prior_held_out_validation",
                        )
                    elif margins and (any(
                            isinstance(value, bool)
                            or not isinstance(value, (int, float))
                            or not math.isfinite(value)
                            or value < 0 for value in margins)
                            or max(margins) >= tolerance):
                        reason = "candidate_uncertainty_exceeds_terminal_tolerance"
                    else:
                        candidate = copy.deepcopy(source)
                        reason = "earlier_trial_candidate_frozen"
                else:
                    reason = "candidate_provenance_invalid_or_not_causal"
        self._pair_models[pair] = candidate
        self._emit("Adaptive Braking Pair Frozen", {
            "pair_index": pair, "segment_ids": [pair*2, pair*2+1],
            "adaptive": candidate is not None, "reason": reason,
            "model_version": None if candidate is None else candidate["version"],
            "training_segment_ids": [] if candidate is None else candidate["training_segment_ids"],
            "experimental_target_distance_m": self.target_distance_m,
            "target_source": "explicit_brake_start_forward_distance",
            "matched_pulse_protocol": candidate is None,
        })

    @staticmethod
    def _level(command):
        return replace(command, phase="level_after_brake", command_acceleration_xy=np.zeros(2),
                       roll_deg=0., pitch_deg=0.)

    def modify(self, command, now_s, state, latest_report):
        """Return the scheduled command or a permanent early-level replacement."""
        if not self.enabled or command.segment_id is None or not command.active:
            return command
        now = _finite(now_s, "now_s")
        if self._last_modify is not None and now <= self._last_modify:
            raise ValueError("adaptive control times must strictly increase")
        self._last_modify = now
        segment = command.segment_id
        if type(segment) is not int or not 0 <= segment < len(self.plan.trial_directions):
            raise ValueError("adaptive segment is outside the original protocol")
        self._freeze_pair(command, latest_report)
        candidate = self._pair_models[segment // 2]
        if candidate is None or command.phase != "brake":
            return command
        if segment in self._latched:
            return self._level(command)
        if segment not in self._episodes:
            try:
                position = np.asarray(state["position_xy"], dtype=float)
                if position.shape != (2,) or not np.isfinite(position).all():
                    raise ValueError("invalid brake-start position")
                direction = np.asarray(command.direction_xy, dtype=float)
                target = position + self.target_distance_m*direction
                options = dict(self.model_config, brake_tilt_deg=float(self.plan.trial_tilt_levels_deg[segment]))
                episode = ModelBasedBrakingController(candidate["model"], target_position_xy=target,
                    direction_xy=direction, brake_deadline_s=now+float(self.plan.trial_brake_s[segment]),
                    config=options)
                for stamp, tilt in self._histories.get(segment, []):
                    episode.record_command(stamp, tilt)
            except (KeyError, TypeError, ValueError, OverflowError) as error:
                self._latched.add(segment)
                self._emit("Adaptive Braking Decision", {"segment_id": segment,
                    "model_version": candidate["version"], "action": "level", "reason": str(error),
                    "target_position_xy": None, "level_latched": True,
                    "identification_phase_completeness_not_guaranteed": True})
                return self._level(command)
            self._episodes[segment] = episode
            self._brake_observations[segment] = set()
            self._emit("Adaptive Braking Episode Started", {"segment_id": segment,
                "model_version": candidate["version"], "target_position_xy": target.tolist(),
                "direction_xy": direction.tolist(), "brake_started_at_s": now,
                "original_brake_deadline_s": episode.brake_deadline_s,
                "minimum_brake_observations": 2, "minimum_brake_periods": 2,
                "target_source": "explicit_brake_start_forward_distance",
                "position_command_target_modified": False})
        episode = self._episodes[segment]
        observations = self._brake_observations[segment]
        first_brake = episode.brake_deadline_s-float(self.plan.trial_brake_s[segment])
        try:
            stamp = _finite(state["time_s"], "state time")
            if first_brake < stamp <= now and len(observations) < 2:
                observations.add(stamp)
        except (KeyError, TypeError, ValueError):
            # The numerical controller below turns invalid state into level.
            pass
        # First allow enough actual brake/measurement cycles to retain the
        # five-phase system-identification contract. Never extend the deadline.
        required = (self._sent_brake_count.get(segment, 0) >= 2
                    and len(observations) >= 2 and now-first_brake >= 2*self.dt-1e-9)
        if now-first_brake < 2*self.dt-1e-9 and now < episode.brake_deadline_s:
            return command
        if now >= episode.brake_deadline_s:
            decision = {"action": "level", "reason": "original_brake_deadline",
                        "level_latched": True}
        elif not required:
            decision = {"action": "fallback", "reason": "insufficient_actual_brake_observations",
                        "level_latched": True}
        else:
            decision = episode.decide(now, state)
        if decision.get("action") not in ("brake", "level", "fallback"):
            decision = dict(decision, action="fallback", reason="invalid_controller_action")
        self._emit("Adaptive Braking Decision", dict(decision, segment_id=segment,
            model_version=candidate["version"], target_position_xy=episode.target_position_xy.tolist(),
            original_brake_deadline_s=episode.brake_deadline_s,
            experimental_target_distance_m=self.target_distance_m))
        if decision["action"] in ("level", "fallback"):
            self._latched.add(segment)
            return self._level(command)
        return command

    def record_sent(self, command, sent_at):
        """Record successful attitude sends; proposals never enter the model."""
        if (not self.enabled or command.segment_id is None or not command.active
                or not command.attitude_control):
            return
        segment = command.segment_id
        stamp = _finite(sent_at, "sent_at")
        if stamp <= self._last_send.get(segment, -math.inf):
            raise ValueError("actual sent command times must strictly increase")
        acceleration = np.asarray(command.command_acceleration_xy, dtype=float)
        direction = np.asarray(command.direction_xy, dtype=float)
        if acceleration.shape != (2,) or direction.shape != (2,) or not np.isfinite(np.r_[acceleration, direction]).all():
            raise ValueError("invalid actual command acceleration/direction")
        projected_tilt = math.atan(float(acceleration @ direction)/9.81)
        self._last_send[segment] = stamp
        history = self._histories.setdefault(segment, [])
        if not history or abs(projected_tilt-history[-1][1]) > 1e-12:
            history.append((stamp, projected_tilt))
            del history[:-64]
        if command.phase == "brake":
            self._sent_brake_count[segment] = self._sent_brake_count.get(segment, 0)+1
        if segment in self._episodes:
            self._episodes[segment].record_command(stamp, projected_tilt)
