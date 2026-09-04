"""Bounded experimental, receding-horizon attitude braking (no device I/O).

This module deliberately does not import the frozen RL/replay/model-fitting
chain. It consumes a *frozen* second-order fit, measured state and actual sent
commands. It does not fit parameters, authorize deployment, move the target or
claim that a POSITION controller has been identified.

Use is explicitly calibration-scoped. Its one-way mode may shorten the original
constant brake pulse, but cannot extend it or restart braking after leveling.
The caller must log/send a decision, then record_command only after that send
succeeds. A fallback contains NO command: the caller must level/abort the
experimental maneuver, not continue a stale brake instruction.
"""
from __future__ import annotations

import copy
import math
import time

import numpy as np
from scipy.linalg import expm


DEFAULTS = {
    "enabled": False,
    "experimental_calibration": False,
    "calibration_one_way_latch": True,
    "brake_tilt_deg": 20.,
    "prediction_horizon_s": .8,
    "prediction_step_s": .01,
    "candidate_pulse_s": (.02, .04, .08, .12),
    "candidate_refinement_step_s": .01,
    "command_valid_for_s": .03,
    "max_compute_s": .02,
    "max_state_age_s": .04,
    "max_state_group_skew_s": .03,
    "max_xy_speed_m_s": 1.6,
    "max_cross_axis_speed_m_s": .10,
    "max_tilt_deg": 29.,
    "max_projected_rate_rad_s": 5.,
    "max_target_distance_m": 1.,
    "max_battery_margin_V": .15,
    "reverse_tolerance_m_s": .02,
    "overshoot_tolerance_m": .03,
    "terminal_velocity_tolerance_m_s": .05,
    "terminal_tilt_tolerance_deg": 3.,
    "position_scale_m": .05,
    "velocity_scale_m_s": .08,
    "max_history_entries": 64,
}


def _number(value, name):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(name + " must be a finite number")
    value = float(value)
    if not math.isfinite(value):
        raise ValueError(name + " must be a finite number")
    return value


def _vector(value, size, name):
    result = np.asarray(value, dtype=float)
    if result.shape != (size,) or not np.isfinite(result).all():
        raise ValueError(name + " has an invalid shape or value")
    return result


def _validated_model(model, experimental):
    if (not isinstance(model, dict) or model.get("schema_version") != 1 or
            model.get("kind") != "delayed_second_order_planar_prediction" or
            model.get("prediction_scope") != "attitude_command_only"):
        raise ValueError("unsupported_model")
    if not experimental and not (
            model.get("deployment_approved") is True and
            model.get("independent_validation_complete") is True):
        raise ValueError("model_not_approved_for_nonexperimental_control")
    quality = model.get("identifiability", {})
    if (quality.get("identifiable") is not True or
            quality.get("bound_active_parameters") != []):
        raise ValueError("model_not_identifiable_or_at_bounds")
    fit = model.get("attitude_fit", {})
    if fit.get("model") != "second_order":
        raise ValueError("second_order_fit_required")
    bounds = {"delay_s": (0., .15), "wn_rad_s": (5., 100.),
              "zeta": (.2, 2.), "gain": (.7, 1.3),
              "bias_world_y_rad": (-.035, .035)}
    params = {}
    for key, (low, high) in bounds.items():
        value = _number(fit.get(key), key)
        if not low <= value <= high:
            raise ValueError("model_parameter_out_of_bounds:" + key)
        params[key] = value
    params["motion_gain"] = _number(model.get("motion_gain"), "motion_gain")
    if not .2 < params["motion_gain"] < 2.5:
        raise ValueError("model_motion_gain_out_of_bounds")
    ranges = model.get("data_ranges")
    if not isinstance(ranges, list) or not ranges:
        raise ValueError("model_missing_training_ranges")
    for row in ranges:
        if row.get("direction_y") not in (-1, 1):
            raise ValueError("model_direction_not_supported")
        for key in ("command_acceleration_m_s2", "velocity_m_s", "theta_rad", "battery_voltage_V"):
            interval = _vector(row.get(key), 2, key)
            if interval[0] > interval[1]:
                raise ValueError("reversed_training_range")
    return params, copy.deepcopy(ranges)


class ModelBasedBrakingController:
    """One fixed-target, fixed-direction braking episode; no mutable fit.

    State fields accepted by decide: time_s, position_xy, velocity_xy,
    orientation_rpy_rad, angular_velocity_rad_s, state_group_skew_s,
    battery_voltage_V. All times are the same host clock. Positive projected
    tilt/velocity is along direction_xy, which currently must be exactly ±Y.

    The numerical workload is at most 64 candidates × 369 time intervals,
    with at most 64 retained historical command transitions. Deadline/budget
    checks reject late results; this is not a hard-real-time scheduling claim.
    """

    def __init__(self, model, *, target_position_xy, direction_xy,
                 brake_deadline_s, config=None, clock=time.perf_counter):
        self.config = dict(DEFAULTS)
        self.config.update(config or {})
        self._clock = clock
        for key in ("enabled", "experimental_calibration", "calibration_one_way_latch"):
            if type(self.config[key]) is not bool:
                raise ValueError(key + " must be boolean")
        for key in set(DEFAULTS)-{"enabled", "experimental_calibration", "calibration_one_way_latch", "candidate_pulse_s"}:
            self.config[key] = _number(self.config[key], key)
            if self.config[key] <= 0:
                raise ValueError(key + " must be positive")
        if not 0 < self.config["brake_tilt_deg"] < 30:
            raise ValueError("brake_tilt_deg must be below 30")
        if not .1 <= self.config["prediction_horizon_s"] <= 1.5:
            raise ValueError("prediction horizon must be in [.1, 1.5] s")
        if not .005 <= self.config["prediction_step_s"] <= .02:
            raise ValueError("prediction step must be in [.005, .02] s")
        if not .01 <= self.config["candidate_refinement_step_s"] <= .02:
            raise ValueError(
                "candidate refinement step must be in [.01, .02] s"
            )
        if self.config["max_state_age_s"] > .1 or self.config["max_state_group_skew_s"] > .03:
            raise ValueError("state age/skew limits cannot exceed .1/.03 s")
        if self.config["max_tilt_deg"] >= 30:
            raise ValueError("max_tilt_deg must stay below 30")
        if self.config["terminal_velocity_tolerance_m_s"] > .10:
            raise ValueError(
                "terminal_velocity_tolerance_m_s cannot exceed .10"
            )
        if self.config["terminal_tilt_tolerance_deg"] > 5.:
            raise ValueError(
                "terminal_tilt_tolerance_deg cannot exceed 5"
            )
        if self.config["max_history_entries"] != int(self.config["max_history_entries"]) or not 2 <= self.config["max_history_entries"] <= 64:
            raise ValueError("max_history_entries must be an integer in [2,64]")
        pulses = tuple(_number(x, "candidate_pulse_s") for x in self.config["candidate_pulse_s"])
        if (not 1 <= len(pulses) <= 8 or any(not 0 < x <= .3 for x in pulses)
                or any(b <= a for a, b in zip(pulses, pulses[1:]))):
            raise ValueError("candidate pulses must be 1–8 increasing values in (0,.3]")
        self.config["candidate_pulse_s"] = pulses
        self.target_position_xy = _vector(target_position_xy, 2, "target_position_xy").copy()
        self.direction_xy = _vector(direction_xy, 2, "direction_xy").copy()
        if abs(self.direction_xy[0]) > 1e-6 or abs(abs(self.direction_xy[1])-1) > 1e-6:
            raise ValueError("only_unit_world_y_direction_supported")
        self.direction_xy = np.array([0., float(np.sign(self.direction_xy[1]))])
        self.target_position_xy.setflags(write=False)
        self.direction_xy.setflags(write=False)
        self.brake_deadline_s = _number(brake_deadline_s, "brake_deadline_s")
        self.model = copy.deepcopy(model)
        self.history = []
        self._last_sent_time = None
        self.level_latched = False
        self._last_decision_time = None
        self._model_error = None
        try:
            self.params, self.ranges = _validated_model(
                self.model, self.config["experimental_calibration"])
            selected = [r for r in self.ranges if r["direction_y"] == self.direction_xy[1]]
            if not selected:
                raise ValueError("requested_direction_was_not_identified")
            self.ranges = selected
            max_brake = max(-float(r["command_acceleration_m_s2"][0]) for r in selected)
            if 9.81*math.tan(math.radians(self.config["brake_tilt_deg"])) > max_brake+1e-6:
                raise ValueError("brake_tilt_exceeds_observed_command")
            wn, zeta = self.params["wn_rad_s"], self.params["zeta"]
            self._matrix = np.array([[0., 1.], [-wn*wn, -2*zeta*wn]])
        except (TypeError, ValueError, KeyError) as error:
            self._model_error = str(error)

    def record_command(self, time_s, projected_tilt_rad):
        """Record only a successful actual send, never a proposed command."""
        stamp = _number(time_s, "command time")
        tilt = _number(projected_tilt_rad, "command tilt")
        if abs(tilt) > math.radians(self.config["max_tilt_deg"]):
            raise ValueError("history_command_tilt_exceeds_limit")
        if self._last_sent_time is not None and stamp <= self._last_sent_time:
            raise ValueError("history_command_times_must_increase")
        self._last_sent_time = stamp
        # Consecutive identical commands carry no new actuator transition.
        if self.history and abs(tilt-self.history[-1][1]) < 1e-12:
            return
        self.history.append((stamp, tilt))
        count = int(self.config["max_history_entries"])
        if len(self.history) > count:
            self.history = self.history[-count:]

    def _result(self, action, reason, now, started, **details):
        result = dict(action=action, reason=reason, projected_tilt_rad=None,
                      roll_deg=None, pitch_deg=None, level_latched=self.level_latched,
                      target_position_xy=self.target_position_xy.tolist(),
                      valid_until_s=now+self.config["command_valid_for_s"],
                      compute_elapsed_s=max(0., self._clock()-started),
                      experimental_calibration=self.config["experimental_calibration"],
                      position_controller_identified=False, model_parameters_updated=False)
        result.update(details)
        return result

    def _state(self, state, now):
        stamp = _number(state["time_s"], "state time")
        if not 0 <= now-stamp <= self.config["max_state_age_s"]:
            raise ValueError("stale_or_future_state")
        skew = _number(state["state_group_skew_s"], "state group skew")
        if not 0 <= skew <= self.config["max_state_group_skew_s"]:
            raise ValueError("state_group_skew_exceeds_limit")
        position = _vector(state["position_xy"], 2, "position")
        velocity = _vector(state["velocity_xy"], 2, "velocity")
        rpy = _vector(state["orientation_rpy_rad"], 3, "orientation")
        omega = _vector(state["angular_velocity_rad_s"], 3, "angular velocity")
        if np.linalg.norm(velocity) > self.config["max_xy_speed_m_s"]:
            raise ValueError("speed_exceeds_safety_envelope")
        if abs(velocity[0]) > self.config["max_cross_axis_speed_m_s"]:
            raise ValueError("cross_axis_velocity_not_modelled")
        if max(abs(rpy[0]), abs(rpy[1])) > math.radians(self.config["max_tilt_deg"]):
            raise ValueError("tilt_exceeds_safety_envelope")
        if np.linalg.norm(self.target_position_xy-position) > self.config["max_target_distance_m"]:
            raise ValueError("target_outside_episode_envelope")
        cy, sy = math.cos(rpy[2]), math.sin(rpy[2])
        sign = self.direction_xy[1]
        theta = math.atan(-sign*(cy*math.tan(rpy[0])/math.cos(rpy[1])+sy*math.tan(rpy[1])))
        rate = -sign*(cy*omega[0]+sy*omega[1])
        if abs(rate) > self.config["max_projected_rate_rad_s"]:
            raise ValueError("angular_rate_exceeds_safety_envelope")
        pv = float(velocity @ self.direction_xy)
        battery = _number(state["battery_voltage_V"], "battery")
        margin = self.config["max_battery_margin_V"]
        if not min(r["battery_voltage_V"][0] for r in self.ranges)-margin <= battery <= max(r["battery_voltage_V"][1] for r in self.ranges)+margin:
            raise ValueError("battery_outside_identified_range")
        extrapolated = not (
            min(r["velocity_m_s"][0] for r in self.ranges) <= pv <= max(r["velocity_m_s"][1] for r in self.ranges)
            and min(r["theta_rad"][0] for r in self.ranges) <= theta <= max(r["theta_rad"][1] for r in self.ranges))
        if extrapolated and not self.config["experimental_calibration"]:
            raise ValueError("state_outside_identified_range")
        return stamp, float(position @ self.direction_xy), pv, theta, rate, float(rpy[2]), extrapolated

    def _forecast(self, state, now, durations):
        """Exact linear attitude transitions; trapezoid kinematics, bounded grid."""
        stamp, p, v, theta, rate, _yaw, _extra = state
        delay = self.params["delay_s"]
        if not self.history or self.history[0][0] > stamp-delay+1e-9:
            raise ValueError("insufficient_effective_command_history")
        if self._last_sent_time is not None and self._last_sent_time > now+1e-9:
            raise ValueError("future_command_in_sent_history")
        horizon = self.config["prediction_horizon_s"]
        dt = self.config["prediction_step_s"]
        end = now+horizon
        # Shift to a local clock before matrix/time calculations; epoch floats
        # otherwise obscure actual delayed transition coincidence.
        age = now-stamp
        history_t = np.array([t-stamp for t, _ in self.history])
        history_u = np.array([u for _, u in self.history])
        steps = int(math.ceil((age+horizon)/dt))
        if steps > 369:
            raise ValueError("prediction_grid_exceeds_bound")
        base = np.linspace(0., age+horizon, steps+1)
        events = np.r_[history_t+delay, age+delay, age+durations+delay]
        grid = np.unique(np.r_[base, events[(events > 0) & (events < age+horizon)]])
        if len(grid) > 370:
            raise ValueError("prediction_grid_exceeds_bound")
        count = len(durations)
        angle = np.full(count, theta); angular = np.full(count, rate)
        velocity = np.full(count, v); position = np.full(count, p)
        max_position = position.copy(); min_velocity = velocity.copy()
        zero_position = np.full(count, np.nan)
        last_acc = self.params["motion_gain"]*9.81*np.tan(angle)
        brake = -math.radians(self.config["brake_tilt_deg"])
        cache = {}
        for left, right in zip(grid[:-1], grid[1:]):
            effective = (left+right)/2-delay
            if effective < age:
                index = np.searchsorted(history_t, effective, side="right")-1
                if index < 0:
                    raise ValueError("missing_historical_input")
                commanded = np.full(count, history_u[index])
            else:
                commanded = np.where(effective < age+durations-1e-10, brake, 0.)
            equilibrium = self.params["gain"]*commanded + self.params["bias_world_y_rad"]*self.direction_xy[1]
            step = float(right-left)
            key = round(step, 10)
            if key not in cache:
                cache[key] = expm(self._matrix*step)
            transition = cache[key]
            relative = angle-equilibrium
            next_angle = transition[0, 0]*relative+transition[0, 1]*angular+equilibrium
            angular = transition[1, 0]*relative+transition[1, 1]*angular
            if not np.isfinite(next_angle).all() or np.any(np.abs(next_angle) > math.radians(self.config["max_tilt_deg"])):
                raise ValueError("predicted_tilt_exceeds_envelope")
            acceleration = self.params["motion_gain"]*9.81*np.tan(next_angle)
            next_velocity = velocity+.5*(last_acc+acceleration)*step
            next_position = position+.5*(velocity+next_velocity)*step
            crossing = np.isnan(zero_position) & (velocity > 0) & (next_velocity <= 0)
            if crossing.any():
                fraction = velocity[crossing]/(velocity[crossing]-next_velocity[crossing])
                zero_position[crossing] = position[crossing] + .5*velocity[crossing]*step*fraction
            max_position = np.maximum(max_position, next_position)
            min_velocity = np.minimum(min_velocity, next_velocity)
            angle, velocity, position, last_acc = next_angle, next_velocity, next_position, acceleration
        if not np.isfinite(np.r_[position, velocity, min_velocity, max_position]).all():
            raise ValueError("nonfinite_prediction")
        return dict(position=position, velocity=velocity, angle=angle,
                    min_velocity=min_velocity,
                    max_position=max_position, zero_position=zero_position,
                    candidate_count=count, integration_steps=len(grid)-1,
                    prediction_end_time_s=end)

    def decide(self, now_s, state):
        started = self._clock()
        now = _number(now_s, "now_s")
        if not self.config["enabled"]:
            return self._result("fallback", "disabled", now, started)
        if self._model_error:
            return self._result("fallback", self._model_error, now, started)
        if self._last_decision_time is not None and now <= self._last_decision_time:
            return self._result("fallback", "nonincreasing_decision_time", now, started)
        self._last_decision_time = now
        try:
            observed = self._state(state, now)
            remaining = max(0., self.brake_deadline_s-now)
            durations = np.unique(np.r_[
                0.,
                np.minimum(self.config["candidate_pulse_s"], remaining),
                remaining,
            ])
            if self.level_latched or remaining <= 0:
                durations = np.array([0.])
            refined = False
            if len(durations) > 1:
                step = self.config["candidate_refinement_step_s"]
                fine = np.arange(step, remaining, step)
                durations = np.unique(np.r_[durations, fine])
                refined = len(fine) > 0
            if len(durations) > 64:
                raise ValueError("candidate_grid_exceeds_bound")
            # One vectorized forecast avoids repeating the time integration;
            # candidate count changes array width, not the number of steps.
            forecast = self._forecast(observed, now, durations)
        except (KeyError, TypeError, ValueError, OverflowError) as error:
            return self._result("fallback", str(error), now, started)
        elapsed = self._clock()-started
        if elapsed > self.config["max_compute_s"]:
            return self._result("fallback", "prediction_compute_budget_exceeded", now, started,
                                candidate_count=forecast["candidate_count"])
        target = float(self.target_position_xy @ self.direction_xy)
        # Position and zero-crossing error are separate from reverse motion.
        # A predicted zero is not declared a stable stop; all tail velocities
        # remain in the objective. Undershoot is preferable to reversing.
        endpoint = np.where(np.isfinite(forecast["zero_position"]), forecast["zero_position"], forecast["position"])
        reverse = np.maximum(0., -forecast["min_velocity"])
        overshoot = np.maximum(0., forecast["max_position"]-target)
        terminal_tilt_deg = np.degrees(forecast["angle"])
        terminal_velocity_ok = (
            np.abs(forecast["velocity"])
            <= self.config["terminal_velocity_tolerance_m_s"]
        )
        terminal_tilt_ok = (
            np.abs(terminal_tilt_deg)
            <= self.config["terminal_tilt_tolerance_deg"]
        )
        no_reverse = reverse <= self.config["reverse_tolerance_m_s"]
        feasible = no_reverse & terminal_velocity_ok & terminal_tilt_ok
        score = ((endpoint-target)/self.config["position_scale_m"])**2
        score += (forecast["velocity"]/self.config["velocity_scale_m_s"])**2
        score += 100.*(reverse/self.config["reverse_tolerance_m_s"])**2
        score += 25.*(overshoot/self.config["overshoot_tolerance_m"])**2
        # If every candidate reverses, choose level to remove commanded
        # braking. Return a visible degraded state, not "safe" or a hidden
        # fallback to the same excessive pulse. Existing measured reverse also
        # always levels: this component never accelerates toward a past target.
        if observed[2] <= 0 or remaining <= 0 or self.level_latched:
            selected = 0
            reason = "already_level_latched" if self.level_latched else (
                "original_brake_deadline" if remaining <= 0 else "measured_nonpositive_velocity")
        elif not np.any(no_reverse):
            selected, reason = 0, "unavoidable_predicted_reverse_level_to_remove_brake"
        elif not np.any(feasible):
            selected = 0
            reason = (
                "no_candidate_satisfies_terminal_state_constraints_"
                "level_to_remove_brake"
            )
        else:
            selected = int(np.argmin(np.where(feasible, score, np.inf)))
            reason = "rolling_prediction_selected_level" if durations[selected] == 0 else "rolling_prediction_continue_brake"
        tilt = -math.radians(self.config["brake_tilt_deg"]) if durations[selected] > 0 else 0.
        if tilt == 0 and self.config["calibration_one_way_latch"]:
            self.level_latched = True
        yaw, sign = observed[5], self.direction_xy[1]
        tilt_deg = math.degrees(tilt)
        # Identical world acceleration -> body attitude convention to the
        # calibration sender: +world Y implies negative roll at yaw=0.
        roll = float(-tilt_deg*sign*math.cos(yaw))
        pitch = float(-tilt_deg*sign*math.sin(yaw))
        return self._result("brake" if tilt else "level", reason, now, started,
            projected_tilt_rad=tilt, roll_deg=roll, pitch_deg=pitch,
            level_latched=self.level_latched, candidate_count=forecast["candidate_count"],
            selected_pulse_s=float(durations[selected]), integration_steps=forecast["integration_steps"],
            predicted_terminal_velocity_m_s=float(forecast["velocity"][selected]),
            predicted_terminal_tilt_deg=float(terminal_tilt_deg[selected]),
            predicted_peak_reverse_m_s=float(reverse[selected]),
            predicted_overshoot_m=float(overshoot[selected]),
            predicted_first_zero_position_m=(None if not math.isfinite(forecast["zero_position"][selected]) else float(forecast["zero_position"][selected])),
            predicted_endpoint_position_m=float(forecast["position"][selected]),
            target_projected_position_m=target,
            state_extrapolates_training_range=observed[6],
            prediction_horizon_s=self.config["prediction_horizon_s"],
            no_reverse_prediction=bool(no_reverse[selected]),
            terminal_velocity_tolerance_m_s=self.config[
                "terminal_velocity_tolerance_m_s"
            ],
            terminal_tilt_tolerance_deg=self.config[
                "terminal_tilt_tolerance_deg"
            ],
            terminal_velocity_constraint_satisfied=bool(
                terminal_velocity_ok[selected]
            ),
            terminal_tilt_constraint_satisfied=bool(
                terminal_tilt_ok[selected]
            ),
            hard_terminal_constraints_satisfied=bool(feasible[selected]),
            hard_feasible_candidate_count=int(np.count_nonzero(feasible)),
            terminal_candidate_grid_refined=refined,
            model_train_segment_ids=copy.deepcopy(self.model.get("train_segment_ids", [])))
