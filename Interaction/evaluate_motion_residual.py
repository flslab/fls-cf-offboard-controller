"""Offline evaluation of bounded, rolling motion-residual correction.

This tool never writes a flight calibration and never sends a command.  Every
reported forecast is made for a held-out opposed-direction trial pair.  At an
anchor it may use measured state and sent commands at or before that anchor;
future measured state and future commands are used only afterwards as labels.

Run with::

    python -m Interaction.evaluate_motion_residual LOG.json --output NEW_DIR
"""
from __future__ import annotations

import argparse
import csv
from dataclasses import asdict, dataclass
import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.integrate import cumulative_trapezoid

from Interaction.braking_replay import extract
from Interaction.braking_split_diagnostic import (
    TiltTrial,
    build_trials,
    fit_tilt,
    motion_gain,
    second_homogeneous,
)
from Interaction.model_based_braking import RollingMotionResidualObserver


@dataclass(frozen=True)
class ResidualConfig:
    """Fixed, predeclared settings for the offline comparison."""

    history_window_s: float = 0.08
    minimum_history_s: float = 0.06
    maximum_sample_gap_s: float = 0.04
    filter_time_constant_s: float = 0.08
    maximum_residual_m_s2: float = 1.5
    sigma_floor_m_s2: float = 0.15
    sigma_multiplier: float = 2.0
    residual_apply_horizon_s: float = 0.08
    minimum_samples: int = 4
    maximum_samples: int = 64
    anchor_spacing_s: float = 0.05
    horizons_s: tuple[float, ...] = (0.05, 0.10, 0.20)
    reverse_threshold_m_s: float = -0.03

    def validate(self) -> None:
        values = (self.history_window_s, self.minimum_history_s,
                  self.maximum_sample_gap_s, self.filter_time_constant_s,
                  self.maximum_residual_m_s2, self.sigma_floor_m_s2,
                  self.sigma_multiplier, self.residual_apply_horizon_s,
                  self.anchor_spacing_s, self.reverse_threshold_m_s,
                  *self.horizons_s)
        if not np.isfinite(values).all():
            raise ValueError("residual-evaluation settings must be finite")
        if min(self.history_window_s, self.minimum_history_s,
               self.maximum_sample_gap_s, self.filter_time_constant_s,
               self.maximum_residual_m_s2, self.sigma_floor_m_s2,
               self.sigma_multiplier, self.residual_apply_horizon_s,
               self.anchor_spacing_s, *self.horizons_s) <= 0:
            raise ValueError("windows, limit, spacing, and horizons must be positive")
        if self.minimum_history_s > self.history_window_s:
            raise ValueError("minimum history exceeds the residual window")
        if self.maximum_sample_gap_s > self.minimum_history_s:
            raise ValueError("maximum sample gap exceeds minimum history")
        if self.residual_apply_horizon_s > max(self.horizons_s):
            raise ValueError("residual apply horizon exceeds all evaluated horizons")
        if (isinstance(self.minimum_samples, bool)
                or int(self.minimum_samples) != self.minimum_samples
                or not 3 <= self.minimum_samples <= 32):
            raise ValueError("minimum samples must be an integer in [3,32]")
        if (isinstance(self.maximum_samples, bool)
                or int(self.maximum_samples) != self.maximum_samples
                or not self.minimum_samples <= self.maximum_samples <= 128):
            raise ValueError("maximum samples is invalid")
        if not 0.05 <= self.residual_apply_horizon_s <= 0.10:
            raise ValueError("residual apply horizon must be in [0.05,0.10] s")
        if self.maximum_residual_m_s2 > 5.0:
            raise ValueError("maximum residual cannot exceed 5 m/s^2")
        if self.sigma_multiplier > 4.0:
            raise ValueError("sigma multiplier cannot exceed 4")
        if tuple(sorted(set(self.horizons_s))) != self.horizons_s:
            raise ValueError("forecast horizons must be unique and increasing")
        if self.reverse_threshold_m_s >= 0:
            raise ValueError("reverse threshold must be negative")


def opposed_trial_pairs(items: list[TiltTrial]) -> list[tuple[TiltTrial, TiltTrial]]:
    """Pair adjacent calibration segments and verify equal-duration opposition."""
    ordered = sorted(items, key=lambda item: item.trial.segment)
    if len(ordered) < 6 or len(ordered) % 2:
        raise ValueError("leave-pair-out evaluation needs at least three complete trial pairs")
    if [item.trial.segment for item in ordered] != list(range(len(ordered))):
        raise ValueError("trial segment ids must be contiguous from zero")
    pairs = []
    for first, second in zip(ordered[::2], ordered[1::2]):
        if not np.isclose(first.trial.duration, second.trial.duration, atol=1e-9):
            raise ValueError("opposed trials in a pair must use the same pulse duration")
        if float(first.trial.direction @ second.trial.direction) > -0.95:
            raise ValueError("adjacent trials are not opposed directions")
        pairs.append((first, second))
    return pairs


def leave_pair_out_fits(items: list[TiltTrial], progress=lambda *args, **kwargs: None) -> list[dict]:
    """Fit dynamics without either member of the pair being evaluated."""
    pairs = opposed_trial_pairs(items)
    folds = []
    for held_pair in pairs:
        held_ids = [item.trial.segment for item in held_pair]
        training = [item for item in items if item.trial.segment not in held_ids]
        progress(f"Motion residual evaluation: hold out {held_ids}", flush=True)
        directional_models = {}
        for sign, label in ((1, "positive_y"), (-1, "negative_y")):
            selected = [item for item in training
                        if int(np.sign(item.trial.direction[1])) == sign]
            if len(selected) < 2:
                raise ValueError("each directional fold needs at least two training trials")
            # The bounded optimizer may probe explosive second-order parameters
            # before rejecting them.  The selected finite fit is checked below.
            with np.errstate(over="ignore", invalid="ignore", divide="ignore"):
                tilt_fit = fit_tilt(selected, "second_order")
            gain = motion_gain(selected)
            fit_values = [value for key, value in tilt_fit.items()
                          if key in {"delay_s", "wn_rad_s", "zeta", "gain",
                                     "bias_world_y_rad", "train_rmse_deg"}]
            if not np.isfinite([*fit_values, gain]).all():
                raise ValueError("leave-pair-out fit returned nonfinite parameters")
            directional_models[label] = dict(
                direction_y=sign,
                training_segments=[item.trial.segment for item in selected],
                tilt_fit=tilt_fit,
                motion_gain=float(gain),
            )
        folds.append(dict(
            held_segments=held_ids,
            training_segments=[item.trial.segment for item in training],
            model_scope="direction_specific_components",
            directional_models=directional_models,
        ))
    return folds


def _step_response(times: np.ndarray, fit: dict) -> np.ndarray:
    return 1.0 + second_homogeneous(
        times, -1.0, 0.0, float(fit["wn_rad_s"]), float(fit["zeta"])
    )


def causal_tilt_forecast(
    item: TiltTrial,
    fit: dict,
    anchor: int,
    elapsed_s: np.ndarray,
) -> np.ndarray:
    """Predict tilt from anchor state and commands already sent at the anchor.

    Delayed effects of already-sent commands are retained.  Commands sent after
    the anchor are deliberately invisible, even if present in the log.
    """
    trial = item.trial
    elapsed_s = np.asarray(elapsed_s, dtype=float)
    if (anchor < 0 or anchor >= len(trial.times) or elapsed_s.ndim != 1
            or len(elapsed_s) == 0 or elapsed_s[0] != 0
            or not np.isfinite(elapsed_s).all() or np.any(np.diff(elapsed_s) < 0)):
        raise ValueError("invalid causal forecast anchor or elapsed-time grid")
    t0 = float(trial.times[anchor])
    sent = trial.command_times <= t0 + 1e-12
    sent_times = trial.command_times[sent]
    sent_commands = trial.commands[sent]
    if len(sent_times) == 0:
        sent_times = np.array([t0])
        sent_commands = np.array([0.0])
    delayed = sent_times + float(fit["delay_s"])
    commands = float(fit["gain"]) * np.arctan(sent_commands / 9.81)
    bias = float(fit["bias_world_y_rad"]) * float(trial.direction[1])

    active = int(np.searchsorted(delayed, t0, side="right") - 1)
    active_target = (float(commands[active]) if active >= 0 else 0.0) + bias
    prediction = active_target + second_homogeneous(
        elapsed_s,
        float(item.angle[anchor]) - active_target,
        float(item.rate[anchor]),
        float(fit["wn_rad_s"]),
        float(fit["zeta"]),
    )

    # Only delayed transitions belonging to commands that had already been sent
    # may occur after the anchor.  The most recently sent target is then held.
    for index, effective_time in enumerate(delayed):
        if effective_time <= t0 + 1e-12:
            continue
        relative = float(effective_time - t0)
        mask = elapsed_s >= relative
        previous = float(commands[index - 1]) if index else 0.0
        change = float(commands[index] - previous)
        prediction[mask] += change * _step_response(elapsed_s[mask] - relative, fit)
    return prediction


def _new_residual_observer(config: ResidualConfig) -> RollingMotionResidualObserver:
    return RollingMotionResidualObserver(
        window_s=config.history_window_s,
        min_window_s=config.minimum_history_s,
        max_sample_gap_s=config.maximum_sample_gap_s,
        filter_tau_s=config.filter_time_constant_s,
        max_abs_accel_m_s2=config.maximum_residual_m_s2,
        sigma_floor_m_s2=config.sigma_floor_m_s2,
        min_samples=config.minimum_samples,
        max_samples=config.maximum_samples,
    )


def residual_snapshots(
    item: TiltTrial,
    motion_scale: float,
    config: ResidualConfig,
) -> dict[int, dict]:
    """Replay the live observer over real samples beginning at brake onset.

    No sample is interpolated at the 80 ms window boundary.  Duplicate, gapped,
    insufficient, and outlying observations therefore have exactly the same
    reset/rejection semantics as :class:`RollingMotionResidualObserver`.
    """
    observer = _new_residual_observer(config)
    trial = item.trial
    first = int(np.searchsorted(trial.times, trial.phase_times["brake"], side="left"))
    snapshots = {}
    for index in range(first, len(trial.times)):
        snapshots[index] = observer.update(
            float(trial.times[index]),
            float(trial.velocities[index]),
            float(item.angle[index]),
            float(motion_scale),
        )
    return snapshots


def rolling_motion_residual(
    item: TiltTrial,
    motion_scale: float,
    anchor: int,
    config: ResidualConfig,
) -> dict:
    """Return the exact live-observer snapshot at a historical anchor."""
    try:
        return residual_snapshots(item, motion_scale, config)[anchor]
    except KeyError as exc:
        raise ValueError("residual anchor precedes brake onset") from exc


def forecast_at_anchor(
    item: TiltTrial,
    tilt_fit: dict,
    motion_scale: float,
    anchor: int,
    horizon_s: float,
    config: ResidualConfig,
    residual_snapshot: dict | None = None,
) -> dict:
    """Return baseline/corrected trajectories without reading future state."""
    if not np.isfinite(horizon_s) or horizon_s <= 0:
        raise ValueError("forecast horizon must be positive and finite")
    trial = item.trial
    t0 = float(trial.times[anchor])
    # A denser deterministic grid prevents telemetry cadence from dominating
    # short-horizon numerical integration.  It contains no future state values.
    dt = min(0.005, horizon_s / 20.0)
    elapsed = np.arange(0.0, horizon_s, dt)
    elapsed = np.r_[elapsed, horizon_s]
    angle = causal_tilt_forecast(item, tilt_fit, anchor, elapsed)
    acceleration = motion_scale * 9.81 * np.tan(angle)
    initial_velocity = float(trial.velocities[anchor])
    baseline_velocity = initial_velocity + cumulative_trapezoid(acceleration, elapsed, initial=0.0)
    baseline_position = cumulative_trapezoid(baseline_velocity, elapsed, initial=0.0)
    residual = (rolling_motion_residual(item, motion_scale, anchor, config)
                if residual_snapshot is None else dict(residual_snapshot))
    correction = (float(residual["filtered_acceleration_m_s2"])
                  if residual["ready"] else 0.0)
    exposure = np.minimum(elapsed, config.residual_apply_horizon_s)
    corrected_velocity = baseline_velocity + correction * exposure
    corrected_position = baseline_position + correction * np.where(
        elapsed <= config.residual_apply_horizon_s,
        0.5 * elapsed ** 2,
        config.residual_apply_horizon_s * elapsed
        - 0.5 * config.residual_apply_horizon_s ** 2,
    )
    sigma = (float(residual["sigma_acceleration_m_s2"])
             if residual["ready"] else 0.0)
    uncertainty_velocity = config.sigma_multiplier * sigma * exposure
    return dict(
        anchor_s=t0,
        horizon_s=float(horizon_s),
        residual_horizon_anchor="measured_state_timestamp",
        residual_valid_until_s=t0 + config.residual_apply_horizon_s,
        elapsed_s=elapsed,
        predicted_tilt_rad=angle,
        baseline_velocity_m_s=baseline_velocity,
        baseline_position_m=baseline_position,
        corrected_velocity_m_s=corrected_velocity,
        corrected_position_m=corrected_position,
        corrected_velocity_lower_bound_m_s=corrected_velocity - uncertainty_velocity,
        corrected_velocity_upper_bound_m_s=corrected_velocity + uncertainty_velocity,
        residual_velocity_uncertainty_m_s=uncertainty_velocity,
        residual=residual,
    )


def _actual_labels(item: TiltTrial, anchor: int, horizon_s: float) -> dict:
    """Read post-anchor state only after prediction, exclusively for scoring."""
    trial = item.trial
    t0, end = float(trial.times[anchor]), float(trial.times[anchor] + horizon_s)
    within = (trial.times > t0) & (trial.times < end)
    times = np.r_[t0, trial.times[within], end]
    velocity = np.interp(times, trial.times, trial.velocities)
    position = np.interp(times, trial.times, trial.positions) - float(trial.positions[anchor])
    return dict(times_s=times, velocity_m_s=velocity, position_m=position)


def _anchors(item: TiltTrial, config: ResidualConfig) -> list[int]:
    trial = item.trial
    first_time = max(
        float(trial.phase_times["brake"]),
        float(trial.times[0] + min(0.025, 0.5 * config.history_window_s)),
    )
    candidates = np.flatnonzero(trial.times >= first_time)
    result, last_time = [], -np.inf
    for index in candidates:
        # Once measured velocity is already beyond the reverse threshold there
        # is no prospective reversal left to predict at this anchor.
        if trial.velocities[index] < config.reverse_threshold_m_s:
            continue
        if trial.times[index] - last_time >= config.anchor_spacing_s - 1e-12:
            result.append(int(index))
            last_time = float(trial.times[index])
    return result


def _unseen_command_within(trial, anchor_time: float, horizon_s: float) -> bool:
    future = trial.command_times[trial.command_times > anchor_time + 1e-12]
    return bool(len(future) and future[0] <= anchor_time + horizon_s + 1e-12)


def _score_forecast(item, anchor, horizon, forecast, config) -> list[dict]:
    labels = _actual_labels(item, anchor, horizon)
    actual_v = labels["velocity_m_s"]
    actual_p = labels["position_m"]
    actual_reverse = bool(np.min(actual_v) < config.reverse_threshold_m_s)
    rows = []
    for model in ("baseline", "bounded_residual"):
        predicted_v = forecast[f"{model if model == 'baseline' else 'corrected'}_velocity_m_s"]
        predicted_p = forecast[f"{model if model == 'baseline' else 'corrected'}_position_m"]
        predicted_reverse = bool(np.min(predicted_v) < config.reverse_threshold_m_s)
        if model == "bounded_residual":
            lower = forecast["corrected_velocity_lower_bound_m_s"]
            upper = forecast["corrected_velocity_upper_bound_m_s"]
        else:
            lower = upper = predicted_v
        conservative_reverse = bool(np.min(lower) < config.reverse_threshold_m_s)
        terminal_covered = (bool(lower[-1] <= actual_v[-1] <= upper[-1])
                            if model == "bounded_residual" else None)
        residual = forecast["residual"]
        rows.append(dict(
            segment=item.trial.segment,
            direction_xy=item.trial.direction.tolist(),
            pulse_duration_s=item.trial.duration,
            anchor_s=forecast["anchor_s"],
            horizon_s=float(horizon),
            model=model,
            initial_velocity_m_s=float(item.trial.velocities[anchor]),
            actual_terminal_velocity_m_s=float(actual_v[-1]),
            predicted_terminal_velocity_m_s=float(predicted_v[-1]),
            velocity_error_m_s=float(predicted_v[-1] - actual_v[-1]),
            actual_displacement_m=float(actual_p[-1]),
            predicted_displacement_m=float(predicted_p[-1]),
            position_error_m=float(predicted_p[-1] - actual_p[-1]),
            actual_reverse=actual_reverse,
            predicted_reverse=predicted_reverse,
            conservative_predicted_reverse=conservative_reverse,
            reversal_classification_correct=actual_reverse == predicted_reverse,
            conservative_reversal_classification_correct=(
                actual_reverse == conservative_reverse
            ),
            terminal_velocity_inside_uncertainty=terminal_covered,
            terminal_velocity_uncertainty_m_s=float(
                forecast["residual_velocity_uncertainty_m_s"][-1]
            ),
            residual_ready=bool(residual["ready"]),
            residual_status=residual["status"],
            residual_sample_count=int(residual["sample_count"]),
            residual_window_span_s=float(residual["window_span_s"]),
            raw_residual_m_s2=residual["raw_acceleration_m_s2"],
            filtered_residual_m_s2=float(residual["filtered_acceleration_m_s2"]),
            residual_sigma_m_s2=float(residual["sigma_acceleration_m_s2"]),
            residual_rejected=bool(residual["rejected"]),
            residual_clipped=bool(residual["clipped"]),
            residual_apply_horizon_s=config.residual_apply_horizon_s,
            residual_valid_until_s=forecast["residual_valid_until_s"],
            residual_horizon_anchor=forecast["residual_horizon_anchor"],
        ))
    return rows


def summarize(rows: list[dict], horizons: tuple[float, ...]) -> dict:
    summary = {}
    for horizon in horizons:
        key = f"{horizon:.2f}"
        summary[key] = {}
        for model in ("baseline", "bounded_residual"):
            selected = [row for row in rows
                        if row["horizon_s"] == horizon and row["model"] == model]
            if not selected:
                summary[key][model] = dict(count=0)
                continue
            velocity_errors = np.array([row["velocity_error_m_s"] for row in selected])
            position_errors = np.array([row["position_error_m"] for row in selected])
            coverage = [
                row["terminal_velocity_inside_uncertainty"] for row in selected
                if (row["terminal_velocity_inside_uncertainty"] is not None
                    and row["residual_ready"])
            ]
            summary[key][model] = dict(
                count=len(selected),
                velocity_mae_m_s=float(np.mean(abs(velocity_errors))),
                velocity_rmse_m_s=float(np.sqrt(np.mean(velocity_errors ** 2))),
                velocity_max_abs_error_m_s=float(np.max(abs(velocity_errors))),
                position_mae_m=float(np.mean(abs(position_errors))),
                position_rmse_m=float(np.sqrt(np.mean(position_errors ** 2))),
                position_max_abs_error_m=float(np.max(abs(position_errors))),
                actual_reversal_count=sum(row["actual_reverse"] for row in selected),
                predicted_reversal_count=sum(row["predicted_reverse"] for row in selected),
                missed_reversal_count=sum(row["actual_reverse"] and not row["predicted_reverse"]
                                           for row in selected),
                false_reversal_count=sum(not row["actual_reverse"] and row["predicted_reverse"]
                                          for row in selected),
                reversal_accuracy=float(np.mean(
                    [row["reversal_classification_correct"] for row in selected]
                )),
                conservative_missed_reversal_count=sum(
                    row["actual_reverse"] and not row["conservative_predicted_reverse"]
                    for row in selected
                ),
                conservative_false_reversal_count=sum(
                    not row["actual_reverse"] and row["conservative_predicted_reverse"]
                    for row in selected
                ),
                terminal_velocity_uncertainty_coverage=(
                    float(np.mean(coverage)) if coverage else None
                ),
                residual_ready_count=sum(row["residual_ready"] for row in selected),
                residual_rejected_count=sum(row["residual_rejected"] for row in selected),
                residual_clipped_count=sum(row["residual_clipped"] for row in selected),
            )
    return summary


def analyze(records: list[dict], config: ResidualConfig | None = None, progress=print) -> dict:
    config = config or ResidualConfig()
    config.validate()
    saved, trials, _, metadata = extract(records)
    if saved.get("repeat_test"):
        raise ValueError("a fitted calibration log is required for leave-pair-out evaluation")
    items = build_trials(records, trials)
    folds = leave_pair_out_fits(items, progress)
    by_segment = {item.trial.segment: item for item in items}
    rows = []
    skipped_unseen_command = {f"{horizon:.2f}": 0 for horizon in config.horizons_s}
    skipped_end_of_log = {f"{horizon:.2f}": 0 for horizon in config.horizons_s}
    for fold in folds:
        for segment in fold["held_segments"]:
            item = by_segment[segment]
            label = "positive_y" if item.trial.direction[1] > 0 else "negative_y"
            component = fold["directional_models"][label]
            observer_history = residual_snapshots(
                item, component["motion_gain"], config
            )
            for anchor in _anchors(item, config):
                anchor_time = float(item.trial.times[anchor])
                for horizon in config.horizons_s:
                    key = f"{horizon:.2f}"
                    if anchor_time + horizon > item.trial.times[-1] + 1e-12:
                        skipped_end_of_log[key] += 1
                        continue
                    # Fair causal comparison: do not score across a command that
                    # had not yet been sent when the forecast was initialized.
                    if _unseen_command_within(item.trial, anchor_time, horizon):
                        skipped_unseen_command[key] += 1
                        continue
                    forecast = forecast_at_anchor(
                        item, component["tilt_fit"], component["motion_gain"],
                        anchor, horizon, config, observer_history[anchor]
                    )
                    scored = _score_forecast(item, anchor, horizon, forecast, config)
                    for row in scored:
                        row["selected_model_component"] = label
                    rows.extend(scored)
    if not rows:
        raise ValueError("no uncontaminated causal forecast windows were available")
    segment_summary = {}
    for segment in sorted(by_segment):
        selected = [row for row in rows if row["segment"] == segment]
        item = by_segment[segment]
        segment_summary[str(segment)] = dict(
            direction_xy=item.trial.direction.tolist(),
            pulse_duration_s=item.trial.duration,
            horizons=summarize(selected, config.horizons_s),
        )
    ready_rows = [row for row in rows if row["residual_ready"]]
    return dict(
        offline_only=True,
        evaluation_status="exploratory_only_not_runtime_eligible",
        control_eligibility_granted=False,
        eligibility_evaluated=False,
        calibration_writes=False,
        flight_commands=False,
        causality_contract=(
            "Each held-pair forecast uses model fits from other pairs plus measured state and "
            "sent commands through its anchor. Future states are labels only. Windows crossing "
            "a command not yet sent at the anchor are excluded."
        ),
        config=asdict(config),
        runtime_semantics=dict(
            residual_observer=(
                "Interaction.model_based_braking.RollingMotionResidualObserver"
            ),
            real_samples_only=True,
            direction_specific_component_selection=True,
            residual_application=(
                "filtered residual validity is anchored to the measured-state timestamp, "
                "applies for at most 0.08 s from that timestamp, and cannot be renewed by "
                "reusing a stale packet; the base model resumes afterwards"
            ),
            adaptive_calibration_warmup=(
                "when the feature is enabled, the calibration adapter feeds observe_state "
                "during protected brake cycles and waits at least the configured minimum window"
            ),
            uncertainty=(
                "EW residual sigma with configured floor; 2-sigma velocity margin over "
                "the residual exposure only"
            ),
            mismatches=[
                "The residual observer remains disabled in runtime defaults.",
                "Each offline fold refits directional components from the other trial pairs; "
                "these are not persisted or approved runtime models.",
                "Logged receive-time samples approximate live update cadence but do not prove "
                "real-time scheduling behavior.",
                "This evaluates forecasts under an unchanged already-sent command only; it does "
                "not validate candidate pulse selection, position handoff, or flight safety.",
            ],
        ),
        metadata=metadata,
        folds=folds,
        summary=summarize(rows, config.horizons_s),
        ready_only_summary=summarize(ready_rows, config.horizons_s),
        segment_summary=segment_summary,
        forecasts=rows,
        skipped_windows=dict(
            unseen_future_command=skipped_unseen_command,
            insufficient_future_labels=skipped_end_of_log,
        ),
    )


def _markdown(report: dict) -> str:
    lines = [
        "# 有界滚动平动残差：离线留对验证",
        "",
        "**只读离线分析：没有修改校准文件，也没有发送飞行命令。**",
        "",
        "状态：`exploratory_only_not_runtime_eligible`。本报告不评定、授予或改变控制资格；"
        "运行时的残差观测器仍默认关闭。",
        "",
        "每折整体留出相邻的 ±Y 试验对。每个锚点只使用此前实测状态和此前已发送命令；"
        "跨越尚未发送命令的窗口不参与评分。未来状态只作为评分标签。重叠窗口不是独立飞行试验。",
        "",
        "观测器与运行时代码相同：最多保留 80ms，只接受跨度至少 60ms 的至少 4 个真实样本，"
        "样本间隙最多 40ms；超过 1.5m/s² 的残差会拒绝并清空，而不是截断。通过的估计使用 80ms "
        "EW 滤波及方差。残差有效期固定为该实测状态时间戳之后 80ms，重复使用旧状态不会续期；"
        "之后恢复基础模型。",
        "",
        "| 预测时长 | 方法 | N | 速度 MAE / RMSE / 最大 m/s | 位置 MAE / RMSE / 最大 cm | 漏判/误报反向 |",
        "|---:|---|---:|---:|---:|---:|",
    ]
    labels = {"baseline": "基础模型", "bounded_residual": "运行时语义残差"}
    for horizon, models in report["summary"].items():
        for model, row in models.items():
            if not row.get("count"):
                lines.append(f"| {horizon}s | {labels[model]} | 0 | — | — | — |")
                continue
            lines.append(
                f"| {horizon}s | {labels[model]} | {row['count']} | "
                f"{row['velocity_mae_m_s']:.4f} / {row['velocity_rmse_m_s']:.4f} / "
                f"{row['velocity_max_abs_error_m_s']:.4f} | "
                f"{100*row['position_mae_m']:.2f} / {100*row['position_rmse_m']:.2f} / "
                f"{100*row['position_max_abs_error_m']:.2f} | "
                f"{row['missed_reversal_count']} / {row['false_reversal_count']} |"
            )
    lines += [
        "",
        "方向选择使用各折训练数据分别拟合的 `positive_y` / `negative_y` 分量。完整的观测器状态、"
        "拒绝次数、EW sigma、2-sigma 速度区间覆盖及保守反向判断在 JSON/CSV 中。"
        "`ready_only_summary` 另列真正应用残差的锚点，避免 warming-up 样本稀释比较。",
        "",
        "## 与实际运行仍有的差异",
        "",
    ]
    lines.extend(f"- {item}" for item in report["runtime_semantics"]["mismatches"])
    lines += [
        "",
        "`report.json` 保留折分、完整指标和逐锚点预测。`forecasts.csv` 便于进一步画图。",
        "该结果只评价短时预测；不构成实机启用、位置接管或安全批准。",
        "",
    ]
    return "\n".join(lines)


def _write_csv(path: Path, rows: list[dict]) -> None:
    if not rows:
        return
    with path.open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main(argv=None) -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("log", type=Path)
    parser.add_argument("--output", type=Path, required=True,
                        help="new analysis directory; existing paths are refused")
    parser.add_argument("--history-window-s", type=float, default=0.08)
    parser.add_argument("--minimum-history-s", type=float, default=0.06)
    parser.add_argument("--maximum-sample-gap-s", type=float, default=0.04)
    parser.add_argument("--filter-time-constant-s", type=float, default=0.08)
    parser.add_argument("--maximum-residual-m-s2", type=float, default=1.5)
    parser.add_argument("--sigma-floor-m-s2", type=float, default=0.15)
    parser.add_argument("--sigma-multiplier", type=float, default=2.0)
    parser.add_argument("--residual-apply-horizon-s", type=float, default=0.08)
    parser.add_argument("--anchor-spacing-s", type=float, default=0.05)
    args = parser.parse_args(argv)
    if args.output.exists():
        parser.error("choose a new output directory")
    config = ResidualConfig(
        history_window_s=args.history_window_s,
        minimum_history_s=args.minimum_history_s,
        maximum_sample_gap_s=args.maximum_sample_gap_s,
        filter_time_constant_s=args.filter_time_constant_s,
        maximum_residual_m_s2=args.maximum_residual_m_s2,
        sigma_floor_m_s2=args.sigma_floor_m_s2,
        sigma_multiplier=args.sigma_multiplier,
        residual_apply_horizon_s=args.residual_apply_horizon_s,
        anchor_spacing_s=args.anchor_spacing_s,
    )
    try:
        config.validate()
        records = json.loads(args.log.read_text())
        report = analyze(records, config)
    except (OSError, json.JSONDecodeError, ValueError) as exc:
        parser.error(str(exc))
    report["metadata"].update(
        source_log=str(args.log.resolve()),
        source_sha256=hashlib.sha256(args.log.read_bytes()).hexdigest(),
    )
    args.output.mkdir(parents=True, exist_ok=False)
    (args.output / "report.json").write_text(
        json.dumps(report, indent=2, allow_nan=False) + "\n"
    )
    (args.output / "README.md").write_text(_markdown(report))
    _write_csv(args.output / "forecasts.csv", report["forecasts"])
    print(json.dumps(report["summary"], indent=2))


if __name__ == "__main__":
    main()
