"""Background, diagnostic-only online identification during calibration.

The control thread calls ``start`` before calibration sample collection (which
may follow takeoff), ``submit_trial`` only after
the completed trial's recovery position command was successfully sent, and
``poll`` to drain events without waiting. ``request_finish`` is nonblocking;
keep polling during normal recovery/landing. ``close`` is idempotent and has a
bounded wait: unfinished work is terminated, never awaited on the flight loop.

Each complete opposed pair is scored using the *previously frozen* candidate,
then incorporated into a refit. The all-data refit is never described as
independently validated. No model from this module can command the aircraft.
Only the spawned, lower-priority worker fits or writes artifacts. Existing
files are never overwritten on session initialization. Interrupted artifacts
remain partial evidence; the normal flight logger remains the raw-data source
for trials that were queued but not consumed before interruption.
"""

from __future__ import annotations

import copy
import json
import math
import multiprocessing
import os
from pathlib import Path
import queue
import tempfile
import time


def _atomic_json(path, value):
    """Replace only an artifact this worker has already exclusively created."""
    path = Path(path)
    descriptor, temporary = tempfile.mkstemp(prefix=path.name + ".", suffix=".tmp",
                                              dir=str(path.parent))
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(value, stream, indent=2, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _plain(value):
    """Convert copied numpy scalars/arrays in calibration samples to JSON."""
    if isinstance(value, dict):
        return {str(key): _plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_plain(item) for item in value]
    if hasattr(value, "tolist"):
        return _plain(value.tolist())
    return value


def _validation_gates(metrics, config):
    trials = metrics.get("per_trial", [])
    limits = {
        "velocity_rmse_m_s": float(config.get("max_velocity_rmse_m_s", .06)),
        "terminal_error_m_s": float(config.get("max_terminal_abs_error_m_s", .06)),
        "end_position_error_m": float(config.get("max_endpoint_abs_error_m", .06)),
    }
    gates = {"at_least_two_trials": len(trials) >= 2}
    for field, limit in limits.items():
        values = [row.get(field) for row in trials]
        gates[field] = bool(values) and all(
            isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value)
            and (field != "velocity_rmse_m_s" or value >= 0)
            and abs(value) <= limit for value in values
        )
    gates["no_reversal_misses_or_false_alarms"] = bool(trials) and all(
        isinstance(row.get("actual_reverse"), bool)
        and isinstance(row.get("predicted_reverse"), bool)
        and row["actual_reverse"] == row["predicted_reverse"] for row in trials
    )
    return gates, limits


class _CalibrationAccumulator:
    """Worker-owned state; injected numerical functions make causality testable."""

    def __init__(self, config, output_path, drone_id, expected_segment_ids,
                 fit_fn, evaluate_fn, metadata=None):
        self.config = config
        self.path = Path(output_path)
        self.raw_path = self.path.with_suffix(".samples.jsonl")
        self.fit_fn = fit_fn
        self.evaluate_fn = evaluate_fn
        self.progress_callback = None
        self.trials = []
        self.expected = list(expected_segment_ids)
        self.report = {
            "schema_version": 1, "kind": "online_prediction_calibration",
            "drone_id": drone_id,
            "context": _plain(copy.deepcopy(metadata or {})),
            "status": "collecting", "data_complete": False,
            "expected_segment_ids": self.expected,
            "received_segment_ids": [], "rejected_trials": [],
            "fit_errors": [], "validation_history": [],
            "candidate": None, "validated_candidate": None,
            "validation_passed": False, "runtime_enabled": False,
            "flight_approved": False, "output_path": str(self.path),
            "report_path": str(self.path),
            "raw_samples_path": str(self.raw_path), "events_dropped": 0,
            "cautions": [
                "Numeric gates are diagnostic only, not flight approval.",
                "The latest all-data refit has no independent validation.",
                "Sequential maneuver amplitude/duration may confound time and battery.",
                "Queued samples interrupted before worker receipt remain in flight logs.",
            ],
        }
        self.path.parent.mkdir(parents=True, exist_ok=True)
        # Exclusive creation protects every earlier calibration and report.
        with self.path.open("x", encoding="utf-8") as stream:
            json.dump(self.report, stream, allow_nan=False)
        self.raw_stream = self.raw_path.open("x", encoding="utf-8")
        self.save()

    def save(self):
        _atomic_json(self.path, self.report)

    def _progress(self, event):
        # Persist before entering the next expensive stage. An interrupted
        # final refit must not erase an already-scored frozen candidate.
        if self.progress_callback is None:
            self.save()
        else:
            self.progress_callback(*event)

    def _reject(self, reason, segment_id=None):
        self.report["rejected_trials"].append({"segment_id": segment_id,
                                                 "reason": reason})
        self.report["data_complete"] = False
        self.report["validation_passed"] = False
        event = ("trial_rejected", {"segment_id": segment_id, "reason": reason})
        self._progress(event)
        return [event]

    def receive(self, samples):
        """Persist received raw data before numerical work; never discard silently."""
        samples = _plain(samples)
        self.raw_stream.write(json.dumps({"samples": samples}, allow_nan=False) + "\n")
        self.raw_stream.flush()
        os.fsync(self.raw_stream.fileno())
        if not samples:
            return self._reject("empty_trial")
        try:
            segment = samples[0]["segment_id"]
            if any(sample["segment_id"] != segment for sample in samples):
                return self._reject("mixed_segments", segment)
            expected_index = len(self.trials)
            if expected_index >= len(self.expected) or segment != self.expected[expected_index]:
                return self._reject("unexpected_or_out_of_order_segment", segment)
            timestamps = [float(sample["timestamp"]) for sample in samples]
            if (not all(math.isfinite(value) for value in timestamps)
                    or any(right <= left for left, right in zip(timestamps, timestamps[1:]))):
                return self._reject("non_monotonic_timestamps", segment)
            if self.trials and timestamps[0] <= float(self.trials[-1][-1]["timestamp"]):
                return self._reject("trial_not_strictly_later", segment)
            phases = {sample["phase"] for sample in samples}
            if not {"accelerate", "brake", "level_after_brake"}.issubset(phases):
                return self._reject("incomplete_attitude_trial", segment)
            signs = {1 if float(sample["direction_xy"][1]) > 0 else -1
                     for sample in samples
                     if abs(float(sample["direction_xy"][1])) > .99
                     and abs(float(sample["direction_xy"][0])) < .01}
            if len(signs) != 1 or any(
                    abs(float(sample["direction_xy"][1])) <= .99
                    or abs(float(sample["direction_xy"][0])) >= .01 for sample in samples):
                return self._reject("trial_must_have_one_y_direction", segment)
        except (KeyError, TypeError, ValueError, IndexError) as error:
            return self._reject("invalid_trial: " + str(error))
        self.trials.append(samples)
        self.report["received_segment_ids"].append(segment)
        events = [("trial_received", {"segment_id": segment, "sample_count": len(samples)})]
        self._progress(events[-1])
        if len(self.trials) % 2:
            return events
        pair = self.trials[-2:]
        pair_ids = [trial[0]["segment_id"] for trial in pair]
        opposed = float(pair[0][0]["direction_xy"][1]) * float(pair[1][0]["direction_xy"][1]) < 0
        if not opposed:
            events.extend(self._reject("pair_requires_both_y_directions", segment))
            return events
        frozen = copy.deepcopy(self.report["candidate"])
        if frozen is not None:
            self.report["worker_phase"] = "validating"
            self._progress(("validation_started", {
                "candidate_version": frozen["version"], "validation_segment_ids": pair_ids,
            }))
            validation = {"training_segment_ids": frozen["training_segment_ids"],
                          "validation_segment_ids": pair_ids,
                          "candidate_version": frozen["version"],
                          "model": copy.deepcopy(frozen["model"]),
                          "independent_validation": True,
                          "runtime_enabled": False, "flight_approved": False}
            try:
                validation_started = time.monotonic()
                if set(frozen["training_segment_ids"]) & set(pair_ids):
                    raise ValueError("training/validation overlap")
                metrics = _plain(self.evaluate_fn(
                    copy.deepcopy(frozen["model"]),
                    [sample for trial in pair for sample in trial],
                    max_sample_gap_s=float(self.config.get("max_sample_gap_s", .06))))
                gates, limits = _validation_gates(metrics, self.config)
                gates["both_y_directions"] = opposed
                gates["reported_segment_ids_match"] = metrics.get("validation_segment_ids") == pair_ids
                gates["complete_per_trial_results"] = len(metrics.get("per_trial", [])) == len(pair_ids)
                gates["per_trial_ids_match"] = [row.get("segment_id") for row in metrics.get("per_trial", [])] == pair_ids
                gates["per_trial_directions_match"] = {row.get("direction_y") for row in metrics.get("per_trial", [])} == {-1, 1}
                identifiability = frozen["model"].get("identifiability", {})
                gates["parameters_identifiable"] = identifiability.get("identifiable") is True
                gates["no_active_parameter_bounds"] = identifiability.get("bound_active_parameters") == []
                validation.update(metrics=metrics, gates=gates, limits=limits,
                                  evaluation_elapsed_s=time.monotonic()-validation_started,
                                  validation_passed=all(gates.values()))
                self.report["validated_candidate"] = copy.deepcopy(validation)
                events.append(("candidate_validated", {
                    "candidate_version": frozen["version"],
                    "training_segment_ids": frozen["training_segment_ids"],
                    "validation_segment_ids": pair_ids,
                    "validation_passed": validation["validation_passed"],
                    "gates": gates, "aggregates": metrics.get("aggregates", {}),
                }))
            except Exception as error:
                validation.update(validation_passed=False,
                                  error=f"{type(error).__name__}: {error}")
                self.report["validated_candidate"] = copy.deepcopy(validation)
                events.append(("validation_failed", {
                    "candidate_version": frozen["version"],
                    "validation_segment_ids": pair_ids, "error": validation["error"],
                }))
            self.report["validation_history"].append(validation)
            self._progress(events[-1])
        try:
            self.report["worker_phase"] = "fitting"
            self._progress(("fit_started", {
                "training_segment_ids": list(self.report["received_segment_ids"]),
            }))
            fitting_started = time.monotonic()
            fitted = _plain(self.fit_fn(
                [sample for trial in self.trials for sample in trial],
                max_sample_gap_s=float(self.config.get("max_sample_gap_s", .06))))
            # Validate serialization before replacing a known candidate.
            if (not isinstance(fitted, dict)
                    or fitted.get("train_segment_ids") != self.report["received_segment_ids"]):
                raise ValueError("fit did not return the exact training segment provenance")
            metadata = self.report['context']
            if metadata.get('mode') == 'offline_sequential_replay':
                # Historical command clocks were reconstructed, not sampled
                # directly at each live command send. Copies retain this fact.
                fitted.setdefault('source_provenance', {}).update(
                    source='reconstructed_flight_log_samples',
                    command_clock=metadata.get('command_clock_method'),
                    completion_certified_by=metadata.get('completion_certification'),
                    source_log_path=metadata.get('source_path'),
                    source_log_sha256=metadata.get('source_sha256'),
                )
            json.dumps(fitted, allow_nan=False)
            candidate = {"version": len(self.trials) // 2,
                         "training_segment_ids": list(self.report["received_segment_ids"]),
                         "model": fitted, "independent_validation": False,
                         "fit_elapsed_s": time.monotonic()-fitting_started,
                         "validation_passed": False, "runtime_enabled": False,
                         "flight_approved": False}
            self.report["candidate"] = candidate
            events.append(("candidate_fitted", {
                "version": candidate["version"],
                "training_segment_ids": candidate["training_segment_ids"],
                "candidate_status": fitted.get("candidate_status"),
                "fit_elapsed_s": candidate['fit_elapsed_s'],
                "report_path": str(self.path),
                "independent_validation": False,
            }))
        except Exception as error:
            failure = {"training_segment_ids": list(self.report["received_segment_ids"]),
                       "error": f"{type(error).__name__}: {error}"}
            self.report["fit_errors"].append(failure)
            events.append(("candidate_rejected", failure))
        self.report["worker_phase"] = "collecting"
        self._progress(events[-1])
        return events

    def finish(self, submission_summary):
        self.report["submission_summary"] = submission_summary
        received = self.report["received_segment_ids"]
        self.report["missing_segment_ids"] = [segment for segment in self.expected if segment not in received]
        complete = (received == self.expected and not self.report["rejected_trials"]
                    and not submission_summary.get("rejected_submissions", 0)
                    and submission_summary.get("accepted_submissions") == len(received)
                    and len(received) >= 2 and len(received) % 2 == 0)
        self.report["data_complete"] = complete
        self.report["candidate_training_complete"] = bool(
            self.report["candidate"]
            and self.report["candidate"]["training_segment_ids"] == self.expected)
        self.report["status"] = "completed" if complete else "partial"
        self.report["worker_phase"] = "finished"
        validated = self.report["validated_candidate"]
        self.report["validation_passed"] = bool(
            complete and validated and validated["validation_passed"]
            and validated["validation_segment_ids"] == received[-2:]
            and validated["training_segment_ids"] == received[:-2])
        self.save()

    def close(self):
        self.raw_stream.close()


def _worker_main(inbox, outbox, config, output_path, drone_id, expected_segment_ids,
                 fit_fn=None, evaluate_fn=None, metadata=None):
    """Spawn entrypoint. No flight objects or network clients cross this boundary."""
    accumulator = None
    thread_limit = None
    try:
        for variable in ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS",
                         "VECLIB_MAXIMUM_THREADS", "NUMEXPR_NUM_THREADS"):
            os.environ[variable] = "1"
        try:
            os.nice(10)
        except (AttributeError, OSError):
            pass
        if fit_fn is None or evaluate_fn is None:
            from Interaction.online_dynamics_model import (
                fit_predictive_model, evaluate_predictive_model,
            )
            fit_fn = fit_fn or fit_predictive_model
            evaluate_fn = evaluate_fn or evaluate_predictive_model
        try:
            from threadpoolctl import threadpool_limits
            thread_limit = threadpool_limits(limits=1)
        except ImportError:
            pass
        accumulator = _CalibrationAccumulator(config, output_path, drone_id,
                                             expected_segment_ids, fit_fn, evaluate_fn,
                                             metadata=metadata)

        def publish(event, data):
            accumulator.save()
            message = {"event": event, "data": data,
                       "report": copy.deepcopy(accumulator.report)}
            try:
                outbox.put_nowait(message)
            except queue.Full:
                accumulator.report["events_dropped"] += 1
                accumulator.save()

        publish("worker_started", {})
        accumulator.progress_callback = publish
        while True:
            message = inbox.get()
            if message["kind"] == "finish":
                accumulator.finish(message["summary"])
                publish("calibration_finished", {})
                break
            if message["kind"] != "trial":
                raise ValueError("unknown worker message")
            accumulator.receive(message["samples"])
    except Exception as error:
        detail = f"{type(error).__name__}: {error}"
        if accumulator is not None:
            accumulator.report.update(status="failed", data_complete=False,
                                      validation_passed=False, worker_error=detail)
            try:
                accumulator.save()
            except Exception:
                pass
        try:
            outbox.put_nowait({"event": "worker_failed", "data": {"error": detail},
                              "report": accumulator.report if accumulator else None})
        except queue.Full:
            pass
    finally:
        if accumulator is not None:
            accumulator.close()
        if thread_limit is not None:
            thread_limit.restore_original_limits()


class OnlinePredictionCalibration:
    """Nonblocking control-thread facade for one bounded spawn worker.

    ``submit_trial`` snapshots caller-owned samples and returns queue acceptance,
    *not* fit success. Any rejection is counted and makes final data incomplete.
    ``poll`` returns ``{event, data}`` mappings (with a report snapshot when
    available); it never reads a report file or waits for numerical work.
    ``latest_report`` is a copied, cached snapshot, initially ``None``.
    The optional multiprocessing context must use ``spawn``; ``metadata`` holds
    inert mission provenance copied into the report's ``context`` field.
    Injected numerical functions are for tests.
    """

    def __init__(self, config, output_path, drone_id, expected_segment_ids,
                 context=None, *, metadata=None, _fit_fn=None, _evaluate_fn=None):
        self.config = copy.deepcopy(dict(config or {}))
        self.output_path = str(output_path)
        self.expected_segment_ids = list(expected_segment_ids)
        if (not self.expected_segment_ids
                or len(set(self.expected_segment_ids)) != len(self.expected_segment_ids)):
            raise ValueError("expected_segment_ids must be nonempty and unique")
        for name in ("max_sample_gap_s", "max_velocity_rmse_m_s",
                     "max_terminal_abs_error_m_s", "max_endpoint_abs_error_m"):
            value = float(self.config.get(name, .06))
            if not math.isfinite(value) or value <= 0:
                raise ValueError(name + " must be finite and positive")
        self._context = context or multiprocessing.get_context("spawn")
        if self._context.get_start_method() != "spawn":
            raise ValueError("online calibration requires spawn, never fork")
        self._queue_size = int(self.config.get("queue_size", 8))
        self._max_samples = int(self.config.get("max_samples_per_trial", 4096))
        if not 1 <= self._queue_size <= 32 or not 1 <= self._max_samples <= 32768:
            raise ValueError("online calibration queue/sample bounds are invalid")
        self._inbox = self._outbox = self._process = None
        self._drone_id = drone_id
        self._metadata = copy.deepcopy(dict(metadata or {}))
        self._fit_fn, self._evaluate_fn = _fit_fn, _evaluate_fn
        self._accepted = self._rejected = 0
        self._finishing = self._finished = self._closed = self._failed = False
        self._latest_report = None
        self._local_events = []

    @property
    def latest_report(self):
        return copy.deepcopy(self._latest_report)

    def start(self):
        if self._closed:
            return False
        if self._process is not None:
            return not self._failed
        try:
            self._inbox = self._context.Queue(maxsize=self._queue_size)
            self._outbox = self._context.Queue(maxsize=64)
            self._process = self._context.Process(
                target=_worker_main,
                args=(self._inbox, self._outbox, self.config, self.output_path,
                      self._drone_id, self.expected_segment_ids, self._fit_fn,
                      self._evaluate_fn, self._metadata),
                daemon=True, name="online-prediction-calibration")
            self._process.start()
        except Exception as error:
            self._failure(f"worker_start_failed: {type(error).__name__}: {error}")
            return False
        return True

    def _failure(self, reason):
        if self._failed:
            return
        self._failed = True
        report = self._latest_report or {
            "schema_version": 1, "kind": "online_prediction_calibration",
            "output_path": self.output_path, "report_path": self.output_path,
            "drone_id": self._drone_id, "context": copy.deepcopy(self._metadata),
            "expected_segment_ids": self.expected_segment_ids,
            "received_segment_ids": [], "validated_candidate": None,
            "candidate": None,
        }
        report.update(status="failed", data_complete=False, validation_passed=False,
                      runtime_enabled=False, flight_approved=False, worker_error=reason,
                      accepted_submissions=self._accepted,
                      rejected_submissions=self._rejected)
        self._latest_report = report
        self._local_events.append({"event": "worker_failed", "data": {"error": reason},
                                   "report": copy.deepcopy(report)})

    def submit_trial(self, samples):
        if (self._process is None or self._closed or self._finishing or self._failed
                or not self._process.is_alive()):
            self._rejected += 1
            return False
        try:
            if not 1 <= len(samples) <= self._max_samples:
                self._rejected += 1
                return False
            frozen = copy.deepcopy(list(samples))
            self._inbox.put_nowait({"kind": "trial", "samples": frozen})
        except Exception:
            self._rejected += 1
            return False
        self._accepted += 1
        return True

    def poll(self):
        if self._outbox is not None and not self._closed:
            for _ in range(64):
                try:
                    message = self._outbox.get_nowait()
                except (queue.Empty, OSError, ValueError):
                    break
                if message.get("report") is not None and not self._failed:
                    self._latest_report = message["report"]
                if message["event"] == "worker_failed":
                    self._failure(message["data"]["error"])
                else:
                    if message["event"] == "calibration_finished":
                        self._finished = True
                    self._local_events.append(message)
            if (self._process is not None and self._process.exitcode is not None
                    and not self._finished and not self._failed):
                self._failure("worker_exited_without_final_report")
        events, self._local_events = self._local_events, []
        return events

    def request_finish(self):
        if self._finishing or self._finished:
            return True
        if self._process is None or self._closed or self._failed:
            return False
        try:
            self._inbox.put_nowait({"kind": "finish", "summary": {
                "accepted_submissions": self._accepted,
                "rejected_submissions": self._rejected}})
        except (queue.Full, OSError, ValueError):
            return False
        self._finishing = True
        return True

    def close(self):
        """Wait at most ~170 ms, then kill unfinished work and retain partial logs."""
        if self._closed:
            return
        self.poll()
        self.request_finish()
        process = self._process
        if process is not None and process.pid is not None:
            process.join(timeout=.10)
            self.poll()
            if process.is_alive():
                process.terminate()
                process.join(timeout=.05)
            if process.is_alive():
                process.kill()
                process.join(timeout=.02)
            if not self._finished and not self._failed:
                self._failure("closed_before_background_fit_finished")
        self._closed = True
        for channel in (self._inbox, self._outbox):
            if channel is not None:
                channel.cancel_join_thread()
                channel.close()
