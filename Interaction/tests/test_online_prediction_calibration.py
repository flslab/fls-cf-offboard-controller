"""Causality, isolation, lifecycle, and conservative reporting contracts."""

import copy
import json
import multiprocessing
from pathlib import Path
import queue
import tempfile
import time
import unittest

from Interaction.online_prediction_calibration import (
    OnlinePredictionCalibration, _CalibrationAccumulator, _atomic_json,
)


def _samples(segment, direction=None):
    direction = (1 if segment % 2 == 0 else -1) if direction is None else direction
    return [{"segment_id": segment, "timestamp": 10 * segment + index * .01,
             "phase": phase, "direction_xy": [0., direction],
             "nested": {"owned": [segment]}}
            for index, phase in enumerate(("level_before_acceleration", "accelerate",
                                            "level_before_brake", "brake", "level_after_brake"))]


def _fake_fit(samples, *, max_sample_gap_s=.06):
    return {"train_segment_ids": list(dict.fromkeys(row["segment_id"] for row in samples)),
            "identifiability": {"identifiable": True, "bound_active_parameters": []}}


def _fake_evaluate(model, samples, *, max_sample_gap_s=.06):
    ids = list(dict.fromkeys(row["segment_id"] for row in samples))
    assert not set(ids) & set(model["train_segment_ids"])
    assert max(model["train_segment_ids"]) < min(ids)
    return {"schema_version": 1, "validation_segment_ids": ids,
            "per_trial": [{"segment_id": segment, "direction_y": 1 if segment % 2 == 0 else -1,
                           "velocity_rmse_m_s": .02, "terminal_error_m_s": -.01,
                           "end_position_error_m": .03, "actual_reverse": False,
                           "predicted_reverse": False} for segment in ids],
            "aggregates": {"velocity_rmse_m_s": .02}}


def _slow_fit(samples, *, max_sample_gap_s=.06):
    time.sleep(30)
    return _fake_fit(samples)


class _LocalQueue(queue.Queue):
    def cancel_join_thread(self):
        pass

    def close(self):
        pass


class _FakeProcess:
    def __init__(self, **kwargs):
        self.alive = False
        self.exitcode = None
        self.pid = None

    def start(self):
        self.alive = True

    def is_alive(self):
        return self.alive


class _FakeContext:
    def get_start_method(self):
        return "spawn"

    def Queue(self, maxsize):
        return _LocalQueue(maxsize=maxsize)

    def Process(self, **kwargs):
        return _FakeProcess(**kwargs)


class CalibrationAccumulatorTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.directory = Path(self.temporary.name)
        self.accumulators = []

    def tearDown(self):
        for accumulator in self.accumulators:
            accumulator.close()
        self.temporary.cleanup()

    def accumulator(self, fit=_fake_fit, evaluate=_fake_evaluate, expected=range(6), config=None):
        accumulator = _CalibrationAccumulator(
            config or {}, self.directory / (str(len(self.accumulators)) + ".json"),
            "lb11", list(expected), fit, evaluate)
        self.accumulators.append(accumulator)
        return accumulator

    def test_prequential_frozen_models_are_disjoint_and_final_refit_not_validated(self):
        calls = []

        def fit(samples, **kwargs):
            model = _fake_fit(samples, **kwargs)
            calls.append(("fit", model["train_segment_ids"][:]))
            return model

        def evaluate(model, samples, **kwargs):
            result = _fake_evaluate(model, samples, **kwargs)
            calls.append(("evaluate", model["train_segment_ids"][:], result["validation_segment_ids"]))
            model["train_segment_ids"].append(999)  # A consumer cannot mutate saved parameters.
            return result

        state = self.accumulator(fit=fit, evaluate=evaluate)
        for segment in range(6):
            state.receive(_samples(segment))
        state.finish({"accepted_submissions": 6, "rejected_submissions": 0})
        self.assertEqual(calls, [("fit", [0, 1]), ("evaluate", [0, 1], [2, 3]),
                                 ("fit", [0, 1, 2, 3]),
                                 ("evaluate", [0, 1, 2, 3], [4, 5]),
                                 ("fit", [0, 1, 2, 3, 4, 5])])
        report = state.report
        self.assertTrue(report["validation_passed"])
        self.assertTrue(report["candidate_training_complete"])
        self.assertFalse(report["candidate"]["independent_validation"])
        self.assertFalse(report["candidate"]["validation_passed"])
        self.assertEqual(report["validated_candidate"]["model"]["train_segment_ids"], [0, 1, 2, 3])
        self.assertEqual(report["validated_candidate"]["validation_segment_ids"], [4, 5])
        self.assertFalse(report["runtime_enabled"])
        self.assertFalse(report["flight_approved"])
        self.assertTrue(report["candidate"]["control_eligible"])
        self.assertEqual(
            report["validated_candidate"][
                "terminal_velocity_error_margins_m_s"
            ],
            {"positive_y": .01, "negative_y": .01},
        )
        rows = [json.loads(line) for line in state.raw_path.read_text().splitlines()]
        self.assertEqual(len(rows), 6)
        self.assertEqual(rows[5]["samples"], _samples(5))

    def test_early_bad_candidate_does_not_override_latest_frozen_validation(self):
        def evaluate(model, samples, **kwargs):
            result = _fake_evaluate(model, samples, **kwargs)
            if len(model["train_segment_ids"]) == 2:
                result["per_trial"][0]["velocity_rmse_m_s"] = 1.
            return result
        state = self.accumulator(evaluate=evaluate)
        for segment in range(6):
            state.receive(_samples(segment))
        state.finish({"accepted_submissions": 6})
        self.assertFalse(state.report["validation_history"][0]["validation_passed"])
        self.assertTrue(state.report["validation_passed"])

    def test_failed_prior_validation_makes_next_candidate_control_ineligible(self):
        def evaluate(model, samples, **kwargs):
            result = _fake_evaluate(model, samples, **kwargs)
            result["per_trial"][0]["terminal_error_m_s"] = -.2
            return result
        state = self.accumulator(evaluate=evaluate, expected=range(4))
        for segment in range(4):
            state.receive(_samples(segment))
        candidate = state.report["candidate"]
        self.assertFalse(candidate["control_eligible"])
        self.assertEqual(
            candidate["control_eligibility_reason"],
            "previous_frozen_candidate_failed_held_out_validation",
        )
        self.assertEqual(
            candidate["model"]["terminal_velocity_error_margin_m_s"], .2
        )

    def test_last_validation_exception_never_reuses_earlier_success(self):
        def evaluate(model, samples, **kwargs):
            if samples[0]["segment_id"] == 4:
                raise RuntimeError("bad final data")
            return _fake_evaluate(model, samples, **kwargs)
        state = self.accumulator(evaluate=evaluate)
        for segment in range(6):
            state.receive(_samples(segment))
        state.finish({"accepted_submissions": 6})
        self.assertFalse(state.report["validation_passed"])
        self.assertEqual(state.report["validated_candidate"]["validation_segment_ids"], [4, 5])
        self.assertIn("bad final data", state.report["validated_candidate"]["error"])
        self.assertFalse(state.report["candidate"]["control_eligible"])
        self.assertEqual(
            state.report["candidate"]["control_eligibility_reason"],
            "previous_frozen_candidate_failed_held_out_validation",
        )

    def test_fitting_exception_preserves_previous_candidate_and_records_failure(self):
        def fit(samples, **kwargs):
            if len(set(row["segment_id"] for row in samples)) > 2:
                raise RuntimeError("fit failure")
            return _fake_fit(samples, **kwargs)
        state = self.accumulator(fit=fit)
        for segment in range(6):
            events = state.receive(_samples(segment))
        state.finish({"accepted_submissions": 6})
        self.assertEqual(state.report["candidate"]["training_segment_ids"], [0, 1])
        self.assertEqual(len(state.report["fit_errors"]), 2)
        self.assertIn("candidate_rejected", [event for event, data in events])
        self.assertFalse(state.report["validation_passed"])
        self.assertFalse(state.report["candidate_training_complete"])

    def test_incomplete_out_of_order_and_nonopposed_trials_cannot_pass(self):
        for case in ("missing", "phase", "order", "direction", "dropped"):
            with self.subTest(case=case):
                state = self.accumulator(expected=range(4))
                for segment in range(3 if case == "missing" else 4):
                    samples = _samples(segment, 1 if case == "direction" else None)
                    if case == "phase" and segment == 1:
                        samples = samples[:-1]
                    if case == "order" and segment == 1:
                        samples = _samples(2)
                    state.receive(samples)
                state.finish({"accepted_submissions": 4, "rejected_submissions": int(case == "dropped")})
                self.assertFalse(state.report["validation_passed"])
                self.assertEqual(state.report["status"], "partial")

    def test_nonmonotonic_clock_and_y_direction_changes_reject(self):
        for case in ("time", "direction"):
            state = self.accumulator()
            samples = _samples(0)
            if case == "time":
                samples[-1]["timestamp"] = samples[0]["timestamp"]
            else:
                samples[-1]["direction_xy"] = [0, -1]
            events = state.receive(samples)
            self.assertEqual(events[0][0], "trial_rejected")
            self.assertEqual(state.report["received_segment_ids"], [])

    def test_numeric_reversal_and_identifiability_gates_are_conservative(self):
        cases = ("velocity_rmse_m_s", "terminal_error_m_s", "end_position_error_m",
                 "reverse", "missing", "unidentifiable", "bound")
        for case in cases:
            with self.subTest(case=case):
                def fit(samples, **kwargs):
                    model = _fake_fit(samples, **kwargs)
                    if case == "unidentifiable":
                        model["identifiability"]["identifiable"] = False
                    if case == "bound":
                        model["identifiability"]["bound_active_parameters"] = ["gain"]
                    return model

                def evaluate(model, samples, **kwargs):
                    result = _fake_evaluate(model, samples, **kwargs)
                    if case == "reverse":
                        result["per_trial"][0]["predicted_reverse"] = True
                    elif case == "missing":
                        result["per_trial"].pop()
                    elif case in cases[:3]:
                        result["per_trial"][0][case] = -.061
                    return result
                state = self.accumulator(fit=fit, evaluate=evaluate, expected=range(4))
                for segment in range(4):
                    state.receive(_samples(segment))
                state.finish({"accepted_submissions": 4})
                self.assertFalse(state.report["validation_passed"])

    def test_artifacts_are_atomic_and_existing_calibration_never_modified(self):
        previous = self.directory / "wrench_calibration.json"
        previous.write_text('{"original": true}')
        with self.assertRaises(FileExistsError):
            _CalibrationAccumulator({}, previous, "lb11", [0, 1], _fake_fit, _fake_evaluate)
        self.assertEqual(previous.read_text(), '{"original": true}')
        state = self.accumulator()
        for segment in range(2):
            state.receive(_samples(segment))
            state.save()
            self.assertEqual(json.loads(state.path.read_text())["received_segment_ids"], list(range(segment + 1)))
        before = state.path.read_text()
        with self.assertRaises(ValueError):
            _atomic_json(state.path, {"invalid": float("nan")})
        self.assertEqual(state.path.read_text(), before)
        self.assertEqual(list(self.directory.glob("*.tmp")), [])


class CalibrationSessionTests(unittest.TestCase):
    def test_submit_freezes_input_and_queue_full_is_explicit(self):
        session = OnlinePredictionCalibration({"queue_size": 1}, "unused.json", "lb11",
                                              [0, 1], context=_FakeContext())
        self.assertTrue(session.start())
        samples = _samples(0)
        original = copy.deepcopy(samples)
        self.assertTrue(session.submit_trial(samples))
        samples[0]["nested"]["owned"][0] = 99
        self.assertFalse(session.submit_trial(_samples(1)))
        self.assertFalse(session.request_finish())
        queued = session._inbox.get_nowait()
        self.assertEqual(queued["samples"], original)
        self.assertTrue(session.request_finish())
        finish = session._inbox.get_nowait()
        self.assertEqual(finish["summary"], {"accepted_submissions": 1, "rejected_submissions": 1})
        session.close()
        session.close()

    def test_worker_exit_is_an_event_not_a_flight_exception(self):
        session = OnlinePredictionCalibration({}, "unused.json", "lb11", [0, 1],
                                              context=_FakeContext())
        session.start()
        session._process.alive = False
        session._process.exitcode = 1
        events = session.poll()
        self.assertEqual(events[0]["event"], "worker_failed")
        self.assertFalse(session.submit_trial(_samples(0)))
        self.assertEqual(session.latest_report["status"], "failed")
        self.assertFalse(session.latest_report["flight_approved"])
        session.close()

    def test_rejects_fork_and_invalid_resource_limits(self):
        if "fork" in multiprocessing.get_all_start_methods():
            with self.assertRaises(ValueError):
                OnlinePredictionCalibration({}, "unused.json", "lb11", [0, 1],
                                            context=multiprocessing.get_context("fork"))
        for config in ({"queue_size": 0}, {"queue_size": 33},
                       {"max_samples_per_trial": 999999},
                       {"max_sample_gap_s": float("nan")},
                       {"terminal_velocity_error_margin_scale": .9},
                       {"terminal_velocity_error_margin_scale": 3.1}):
            with self.assertRaises(ValueError):
                OnlinePredictionCalibration(config, "unused.json", "lb11", [0, 1])

    def test_real_spawn_finishes_and_does_not_mutate_prior_artifact(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            previous = Path(directory) / "wrench_calibration.json"
            previous.write_text('{"retained": 42}')
            session = OnlinePredictionCalibration({}, path, "lb11", range(4),
                                                  _fit_fn=_fake_fit, _evaluate_fn=_fake_evaluate)
            try:
                self.assertTrue(session.start())
                for segment in range(4):
                    self.assertTrue(session.submit_trial(_samples(segment)))
                self.assertTrue(session.request_finish())
                events = self.wait_for(session, "calibration_finished")
                self.assertIn("calibration_finished", [event["event"] for event in events])
                report = session.latest_report
                self.assertTrue(report["validation_passed"])
                report["flight_approved"] = True
                self.assertFalse(session.latest_report["flight_approved"])
                self.assertTrue(json.loads(path.read_text())["data_complete"])
                self.assertEqual(previous.read_text(), '{"retained": 42}')
            finally:
                session.close()
            self.assertFalse(session._process.is_alive())

    def test_real_spawn_existing_output_becomes_failure_event(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            path.write_text('{"original": true}')
            session = OnlinePredictionCalibration({}, path, "lb11", range(2),
                                                  _fit_fn=_fake_fit, _evaluate_fn=_fake_evaluate)
            try:
                session.start()
                events = self.wait_for(session, "worker_failed")
                self.assertIn("worker_failed", [event["event"] for event in events])
                self.assertEqual(path.read_text(), '{"original": true}')
                self.assertFalse(session.latest_report["validation_passed"])
            finally:
                session.close()

    def test_real_spawn_production_numerics_match_service_report_contract(self):
        from Interaction.tests.test_online_dynamics_model import synthetic_samples
        samples = synthetic_samples()
        with tempfile.TemporaryDirectory() as directory:
            session = OnlinePredictionCalibration({}, Path(directory) / "report.json",
                                                  "lb11", range(4))
            try:
                self.assertTrue(session.start())
                for segment in range(4):
                    self.assertTrue(session.submit_trial([
                        sample for sample in samples if sample["segment_id"] == segment]))
                self.assertTrue(session.request_finish())
                self.wait_for(session, "calibration_finished")
                report = session.latest_report
                self.assertEqual(report["status"], "completed")
                self.assertTrue(report["validation_passed"], report["validated_candidate"])
                self.assertEqual(report["validated_candidate"]["model"]["train_segment_ids"], [0, 1])
                self.assertEqual(report["candidate"]["model"]["train_segment_ids"], [0, 1, 2, 3])
                self.assertFalse(report["candidate"]["independent_validation"])
            finally:
                session.close()

    def test_close_is_bounded_even_during_busy_worker_and_raw_data_survives(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "report.json"
            session = OnlinePredictionCalibration({}, path, "lb11", range(2),
                                                  _fit_fn=_slow_fit, _evaluate_fn=_fake_evaluate)
            session.start()
            session.submit_trial(_samples(0))
            session.submit_trial(_samples(1))
            self.wait_for(session, "worker_started")
            deadline = time.monotonic() + 5
            while time.monotonic() < deadline:
                raw = path.with_suffix(".samples.jsonl")
                if raw.exists() and len(raw.read_text().splitlines()) == 2:
                    break
                time.sleep(.01)
            started = time.monotonic()
            session.close()
            self.assertLess(time.monotonic() - started, .5)
            self.assertFalse(session._process.is_alive())
            self.assertFalse(session.latest_report["validation_passed"])
            self.assertEqual(len(raw.read_text().splitlines()), 2)
            self.assertNotEqual(json.loads(path.read_text())["status"], "completed")
            session.close()

    @staticmethod
    def wait_for(session, expected):
        events = []
        deadline = time.monotonic() + 8
        while time.monotonic() < deadline:
            events.extend(session.poll())
            if any(event["event"] == expected for event in events):
                return events
            time.sleep(.01)
        raise AssertionError("Timed out waiting for " + expected + ": " + repr(events))


if __name__ == "__main__":
    unittest.main()
