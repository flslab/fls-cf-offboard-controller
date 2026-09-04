"""Paired evaluation must preserve common conditions and the complete tail."""
from copy import deepcopy
from dataclasses import asdict, replace
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
import importlib.util

import numpy as np

HAS_GYM = importlib.util.find_spec('gymnasium') is not None

if HAS_GYM:
    from Interaction.evaluate_rl_braking import (EvaluationCase, make_synthetic_cases,
        make_observed_cases, run_episode, summarize, main)
    from Interaction.rl_braking_env import BrakingScenario, RLBrakeConfig
from Interaction.offline_braking_selector import FrozenTiltModel, BrakingSnapshot
from Interaction.validate_predictive_brake_release import simulate, StressCase


MODEL = FrozenTiltModel(.025,14.56,.987,1.035,1.095,-.00148)


def case():
    return EvaluationCase('test','test',BrakingScenario('test', MODEL,
        BrakingSnapshot(0.,0.,.718,np.radians(5.75),np.radians(-56.6)),
        ((-.6,0.),(-.4,np.radians(20)),(-.19,0.)), first_decision_s=.009),MODEL)


class FixedPolicy:
    def __init__(self, duration=.24):
        self.duration = duration
        self.observations = []

    def predict(self, obs, deterministic):
        assert deterministic
        self.observations.append(obs.copy())
        return int(float(obs[-1])*.5 >= self.duration-1e-7), None


@unittest.skipUnless(HAS_GYM, 'optional offline RL dependencies are not installed')
class RLEvaluationTest(unittest.TestCase):
    def test_same_seed_population_exact_and_distinct_from_development(self):
        a=make_synthetic_cases(MODEL,3,491723)
        b=make_synthetic_cases(MODEL,3,491723)
        c=make_synthetic_cases(MODEL,3,9001,'development')
        self.assertEqual(a,b)
        self.assertNotEqual(a[0].scenario.snapshot,c[0].scenario.snapshot)
        with self.assertRaises(ValueError):
            make_synthetic_cases(MODEL,0)

    def test_fixed_baseline_matches_existing_simulator(self):
        selected=case()
        seed=dict(name='test',kind='observed_initial_state',snapshot=selected.scenario.snapshot,
            command_history=selected.scenario.command_history,model=MODEL,
            first_brake_after_snapshot_s=.009,baseline_brake_duration_s=.24)
        prior,_,_=simulate(seed,StressCase('nominal'),'fixed_240ms')
        row,_=run_episode(selected,'fixed_240ms')
        for key in ['release_after_brake_s','terminal_mean_velocity_m_s','min_velocity_m_s',
                    'max_rollback_m','final_displacement_m']:
            self.assertAlmostEqual(row[key],prior[key],places=10)

    def test_predictive_baseline_matches_existing_simulator(self):
        selected=case()
        seed=dict(name='test',kind='observed_initial_state',snapshot=selected.scenario.snapshot,
            command_history=selected.scenario.command_history,model=MODEL,
            first_brake_after_snapshot_s=.009,baseline_brake_duration_s=.24)
        prior,_,_=simulate(seed,StressCase('nominal'),'predictive_level')
        row,_=run_episode(selected,'predictive_margin_080')
        for key in ['release_after_brake_s','terminal_mean_velocity_m_s','min_velocity_m_s','max_rollback_m']:
            self.assertAlmostEqual(row[key],prior[key],places=10)

    def test_policy_receives_only_observation_and_identical_actions_are_paired(self):
        fixed,_=run_episode(case(),'fixed_240ms')
        policy=FixedPolicy()
        learned,trace=run_episode(case(),'dummy',policy=policy,trace=True)
        self.assertAlmostEqual(fixed['loss'],learned['loss'],places=10)
        self.assertAlmostEqual(learned['reward'],-learned['loss'])
        self.assertTrue(all(isinstance(obs,np.ndarray) and obs.dtype==np.float32 for obs in policy.observations))
        self.assertTrue(trace['decisions'])
        self.assertGreaterEqual(learned['observation_after_level_s'],1.5-1e-10)

    def test_case_not_mutated_by_any_method(self):
        selected=case()
        before=deepcopy(selected)
        for method in ['fixed_240ms','predictive_margin_080','predictive_margin_025']:
            run_episode(selected,method)
        self.assertEqual(selected,before)
        with self.assertRaises(ValueError):
            run_episode(selected,'missing_policy')

    def test_population_aggregation_does_not_pool_regressions_with_holdout(self):
        a,_=run_episode(case(),'fixed_240ms')
        b=dict(a,population='held_out_synthetic')
        groups=summarize([a,b])
        self.assertEqual(len(groups),2)
        self.assertTrue(all(g['count']==1 for g in groups))
        self.assertTrue(all(g['reversal_count']==1 for g in groups))

    def test_prepare_manifest_does_not_evaluate_or_train(self):
        report=dict(frozen_tilt_fit=dict(model='second_order',delay_s=.025,
            wn_rad_s=14.56,zeta=.987,gain=1.035,bias_world_y_rad=-.00148),frozen_motion_gain=1.095)
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            source=root/'frozen.json'
            source.write_text(json.dumps(report))
            output=root/'prepared'
            with patch('Interaction.evaluate_rl_braking.run_episode') as rollout:
                self.assertEqual(main(['--frozen-report',str(source),'--output',str(output),
                    '--count','3','--prepare-only']),0)
            rollout.assert_not_called()
            manifest=json.loads((output/'manifest.json').read_text())
            self.assertEqual(len(manifest['cases']),3)
            self.assertEqual(manifest['population_seed'],491723)
            self.assertFalse((output/'report.json').exists())

    def test_exclusion_requires_matching_log(self):
        with self.assertRaises(ValueError):
            make_observed_cases([],{},['missing.json:1'])

    def test_fixed_baseline_rejects_rounded_or_premature_switch(self):
        for config in [replace(RLBrakeConfig(),control_period_s=.1),
                       replace(RLBrakeConfig(),max_brake_s=.2)]:
            with self.subTest(config=config), self.assertRaises(ValueError):
                run_episode(case(),'fixed_240ms',config=config)


if __name__=='__main__':
    unittest.main()
