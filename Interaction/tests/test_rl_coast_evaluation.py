"""Paired causal comparison and immutable target provenance for offline v2."""
from copy import deepcopy
from dataclasses import asdict
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

import numpy as np
from Interaction.offline_braking_selector import BrakingSnapshot, FrozenTiltModel

HAS_GYM=importlib.util.find_spec('gymnasium') is not None
if HAS_GYM:
    from Interaction.rl_braking_env import BrakingScenario
    from Interaction.rl_coast_env import CoastScenario
    from Interaction.evaluate_rl_coast import (CoastEvaluationCase, make_cases, observed_regressions,
        nominal_distance_action, run_episode, summarize, main, V1_LABEL)

MODEL=FrozenTiltModel(.025,14.56,.987,1.035,1.095,-.00148)


def case():
    b=BrakingScenario('test',MODEL,BrakingSnapshot(0.,0.,.718,np.radians(5.75),np.radians(-56.6)),
                      ((-.6,0.),(-.4,np.radians(20)),(-.19,0.)))
    return CoastEvaluationCase('test','test',CoastScenario(b,.2),MODEL)


class FixedPolicy:
    def __init__(self,duration=0.,shape=117):
        self.duration=duration
        self.shape=shape
        self.calls=0

    def predict(self,obs,deterministic=True):
        assert deterministic and obs.shape==(self.shape,)
        self.calls+=1
        return int(float(obs[113])*.5 >= self.duration-1e-7),None


@unittest.skipUnless(HAS_GYM,'optional offline RL dependencies are not installed')
class CoastEvaluationTests(unittest.TestCase):
    def test_paired_identical_immediate_actions(self):
        a,_=run_episode(case(),'immediate_position')
        policy=FixedPolicy()
        b,trace=run_episode(case(),'dummy',policy=policy,trace=True)
        self.assertEqual(a['loss'],b['loss'])
        self.assertEqual(b['loss'],-b['reward'])
        self.assertEqual(policy.calls,1)
        self.assertEqual(trace['target_position_m'],.2)
        self.assertTrue(all(d['target_position_m']==.2 for d in trace['decisions']))

    def test_paired_fixed_240ms(self):
        a,_=run_episode(case(),'fixed240ms_position')
        b,_=run_episode(case(),'dummy',policy=FixedPolicy(.24))
        self.assertAlmostEqual(a['loss'],b['loss'],places=9)
        self.assertAlmostEqual(a['handoff_after_s'],.24,places=10)

    def test_v1_only_sees_exact_114_value_prefix(self):
        p=FixedPolicy(.1,shape=114)
        a,_=run_episode(case(),V1_LABEL,policy=p)
        b,_=run_episode(case(),'dummy',policy=FixedPolicy(.1))
        self.assertEqual(a['loss'],b['loss'])

    def test_all_methods_preserve_scenario(self):
        c=case()
        old=deepcopy(c)
        for method in ('immediate_position','fixed240ms_position','nominal_distance_gate','predictive080_position_adapter'):
            r,_=run_episode(c,method)
            self.assertTrue(np.isfinite(r['loss']))
            self.assertEqual(c,old)

    def test_nominal_gate_uses_only_measured_inputs(self):
        state=BrakingSnapshot(0.,0.,.5,0.,0.)
        info=dict(snapshot=state,decision_time_s=.01,target_position_m=.3)
        self.assertEqual(nominal_distance_action(info),1)
        self.assertEqual(nominal_distance_action(dict(info,target_position_m=.01)),0)
        self.assertEqual(nominal_distance_action(dict(info,hidden_model='poison',future_velocity=-4)),1)

    def test_distinct_fresh_deterministic_populations(self):
        a=make_cases(MODEL,2,19001,'development')
        self.assertEqual(a,make_cases(MODEL,2,19001,'development'))
        self.assertNotEqual(a[0].scenario,make_cases(MODEL,2,804271)[0].scenario)
        for seed in (9001,491723):
            with self.assertRaises(ValueError):
                make_cases(MODEL,2,seed)

    def test_prepare_only_never_runs_rollout(self):
        source=dict(frozen_tilt_fit=dict(model='second_order',delay_s=.025,wn_rad_s=14.56,
            zeta=.987,gain=1.035,bias_world_y_rad=-.00148),frozen_motion_gain=1.095)
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)
            (p/'frozen.json').write_text(json.dumps(source))
            with patch('Interaction.evaluate_rl_coast.run_episode') as rollout:
                self.assertEqual(main(['--frozen-report',str(p/'frozen.json'),'--output',str(p/'out'),
                                      '--count','2','--prepare-only']),0)
            rollout.assert_not_called()
            data=json.loads((p/'out/manifest.json').read_text())
            self.assertFalse(data['position_model_identified'])
            self.assertEqual(len(data['cases']),2)
            self.assertFalse((p/'out/report.json').exists())

    def test_observed_regressions_keep_only_named_initial_states_and_assign_targets(self):
        b=case().scenario.braking
        row=dict(name='original',population='observed_initial_nominal_regression',scenario=asdict(b),nominal_model=asdict(MODEL))
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'source.json'
            p.write_text(json.dumps(dict(cases=[row,dict(row,population='held_out_synthetic')])))
            cases,source=observed_regressions(p,MODEL)
        self.assertEqual(len(cases),3)
        self.assertEqual([c.scenario.target_distance_m for c in cases],[.08,.16,.28])
        self.assertTrue(all(c.scenario.braking.snapshot==b.snapshot for c in cases))
        self.assertTrue(all(c.population=='observed_initial_synthetic_target_regression' for c in cases))
        self.assertIn('not measured endpoints',source['note'])

    def test_joint_settling_and_nonreversal_are_separate(self):
        a,_=run_episode(case(),'immediate_position')
        altered=dict(a,joint_settled_in_simulation=True,no_reverse_in_simulation=False)
        result=summarize([altered])[0]
        self.assertEqual(result['joint_settled_count'],1)
        self.assertEqual(result['no_reverse_settled_count'],0)
        grouped=summarize([altered,dict(altered,population='regression')])
        self.assertEqual(len(grouped),2)


if __name__=='__main__':
    unittest.main()
