import ast
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
from pathlib import Path
from tempfile import TemporaryDirectory
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

import yaml

from Interaction import config
from Interaction.mission_profiles import resolve_mission_profiles
from Interaction.calibration_contact_logging import calibration_log_vars
from Interaction.contact_validation_capture import (configure_contact_validation_capture,
    PotentiometerSampleCapture, validate_profile)
from Interaction.potentiometer_force_sensor import PotentiometerForceSensor, parse_potentiometer_line
from Interaction.tests.test_calibration_contact_logging import fake_cf


def profile_fixture():
    return dict(schema_version=1,drone_id='lb11',mass_kg=.17,command_authority=False,
        deployment_ready=False,baseline=dict(specific_force_xy_m_s2=[.1,-.3]),
        rotational=dict(effective_offset_m=[.02,.03,-.02]),
        detector=dict(component_thresholds=[.08,.08,.12],covariance_floor=[.015,.015,.025]),
        onset=dict(onset_mode='continuous',onset_dwell_s=.03,onset_max_gap_s=.025))


class CaptureTests(unittest.TestCase):
    def test_packaged_mission_and_profile_are_complete_without_private_calibration(self):
        root=Path(__file__).resolve().parents[2]
        mission=yaml.safe_load((root/'Interaction/examples/contact_validation_lb11.yaml').read_text())
        mission=resolve_mission_profiles(mission)
        options=mission['Interaction']['config']['contact_validation_capture']
        profile_path=root/options['profile_path']
        contents=profile_path.read_bytes()
        self.assertEqual(hashlib.sha256(contents).hexdigest(), options['profile_sha256'])
        profile=json.loads(contents)
        self.assertFalse(profile['command_authority'])
        self.assertFalse(profile['deployment_ready'])
        self.assertEqual(profile['source_sha256'],
                         '41673093caf448483a7e2a760cabd94a97fdca8698e56e53cf0ba71b03f4daf6')
        options['profile_path']=str(profile_path)
        selected=config.log_vars_for_mission(mission)
        cf=fake_cf(calibration_log_vars(selected))
        args=SimpleNamespace(interaction=True,sense=True,log=True,drone_id='lb11',cf_log_period=10)
        logs=SimpleNamespace(live_logger=Mock(),capture_packet_timing=False)
        configure_contact_validation_capture(logs,cf,selected,mission,args)
        manifest=logs.live_logger.write.call_args.args[0]['data']
        self.assertEqual(manifest['primary_detector'],'potentiometer')
        self.assertFalse(manifest['profile_runtime_enabled'])
        self.assertEqual(len(manifest['groups']),7)
        self.assertEqual(manifest['requested_packets_per_s'],700)

    def setup(self,directory):
        path=Path(directory)/'profile.json'; path.write_text(json.dumps(profile_fixture()))
        options=dict(enabled=True,profile_path=str(path),
            profile_sha256=hashlib.sha256(path.read_bytes()).hexdigest())
        mission={'Interaction':{'config':{'contact_validation_capture':options,
            'wrench_interaction':{'mass':.17},'detection_method':'potentiometer'}}}
        args=SimpleNamespace(interaction=True,sense=True,log=True,drone_id='lb11',cf_log_period=10)
        logs=SimpleNamespace(live_logger=Mock(),capture_packet_timing=False)
        cf=fake_cf(calibration_log_vars(config.LOG_VARS))
        return path,options,mission,args,logs,cf

    def test_disabled_is_exact_noop_and_needs_no_profile(self):
        selected=config.LOG_VARS
        self.assertIs(configure_contact_validation_capture(None,None,selected,{},SimpleNamespace()),selected)

    def test_capture_embeds_frozen_profile_without_mutating_mission_or_calibration(self):
        with TemporaryDirectory() as directory:
            path,options,mission,args,logs,cf=self.setup(directory)
            before=deepcopy(mission); contents=path.read_bytes()
            result=configure_contact_validation_capture(logs,cf,config.LOG_VARS,mission,args)
            self.assertEqual(mission,before); self.assertEqual(path.read_bytes(),contents)
            self.assertEqual(set(result)-set(config.LOG_VARS),{'FORCE_IMU'})
            entry=logs.live_logger.write.call_args.args[0]
            self.assertEqual(entry['type'],'contact_validation_capture')
            m=entry['data']; self.assertFalse(m['profile_runtime_enabled']); self.assertFalse(m['refit_enabled'])
            self.assertEqual(m['frozen_profile'],profile_fixture())
            self.assertEqual(m['primary_detector'],'potentiometer')
            self.assertEqual(m['imu_requested_rate_hz'],100)
            self.assertTrue(logs.capture_packet_timing)
            self.assertIsInstance(logs.contact_validation_pot_callback,PotentiometerSampleCapture)

    def test_missing_sense_log_and_wrong_modes_rejected_before_recording(self):
        for change in ({'sense':False},{'log':False},{'interaction':False},
                       {'calibrate':True},{'droneless':True}):
            with TemporaryDirectory() as directory:
                _,_,mission,args,logs,cf=self.setup(directory)
                for key,value in change.items(): setattr(args,key,value)
                with self.assertRaises(ValueError):
                    configure_contact_validation_capture(logs,cf,config.LOG_VARS,mission,args)
                logs.live_logger.write.assert_not_called()

    def test_missing_or_changed_profile_and_wrong_drone_rejected(self):
        for condition in ('changed','missing','wrong_drone'):
            with TemporaryDirectory() as directory:
                path,_,mission,args,logs,cf=self.setup(directory)
                if condition=='changed': path.write_text('{}')
                elif condition=='missing': path.unlink()
                else: args.drone_id='lb12'
                with self.assertRaises((ValueError,FileNotFoundError)):
                    configure_contact_validation_capture(logs,cf,config.LOG_VARS,mission,args)
                self.assertFalse(logs.capture_packet_timing)
                logs.live_logger.write.assert_not_called()

    def test_bad_profile_coefficients_and_authority_rejected(self):
        for change in ({'command_authority':True},{'mass_kg':.18},
                       {'baseline':{'specific_force_xy_m_s2':[float('nan'),0]}},
                       {'rotational':{'effective_offset_m':[1,0,0]}},{'detector':None}):
            with self.assertRaises(ValueError):
                validate_profile({**profile_fixture(),**change},'lb11',.17)

    def test_missing_imu_toc_preserves_original_logging_and_rejects_capture(self):
        with TemporaryDirectory() as directory:
            _,_,mission,args,logs,cf=self.setup(directory)
            del cf.log.toc.toc['gyro']['z']
            with self.assertRaises(RuntimeError):
                configure_contact_validation_capture(logs,cf,config.LOG_VARS,mission,args)
            logs.live_logger.write.assert_not_called()
            self.assertFalse(logs.capture_packet_timing)

    def test_raw_pot_saves_every_sample_and_both_clocks(self):
        writer=Mock(); capture=PotentiometerSampleCapture(writer)
        sample=parse_potentiometer_line('1000,1001,1001,4,0',host_time=10.,host_monotonic_time=20.)
        capture(sample); capture(replace(sample,arduino_time_ms=1010,host_time=10.01,host_monotonic_time=20.01))
        records=[c.args[0] for c in writer.write.call_args_list]
        self.assertEqual([r['data']['sample_sequence'] for r in records],[0,1])
        self.assertEqual(records[0]['data']['host_monotonic_time'],20.)
        self.assertEqual(records[1]['data']['arduino_time_ms'],1010)
        self.assertFalse(records[0]['data']['command_authority'])

    def test_callback_failure_cannot_kill_sensor_or_change_latest_sample(self):
        callback=Mock(side_effect=RuntimeError('writer stopped'))
        reader=PotentiometerForceSensor(sample_callback=callback)
        sample=parse_potentiometer_line('1000,1001,1001,4,0')
        reader._latest=sample
        with self.assertLogs('Interaction.potentiometer_force_sensor',level='ERROR'):
            reader._capture_sample(sample)
        reader._capture_sample(sample)
        callback.assert_called_once_with(sample)
        self.assertIs(reader.latest(),sample)
        self.assertIsNone(reader._reader_error)

    def test_controller_call_only_in_logging_and_sensor_gets_optional_callback(self):
        root=Path(__file__).resolve().parents[2]
        tree=ast.parse((root/'controller.py').read_text())
        methods={n.name:n for n in ast.walk(tree) if isinstance(n,ast.FunctionDef)}
        calls=[n for n in ast.walk(methods['setup_logging']) if isinstance(n,ast.Call)
               and isinstance(n.func,ast.Name) and n.func.id=='configure_contact_validation_capture']
        self.assertEqual(len(calls),1)
        constructors=[n for n in ast.walk(methods['setup_force_sensor']) if isinstance(n,ast.Call)
                      and isinstance(n.func,ast.Name) and n.func.id=='PotentiometerForceSensor']
        self.assertEqual(len(constructors),1)
        self.assertIn('sample_callback',{k.arg for k in constructors[0].keywords})


if __name__=='__main__': unittest.main()
