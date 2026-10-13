"""Actual firmware C transaction, live worker and matched estimator replay."""
from copy import deepcopy
import ctypes as C
import hashlib
import json
from pathlib import Path
import subprocess
from tempfile import TemporaryDirectory
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import numpy as np

from Interaction.estimator_hover_initialization import fit_hover_roll_pitch
from Interaction.estimator_hover_live import (
    LiveHoverXY, requested, load_static_candidate, read_saved_fit, select_saved_fit,
    save_fit, add_capture_logs, IMU_GROUP, KF_GROUP)
from Interaction.estimator_xy_loading import (
    FIELDS, REQUIRED, firmware_xy_coefficients, onboard_xy_candidate, prepare_xy_loading, load_xy)
from Interaction.tests.test_estimator_hover_initialization import hover_capture


def diagonal_capture():
    rows, candidate, extra = hover_capture()
    candidate['measured_from_reference'] = np.diag([1.02,.99,1.01]).tolist()
    A=np.asarray(candidate['measured_from_reference']); b=np.asarray(candidate['accel_bias_m_s2'])
    acc=(A@([0.,0.,9.81]+extra)+b)/9.81
    for row in rows:
        if row['group']=='imu':
            row['data'].update(dict(zip(['acc.'+k for k in 'xyz'],acc)))
    return rows, candidate


class NativeXY:
    def __init__(self, library):
        self.lib=C.CDLL(str(library))
        self.lib.reset()
        self.lib.stage.argtypes=[C.c_float]*4
        self.lib.process.argtypes=[C.c_uint,C.c_bool,C.c_bool,C.c_bool]
        self.lib.correct.argtypes=[C.POINTER(C.c_float)]
        self.lib.value.restype=C.c_uint

    def stage(self, c): self.lib.stage(*[c[k] for k in FIELDS])
    def process(self, generation, armed=True, default=True, brake=False):
        return bool(self.lib.process(generation,armed,default,brake))
    def correct(self, vector):
        v=(C.c_float*3)(*vector);self.lib.correct(v);return list(v)
    def state(self): return dict(zip(('ack','on','status'),[self.lib.value(i) for i in range(3)]))


class Params:
    def __init__(self, native):
        self.native=native;self.callbacks={};self.armed=False;self.default=True;self.commit=0
        self.values={'eskfXY.api':1,'eskfXY.req':0,'stabilizer.estimator':2,
                     **{'eskfXY.'+k:v for k,v in zip(FIELDS,[1.,1.,0.,0.])}}
        self.toc=SimpleNamespace(toc={'eskfXY':dict.fromkeys(REQUIRED)})
        self.writes=[]
    def add_update_callback(self,group,name,cb):self.callbacks[group+'.'+name]=cb
    def remove_update_callback(self,group,name,cb):self.callbacks.pop(group+'.'+name)
    def set_value(self,name,value):
        self.writes.append((name,value));self.values[name]=float(value)
        if name=='eskfXY.req':self.commit=int(value)
    def request_param_update(self,name):
        self.native.stage({k:self.values['eskfXY.'+k] for k in FIELDS})
        self.native.process(self.commit,self.armed,self.default)
        state=self.native.state()
        value=state[name.split('.')[1]] if name in ('eskfXY.ack','eskfXY.on','eskfXY.status') else self.values[name]
        self.callbacks[name](name,str(int(value)) if float(value).is_integer() else str(value))


class XYLoadingTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp=TemporaryDirectory();root=Path(cls.temp.name)
        header=Path(__file__).resolve().parents[1]/'native_estimator3'
        (root/'bridge.c').write_text('''#include "estimator_xy_calibration.h"
static estimatorXYCalibration_t c;
void reset(void){c=(estimatorXYCalibration_t){.staged={1,1,0,0}};}
void stage(float sx,float sy,float bx,float by){c.staged=(estimatorXYCoefficients_t){sx,sy,bx,by};}
int process(uint32_t req,bool armed,bool ordinary,bool brake){c.request=req;return estimatorXYProcess(&c,armed,ordinary,brake);}
void correct(float *v){estimatorXYCorrect(&c,v);}
uint32_t value(int i){return i==0?c.applied:i==1?c.enabled:c.status;}
''')
        cls.library=root/'xy.dylib'
        subprocess.run(['cc','-std=c11','-Wall','-Wextra','-Werror','-shared','-fPIC',
            '-I'+str(header),str(root/'bridge.c'),'-o',str(cls.library)],check=True,capture_output=True)

    @classmethod
    def tearDownClass(cls):cls.temp.cleanup()
    def setUp(self):self.native=NativeXY(self.library)

    def test_atomic_commit_frozen_coefficients_and_disarm_reset(self):
        c=dict(sx=1.01,sy=.99,bx=.1,by=-.2)
        self.native.stage(c)
        self.assertEqual(self.native.correct([1.,2.,3.]),[1.,2.,3.])
        self.assertTrue(self.native.process(7))
        np.testing.assert_allclose(self.native.correct([1.,2.,3.]),[.91,2.18,3.],atol=1e-6)
        self.native.stage(dict(sx=1.,sy=1.,bx=.5,by=.5))
        self.assertFalse(self.native.process(8))
        self.assertEqual(self.native.state(),dict(ack=7,on=1,status=4))
        np.testing.assert_allclose(self.native.correct([1.,2.,3.]),[.91,2.18,3.],atol=1e-6)
        self.native.process(8,armed=False)
        self.assertEqual(self.native.state(),dict(ack=0,on=0,status=0))

    def test_invalid_wrong_estimator_and_braking_commit_are_rejected(self):
        for c,default,brake in [([float('nan'),1,0,0],True,False),([1,1,2,0],True,False),
                               ([1,1,0,0],False,False),([1,1,0,0],True,True)]:
            self.native.lib.reset();self.native.lib.stage(*c)
            self.assertFalse(self.native.process(10,default=default,brake=brake))
            self.assertEqual(self.native.state()['on'],0)
            self.assertEqual(self.native.correct([1,2,3]),[1,2,3])

    def test_fit_maps_to_firmware_equation_preserving_raw_z_and_gyro(self):
        rows,candidate=diagonal_capture();fit=fit_hover_roll_pitch(rows,candidate,'hash')
        c=firmware_xy_coefficients(candidate,fit,'hash');self.native.stage(c);self.native.process(2)
        raw=np.array([.25,-.3,9.9]);corrected=self.native.correct(raw)
        expected=np.linalg.solve(candidate['measured_from_reference'],raw-candidate['accel_bias_m_s2'])
        expected-=np.asarray(fit['additional_body_accel_bias_m_s2']);expected[2]=raw[2]
        np.testing.assert_allclose(corrected,expected,atol=1e-6)
        replay=onboard_xy_candidate(c)
        np.testing.assert_allclose(np.linalg.solve(replay['measured_from_reference'],raw-replay['accel_bias_m_s2']),expected)
        self.assertEqual(replay['gyro_residual_bias_rad_s'],[0,0,0])
        bad=deepcopy(candidate);bad['measured_from_reference'][0][1]=.01
        with self.assertRaisesRegex(ValueError,'diagonal'):firmware_xy_coefficients(bad,fit,'hash')

    def test_fresh_stage_and_generation_ack_without_switching_estimator(self):
        params=Params(self.native);cf=SimpleNamespace(param=params)
        prepare_xy_loading(cf);params.armed=True
        result=load_xy(cf,dict(sx=1.01,sy=.99,bx=.1,by=-.2))
        self.assertEqual(self.native.state()['ack'],result['generation'])
        self.assertTrue(result['frozen']);self.assertFalse(result['z_changed'])
        self.assertFalse(any(name=='stabilizer.estimator' for name,_ in params.writes))
        self.assertEqual(params.callbacks,{})
        # Another armed-flight update is refused before staging.
        with self.assertRaises(RuntimeError):
            with patch('Interaction.estimator_xy_loading.confirm_firmware_mode_parameters',side_effect=RuntimeError('already on')):
                load_xy(cf,dict(sx=1,sy=1,bx=0,by=0))

    def test_commit_cancellation_cannot_apply_after_cleanup(self):
        params=Params(self.native);params.armed=True;cf=SimpleNamespace(param=params)
        def cancelled(action):raise RuntimeError('cancelled')
        with self.assertRaisesRegex(RuntimeError,'cancelled'):
            load_xy(cf,dict(sx=1,sy=1,bx=0,by=0),commit_guard=cancelled)
        self.assertEqual(self.native.state()['on'],0)
        self.assertFalse(any(k=='eskfXY.req' for k,_ in params.writes))

    def test_live_initial_window_fit_load_and_no_learning_during_interaction(self):
        rows,candidate=diagonal_capture();params=Params(self.native)
        with TemporaryDirectory() as tmp:
            root=Path(tmp);(root/'fit').mkdir();(root/'dataset.json').write_text('{}')
            candidate.update(drone_id='lb11',dataset_sha256=hashlib.sha256(b'{}').hexdigest())
            path=root/'fit/candidate.json';path.write_text(json.dumps(candidate))
            owner=SimpleNamespace(cf=SimpleNamespace(param=params),args=SimpleNamespace(drone_id='lb11',
                log_dir=tmp,tag='flight'),mission={'Interaction':{'config':{'level_coast':{
                    'estimator_imu_calibration_file':str(path)}}}},log_manager=SimpleNamespace(
                        add_cf_packet_listener=Mock(return_value=Mock()),
                        add_mocap_frame_listener=Mock(return_value=Mock())))
            live=LiveHoverXY(owner);params.armed=True
            live.update([0,0,.8],can_begin=True,now=100.)
            with live.lock:live.rows.extend(r for r in rows if r['group']!='event')
            self.assertFalse(live.update([0,0,.8],can_begin=True,now=105.01))
            live.worker.join(3)
            self.assertIsNone(live.failure);self.assertIsNotNone(live.result)
            self.assertFalse(live.update([0,0,.8],can_begin=True,now=live.result['applied_monotonic_s']+.1))
            self.assertTrue(live.update([0,0,.8],can_begin=True,now=live.result['applied_monotonic_s']+.31))
            frozen=self.native.correct([1,2,3]);size=len(live.rows)
            live.record('imu',{'acc.x':100},106.,60000)
            self.assertEqual(len(live.rows),size)
            self.assertEqual(self.native.correct([1,2,3]),frozen)
            self.assertTrue((live.output/'loaded.json').exists());live.close()
            with self.assertRaisesRegex(RuntimeError,'cancelled'):live.commit(lambda:None)

            # A reboot/disarm clears firmware RAM, but the saved file survives.
            # The second flight loads it with a new generation and no window.
            saved=read_saved_fit(live.cache_path,candidate,live.fingerprint,'lb11')
            self.assertFalse(saved['gyro_saved'])
            first_generation=live.result['generation']
            self.native.process(0,armed=False)
            params=Params(self.native);owner.cf.param=params;owner.args.tag='second'
            owner.log_manager.add_cf_packet_listener.reset_mock()
            owner.log_manager.add_mocap_frame_listener.reset_mock()
            second=LiveHoverXY(owner);params.armed=True
            self.assertFalse(second.update([0,0,.8],can_begin=True,now=200.))
            second.worker.join(3)
            self.assertIsNone(second.failure);self.assertIsNone(second.window)
            self.assertEqual(len(second.rows),0)
            self.assertTrue(second.result['reused_from_cache'])
            self.assertNotEqual(second.result['generation'],first_generation)
            owner.log_manager.add_cf_packet_listener.assert_not_called()
            owner.log_manager.add_mocap_frame_listener.assert_not_called()
            self.assertFalse(second.update([0,0,.8],can_begin=True,now=second.result['applied_monotonic_s']+.1))
            self.assertTrue(second.update([0,0,.8],can_begin=True,now=second.result['applied_monotonic_s']+.31))
            self.assertEqual(self.native.correct([1,2,3]),frozen)
            second.close()

    def saved(self, root):
        rows,candidate=diagonal_capture()
        fit=fit_hover_roll_pitch(rows,candidate,'identity')
        coefficients=firmware_xy_coefficients(candidate,fit,'identity')
        loaded=dict(api=1,coefficients=coefficients,firmware_applied=True,frozen=True,generation=7)
        path=root/'hover_xy.json'
        saved=save_fit(path,candidate,'identity','lb11',fit,loaded)
        return path,candidate,fit,loaded,saved

    def test_saved_identity_and_changed_coefficients_are_rejected(self):
        with TemporaryDirectory() as tmp:
            path,candidate,fit,loaded,saved=self.saved(Path(tmp))
            for fault in ('drone','fingerprint','coefficients','z','gyro','api','json'):
                with self.subTest(fault=fault):
                    changed=deepcopy(saved)
                    if fault=='drone':changed['drone_id']='lb12'
                    if fault=='fingerprint':changed['source_candidate_sha256']='other'
                    if fault=='coefficients':changed['coefficients']['bx']+=.1
                    if fault=='z':changed['fit']['additional_body_accel_bias_m_s2'][2]=.1
                    if fault=='gyro':changed['gyro_saved']=True
                    if fault=='api':changed['firmware_api']=2
                    path.write_text('{bad' if fault=='json' else json.dumps(changed))
                    with self.assertRaises((ValueError,KeyError,TypeError)):
                        read_saved_fit(path,candidate,'identity','lb11')

    def test_auto_refresh_reuse_and_failed_refresh_preserve_old_cache(self):
        with TemporaryDirectory() as tmp:
            path,candidate,fit,loaded,saved=self.saved(Path(tmp));before=path.read_bytes()
            source=path.with_name('candidate.json')
            auto,reason=select_saved_fit({},candidate,'identity',source,'lb11')
            self.assertEqual(auto,saved);self.assertEqual(reason,'saved')
            refresh,reason=select_saved_fit({'estimator_hover_xy_mode':'refresh'},candidate,'identity',source,'lb11')
            self.assertIsNone(refresh);self.assertEqual(reason,'manual_refresh')
            with self.assertRaisesRegex(ValueError,'acknowledgement'):
                save_fit(path,candidate,'identity','lb11',fit,{**loaded,'firmware_applied':False})
            self.assertEqual(path.read_bytes(),before)
            missing,_=select_saved_fit({},candidate,'different',source,'lb11')
            self.assertIsNone(missing)
            with self.assertRaisesRegex(ValueError,'no valid saved'):
                select_saved_fit({'estimator_hover_xy_mode':'reuse'},candidate,'different',source,'lb11')
            path.unlink()
            self.assertIsNone(select_saved_fit({},candidate,'identity',source,'lb11')[0])

    def test_cached_path_skips_extra_logs_and_refresh_requests_them(self):
        with TemporaryDirectory() as tmp:
            path,candidate,fit,loaded,saved=self.saved(Path(tmp))
            mission={'Interaction':{'config':{'behavior':'level_coast','level_coast':{
                'coast_command_mode':'scurve','estimator_hover_xy':True}}}}
            args=SimpleNamespace(vicon=True,calibrate=False,drone_id='lb11',cf_log_period=10)
            selected={'existing':{}}
            with patch('Interaction.estimator_hover_live.load_static_candidate',
                       return_value=(candidate,'identity',path.with_name('candidate.json'))), \
                 patch('Interaction.estimator_hover_live.capture_group_manifest') as manifest:
                self.assertIs(add_capture_logs(selected,mission,None,args),selected)
                manifest.assert_not_called()
                mission['Interaction']['config']['level_coast']['estimator_hover_xy_mode']='refresh'
                fresh=add_capture_logs(selected,mission,None,args)
                self.assertIn(IMU_GROUP,fresh);self.assertIn(KF_GROUP,fresh)
                manifest.assert_called_once()

    def test_missing_api_partial_write_and_rejected_commit_do_not_report_success(self):
        cf=SimpleNamespace(param=Params(self.native))
        del cf.param.toc.toc['eskfXY']['api']
        with self.assertRaisesRegex(RuntimeError,'lacks'):prepare_xy_loading(cf)
        self.assertEqual(cf.param.writes,[])
        params=Params(self.native);params.armed=True;cf=SimpleNamespace(param=params)
        real_write=params.set_value
        def broken(name,value):
            if name=='eskfXY.sy':raise OSError('partial transport failure')
            real_write(name,value)
        params.set_value=broken
        with self.assertRaisesRegex(OSError,'partial'):
            load_xy(cf,dict(sx=1,sy=1,bx=0,by=0))
        self.assertEqual(self.native.state()['on'],0)
        self.assertFalse(any(k=='eskfXY.req' for k,_ in params.writes))
        params=Params(self.native);params.armed=True;params.default=False
        with self.assertRaisesRegex(RuntimeError,'not acknowledged'):
            load_xy(SimpleNamespace(param=params),dict(sx=1,sy=1,bx=0,by=0),timeout_s=.01)
        self.assertEqual(self.native.state()['on'],0)
        self.assertEqual(params.callbacks,{})

    def test_estimator_selection_is_blocked_until_frozen_fit_is_ready(self):
        from Interaction.level_coast_scurve import ContactEstimatorSelector
        cf=SimpleNamespace(param=Mock(),_estimator_hover_xy=SimpleNamespace(ready=False))
        selector=ContactEstimatorSelector(cf);selector.prepared=True
        with self.assertRaisesRegex(RuntimeError,'not been acknowledged'):selector.request(True,10.)
        cf.param.set_value.assert_not_called()

    def test_config_is_opt_in_and_requires_scurve(self):
        self.assertFalse(requested(None))
        for value in ('true',1):
            with self.assertRaises(ValueError):requested({'Interaction':{'config':{'level_coast':{'estimator_hover_xy':value}}}})
        with self.assertRaisesRegex(ValueError,'scurve'):
            requested({'Interaction':{'config':{'behavior':'level_coast','level_coast':{'estimator_hover_xy':True}}}})
        with self.assertRaisesRegex(ValueError,'mode'):
            requested({'Interaction':{'config':{'level_coast':{'estimator_hover_xy':True,
                'estimator_hover_xy_mode':'invalid'}}}})


if __name__=='__main__':unittest.main()
