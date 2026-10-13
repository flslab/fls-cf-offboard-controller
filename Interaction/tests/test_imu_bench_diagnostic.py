"""Clock, sampling, fault preservation and command-free capture checks."""
from contextlib import contextmanager, redirect_stdout
from argparse import Namespace
import io
import json
from pathlib import Path
from tempfile import TemporaryDirectory
import time
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np

from Interaction.imu_bench_diagnostic import FIELDS, MOTORS, Recorder, capture, save_json, radio_packets
from Interaction.imu_bench_report import causal_average, decode, sampling_comparison, analyze


def row(epoch, tick=None, stage='gentle_taps', ay=0.):
    data = dict(zip(FIELDS, [0., 0., 0., 0., ay/9.81, 1.]))
    data['contactImu.epoch'] = epoch
    return {'kind': 'imu', 'data': data, 'cf_log_tick_ms_mod24': tick if tick is not None else epoch,
            'host_monotonic_ns': time.monotonic_ns(), 'stage': stage}


class AnalysisTests(unittest.TestCase):
    def test_producer_wrap_and_duplicate(self):
        rows = [row(65534, 100), row(65535, 101), row(0, 102), row(0, 103), row(2, 104)]
        samples, issues = decode(rows)
        np.testing.assert_allclose([s['t'] for s in samples], [0,.001,.002,.004])
        self.assertEqual(issues['duplicate_epochs'], 1)
        self.assertEqual(issues['clock_discontinuities'], 0)

    def test_clock_reset_splits_segment(self):
        samples, issues = decode([row(200, 400), row(201, 401), row(0, 0), row(1, 1)])
        self.assertEqual(issues['clock_discontinuities'], 1)
        self.assertNotEqual(samples[1]['segment'], samples[2]['segment'])

    def test_nan_rejected_and_breaks_continuity(self):
        samples, issues = decode([row(1), row(2, ay=float('nan')), row(3)])
        self.assertEqual(issues['invalid_payloads'], 1)
        self.assertNotEqual(samples[0]['segment'], samples[1]['segment'])

    def test_causal_mean_is_left_rectangle_not_future_or_sample_mean(self):
        t = np.arange(0, .201, .001)
        y = np.column_stack([t, t*0+3, t*0])
        mean = causal_average(t,y)
        self.assertTrue(np.isnan(mean[79]).all())
        np.testing.assert_allclose(mean[100], [.0595,3,0], atol=1e-12)
        y[101:] = 1e6
        np.testing.assert_allclose(causal_average(t,y)[100], mean[100])

    def test_average_does_not_bridge_gap(self):
        t = np.r_[np.arange(0,.101,.001), np.arange(.15,.301,.001)]
        avg = causal_average(t,np.ones((len(t),3)))
        self.assertTrue(np.isnan(avg[(t>=.15)&(t<.23)]).all())
        np.testing.assert_allclose(avg[t>.24],1)

    def test_100hz_phase_error_zero_constant_and_nonzero_oscillation(self):
        t=np.arange(0,2,.001)
        v=sampling_comparison(t,np.ones((len(t),3)))
        self.assertEqual(len(v),10)
        self.assertLess(max(max(a['max_abs_difference_m_s2']) for a in v),1e-10)
        signal=np.tile(np.sin(2*np.pi*19*t)[:,None],(1,3))
        v=sampling_comparison(t,signal)
        self.assertGreater(max(a['p95_abs_difference_m_s2'][1] for a in v),.01)

    def test_report_on_partial_and_truncated_log(self):
        with TemporaryDirectory() as temp:
            p=Path(temp)
            save_json(p/'session.json',{'status':'interrupted','telemetry':{'requested_period_ms':1}})
            (p/'packets.jsonl').write_text('\n'.join(json.dumps(row(i,ay=np.sin(i*.119))) for i in range(1500))+'\n{"broken":')
            report=analyze(p)
            self.assertEqual(report['issues']['truncated_json_lines'],1)
            self.assertFalse(report['stages'][2]['high_rate_qualified'])
            self.assertTrue((p/'report.html').exists())
            self.assertEqual(report['capture_status'],'interrupted')


class CaptureTests(unittest.TestCase):
    def test_radio_transport_only_subscribes_and_cleans_up(self):
        # The fake aircraft deliberately has no commander/platform/param interface.
        class Callbacks:
            def add_callback(self, cb): pass
            def remove_callback(self, cb): pass
        class Block:
            def __init__(self, name, period):
                self.name, self.period, self.fields = name, period, []
                self.data_received_cb = Callbacks(); self.error_cb = Callbacks()
                self.started = self.stopped = self.deleted = False
            def add_variable(self, field, dtype): self.fields.append((field,dtype))
            def start(self): self.started = True
            def stop(self): self.stopped = True
            def delete(self): self.deleted = True
        blocks=[]; toc={}
        for field in FIELDS+MOTORS+['contactImu.epoch']:
            group,name=field.split('.')
            toc.setdefault(group,{})[name]=SimpleNamespace(ctype='float')
        cf=SimpleNamespace(disconnected=Callbacks(),log=SimpleNamespace(
            toc=SimpleNamespace(toc=toc),add_config=blocks.append))
        @contextmanager
        def connect(*args,**kwargs): yield SimpleNamespace(cf=cf)
        with patch('cflib.crtp.init_drivers'), patch('cflib.crazyflie.Crazyflie',return_value=cf), \
             patch('cflib.crazyflie.syncCrazyflie.SyncCrazyflie',connect), \
             patch('Interaction.imu_bench_diagnostic.fls_log_config',Block):
            with radio_packets('fake://',1) as (_,metadata):
                self.assertEqual(metadata['requested_period_ms'],1)
        self.assertEqual([(b.name,b.period) for b in blocks],[('imu',1)])
        self.assertTrue(all((m,'uint16_t') in blocks[0].fields for m in MOTORS))
        self.assertTrue(all(b.started and b.stopped and b.deleted for b in blocks))

    def test_frozen_source_clock_is_rejected_despite_live_packets(self):
        with TemporaryDirectory() as temp:
            count=0
            def read(timeout):
                nonlocal count
                time.sleep(.002); count+=1
                if count%10==1:
                    return {'kind':'motors','data':dict.fromkeys(MOTORS,0),
                            'cf_log_tick_ms_mod24':count,'host_monotonic_ns':time.monotonic_ns()}
                return row(1,count)
            recorder=Recorder(Path(temp)/'packets.jsonl',read)
            recorder.start(); recorder.thread.join(2)
            self.assertFalse(recorder.ready.is_set())
            with self.assertRaisesRegex(RuntimeError,'stopped advancing'):
                recorder.close()

    def test_rejected_confirmation_never_connects_and_is_saved(self):
        with TemporaryDirectory() as temp:
            p=Path(temp)/'session'
            args=Namespace(output=p,uri='fake://',drone_id='test',period_ms=1,vicon_host=None)
            def forbidden(*_):
                self.fail('must not connect')
            with redirect_stdout(io.StringIO()):
                status=capture(args,prompt=lambda _: 'no',source=forbidden)
            self.assertEqual(status,1)
            self.assertEqual(json.loads((p/'session.json').read_text())['status'],'failed')

    def test_existing_output_not_overwritten(self):
        with TemporaryDirectory() as temp:
            args=Namespace(output=Path(temp),uri='fake://',drone_id='test',period_ms=1,vicon_host=None)
            with self.assertRaises(FileExistsError):
                capture(args)

    def test_motor_failure_preserves_trigger_packet(self):
        with TemporaryDirectory() as temp:
            r={'kind':'motors','data':dict.fromkeys(MOTORS,1),'cf_log_tick_ms_mod24':0,
               'host_monotonic_ns':time.monotonic_ns()}
            recorder=Recorder(Path(temp)/'packets.jsonl',lambda timeout:r)
            recorder.start(); recorder.thread.join(1)
            with self.assertRaisesRegex(RuntimeError,'motor output'):
                recorder.close()
            self.assertEqual(json.loads(recorder.path.read_text())['data']['motor.m1'],1)

    def test_motor_guard_runs_on_same_packet_as_imu(self):
        with TemporaryDirectory() as temp:
            count=0
            def read(timeout):
                nonlocal count
                count+=1; time.sleep(.002)
                packet=row(count)
                packet['data'].update(dict.fromkeys(MOTORS,0 if count<5 else 100))
                return packet
            recorder=Recorder(Path(temp)/'packets.jsonl',read)
            recorder.start(); recorder.thread.join(1)
            self.assertTrue(recorder.ready.is_set())
            with self.assertRaisesRegex(RuntimeError,'motor output'):
                recorder.close()
            self.assertEqual(recorder.counts['motor_checks'],4)

    def test_no_packets_times_out_and_closes_file(self):
        with TemporaryDirectory() as temp:
            def read(timeout):
                time.sleep(timeout)
                return None
            recorder=Recorder(Path(temp)/'packets.jsonl',read)
            recorder.start(); recorder.thread.join(2)
            self.assertFalse(recorder.thread.is_alive())
            with self.assertRaisesRegex(RuntimeError,'zero-motor'):
                recorder.close()
            self.assertTrue(recorder.path.exists())

    def test_simulated_capture_all_stages_no_hardware(self):
        with TemporaryDirectory() as temp:
            p=Path(temp)/'session'
            args=Namespace(output=p,uri='fake://',drone_id='test',period_ms=1,vicon_host=None)
            @contextmanager
            def source(uri,period):
                start=time.monotonic()
                count=0
                def read(timeout):
                    nonlocal count
                    time.sleep(.001)
                    tick=int((time.monotonic()-start)*1000); count+=1
                    if count%10==1:
                        return {'kind':'motors','data':dict.fromkeys(MOTORS,0),
                                'cf_log_tick_ms_mod24':tick,'host_monotonic_ns':time.monotonic_ns()}
                    return row(tick%65536,tick)
                yield read,{'requested_period_ms':1}
            short=[(name,.05,'test') for name in ('static_before','slow_roll','gentle_taps','static_after')]
            with patch('Interaction.imu_bench_diagnostic.PLAN',short), redirect_stdout(io.StringIO()):
                status=capture(args,prompt=lambda s:'PROPS_OFF' if 'PROPS_OFF' in s else '',source=source)
            self.assertEqual(status,0)
            manifest=json.loads((p/'session.json').read_text())
            self.assertTrue(all(s['completed'] for s in manifest['stages']))
            self.assertFalse(manifest['command_authority'])
            self.assertFalse(manifest['calibration_applied'])
            self.assertTrue((p/'report.html').exists())


if __name__=='__main__':
    unittest.main()
