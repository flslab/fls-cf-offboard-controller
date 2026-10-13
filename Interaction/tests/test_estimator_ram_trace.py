"""Wire, timestamp, frozen-download and real firmware-C recorder regressions."""
import json
from pathlib import Path
import shutil
import struct
import subprocess
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import Mock, patch
from types import SimpleNamespace

from Interaction.estimator_ram_trace import HEADER, RECORD, MAGIC, decode, save, validate_flight_trace, Recorder, validate_interaction_trace, interaction_step, finish_interaction_recorder
from Interaction.replay_estimator3 import run_replay
from Interaction.tests.test_estimator_imu_calibration import fixture_dataset
from Interaction.estimator_imu_calibration import fit_dataset

FIRMWARE=Path(__file__).resolve().parents[3]/'crazyflie-firmware'


def wire(records, *, hz=1000, **changes):
    header=dict(magic=MAGIC, version=1, record_bytes=40, capacity=640, count=len(records),
                overwritten=0,session=3,status=2,imu_hz=hz,start_us=1000000,stop_us=1100000,
                unmatched_imu=0,queue_rejected=0,decimated=0)
    header.update(changes)
    return HEADER.pack(*header.values())+b''.join(RECORD.pack(k,t&0xffffffff,s&0xffffffff,*(d+[0.]*(7-len(d))))
                                                 for k,t,s,d in records)


def flight_records():
    records=[]
    for i in range(300):
        t=1000000+i*1000
        if i%20==0:
            records.extend([(3,t,t,[1,0,0,0]),(4,t,t,[0,0,1,0,0,0]),(0x102,t,t,[0,0,1,.01])])
        records.append((0x301,t+100,t,[0,0,0,0,0,1]))
    return records


class WireTests(unittest.TestCase):
    def test_microsecond_timestamps_and_wrap_are_preserved(self):
        base=(1<<32)-1000
        header, rows=decode(wire([(3,base,base,[1,0,0,0]),(4,base,base,[0]*6),
                                  (0x301,base+100,base,[0,0,0,0,0,1]),
                                  (0x301,base+1100,base+1000,[0,0,0,0,0,1])]))
        imu=[r for r in rows if r['group']=='imu']
        self.assertEqual([r['device_time_us'] for r in imu],[0,1000])
        self.assertEqual(header['maximum_imu_gap_us'],1000)
        self.assertFalse(header['decimation_enabled'])

    def test_rejected_queue_inputs_are_explicit_and_not_replayed(self):
        header, rows=decode(wire([(0x101,1000,1000,[0,0,0,0,0,1]),(2,1100,1100,[0,0,0])],queue_rejected=2))
        self.assertEqual(header['rejected_input_records'],2)
        self.assertFalse(any(r['group']=='imu' for r in rows))

    def test_corrupt_truncated_unfrozen_and_future_inputs_rejected(self):
        valid=wire(flight_records())
        variants=[valid[:-1],valid+b'x',wire(flight_records(),status=1),wire(flight_records(),version=99),
                  wire([(0x301,1000,1001,[0,0,0,0,0,1])]),wire([(9,1000,1000,[0])]),
                  wire([(0x301,1000,1000,[float('nan')])]),
                  wire([(0x301,2000,2000,[0]*6),(0x301,1000,1000,[0]*6)])]
        for raw in variants:
            with self.subTest(size=len(raw)),self.assertRaises(ValueError):decode(raw)

    def test_invalid_capture_keeps_original_binary(self):
        with TemporaryDirectory() as d:
            output=Path(d)/'capture'
            with self.assertRaises(ValueError):save(b'broken',output)
            self.assertEqual((output/'trace.bin').read_bytes(),b'broken')

    def test_flight_config_rejects_unsafe_phase_and_invalid_rates(self):
        config={'phase':'+Y_out','hz':1000,'duration_ms':600,'offset_s':1.}
        validate_flight_trace(config)
        for change in ({'hz':True},{'hz':100},{'phase':'land'},{'offset_s':3},{'duration_ms':4000}):
            with self.subTest(change=change),self.assertRaises(ValueError):validate_flight_trace({**config,**change})

    def test_async_start_does_not_block_for_readback(self):
        cf=Mock();cf.param.toc.toc={'ramTrace':dict.fromkeys(('mode','imuHz','durationMs','status','session'))}
        recorder=Recorder(cf)
        with self.assertRaises(RuntimeError):recorder.start_async(1000,600)
        recorder.prepared=(1000,600)
        recorder.prepared_session=0
        recorder.start_async(1000,600)
        cf.param.set_value.assert_called_once_with('ramTrace.mode','1')
        cf.param.request_param_update.assert_not_called()

    def test_interaction_starts_once_and_triggers_only_first_release(self):
        recorder=Mock();recorder.prepared=(1000,0)
        cf=SimpleNamespace(_interaction_ram_recorder=recorder,_interaction_ram_started=False,_interaction_ram_released=False)
        owner=SimpleNamespace(cf=cf,_log_event=Mock())
        for phase,released in [('prepare',False),('ready',False),('contact',False),('coast',True),('ready',False),('coast',True)]:
            interaction_step(owner,phase,released)
        recorder.start_async.assert_called_once_with(1000,0)
        recorder.release_async.assert_called_once()
        self.assertEqual(owner._log_event.call_count,2)

    def test_interaction_rejects_scurve_and_unconfirmed_landing_never_downloads(self):
        with self.assertRaises(ValueError):validate_interaction_trace({},'scurve')
        self.assertIsNone(validate_interaction_trace(None,'scurve'))
        for bad in ({'hz':100},{'hz':True},{'post_release_ms':-1},{'bad':1}):
            with self.assertRaises(ValueError):validate_interaction_trace(bad,'orientation')
        recorder=Mock()
        controller=SimpleNamespace(cf=SimpleNamespace(_interaction_ram_recorder=recorder,_interaction_ram_started=True),flying=True)
        with self.assertRaises(RuntimeError):finish_interaction_recorder(controller)
        recorder.freeze_async.assert_called_once()
        recorder.download.assert_not_called()

    def test_real_C_replay_keeps_1ms_spacing(self):
        data,*_=fixture_dataset()
        candidate=fit_dataset(data)
        with TemporaryDirectory() as d:
            root=Path(d)
            capture=root/'ram'
            save(wire(flight_records()),capture)
            (root/'candidate.json').write_text(json.dumps(candidate))
            report=run_replay(capture/'packets.jsonl',root/'candidate.json',root/'replay')
            samples=report['raw']['samples']
            self.assertGreater(len(samples),90)
            self.assertAlmostEqual(samples[1]['time_s']-samples[0]['time_s'],.001)
            self.assertEqual(report['raw']['gap_limit_us'],3000)
            self.assertEqual(report['raw']['segments'],1)
            self.assertGreater(samples[-1]['position_fusions'],1)
            self.assertEqual(report['ram_capture']['imu_hz'],1000)

    def test_stale_or_changing_download_session_is_rejected(self):
        raw=wire(flight_records())
        for stale in (True,False):
            cf=Mock();cf.param.toc.toc={'ramTrace':dict.fromkeys(('mode','imuHz','durationMs','status','session'))}
            cf.mem.mems=[SimpleNamespace(id=1,type=0x18,size=30000)]
            recorder=Recorder(cf)
            recorder.expected_session=4 if stale else 3
            changed=wire(flight_records(),session=4)
            with patch.object(recorder,'_get',return_value=2),patch.object(recorder,'_read',side_effect=[raw[:HEADER.size],raw,changed[:HEADER.size]]),self.assertRaises(RuntimeError):
                recorder.download()

    def test_download_checks_frozen_state_without_reading_live_buffer(self):
        cf=Mock();cf.param.toc.toc={'ramTrace':dict.fromkeys(('mode','imuHz','durationMs','status','session'))}
        recorder=Recorder(cf)
        with patch.object(recorder,'_get',return_value=1),self.assertRaises(RuntimeError):recorder.download()
        cf.mem.read.assert_not_called()


class FirmwareCoreTests(unittest.TestCase):
    def test_real_recorder_wrap_freeze_pairing_and_estimator_guard(self):
        if not FIRMWARE.exists():self.skipTest('sibling firmware checkout needed for recorder C regression')
        with TemporaryDirectory() as d:
            root=Path(d)
            # Stand-in OS/firmware types; the production recorder C is compiled unchanged.
            stub = r"""
#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
typedef struct {float x,y,z;} V;
typedef struct {V gyro;uint32_t timestampUs;} gyroscopeMeasurement_t;
typedef struct {V acc;uint32_t timestampUs;} accelerationMeasurement_t;
enum {MeasurementTypeGyroscope,MeasurementTypeAcceleration,MeasurementTypePosition,MeasurementTypePose};
enum {MeasurementSourceLocationService=4};
typedef struct {int type;union {gyroscopeMeasurement_t gyroscope;accelerationMeasurement_t acceleration;
struct {float pos[3],stdDev;int source;} position;struct {float pos[3],stdDevPos;} pose;} data;} measurement_t;
typedef struct {struct {float w,x,y,z;} attitudeQuaternion;V position,velocity;} state_t;
typedef struct {int type;uint32_t (*getSize)(uint8_t);bool (*read)(uint8_t,uint32_t,uint8_t,uint8_t*);} MemoryHandlerDef_t;
#define MEM_TYPE_APP 24
#define taskENTER_CRITICAL() ((void)0)
#define taskEXIT_CRITICAL() ((void)0)
#define PARAM_GROUP_START(x)
#define PARAM_GROUP_STOP(x)
#define PARAM_ADD(a,b,c)
static uint32_t clockUs;
static uint64_t usecTimestamp(void){return clockUs;}
static void memoryRegisterHandler(const MemoryHandlerDef_t *x){(void)x;}
"""
            (root/'stub.h').write_text(stub)
            for name in ('estimator_ram_trace.h','FreeRTOS.h','task.h','mem.h','param.h','usec_time.h'):
                (root/name).write_text('#include "stub.h"\n')
            harness=r"""
#include <assert.h>
#include "ram_trace_buffer.h"
#include "ADAPTER"
int main(void){
  RamTraceBuffer b={0};uint8_t out[80];RamTraceRecord r={.kind=1};
  assert(!ramTraceRead(&b,0,1,out));ramTraceBegin(&b,0,1000);
  for(unsigned i=0;i<8;i++){r.sampleUs=i;ramTraceAppend(&b,&r);}
  assert(b.header.overwritten==4);ramTraceFreeze(&b,9);
  RamTraceRecord first;assert(ramTraceRead(&b,sizeof(RamTraceHeader),sizeof(first),(uint8_t*)&first));
  assert(first.sampleUs==4);assert(!ramTraceRead(&b,0,1000,out));
  ramTraceAppend(&b,&r);assert(b.header.count==4);
  ramTraceBegin(&b,10,250);assert(b.header.count==0&&b.header.session==2);
  state_t state={.attitudeQuaternion.w=1};requestedMode=1;clockUs=1000;
  estimatorRamTraceService(&state,2);assert(trace.header.status==1);
  measurement_t m={.type=MeasurementTypeGyroscope};m.data.gyroscope.timestampUs=2000;
  clockUs=2010;estimatorRamTraceMeasurement(&m,true);
  m.type=MeasurementTypeAcceleration;m.data.acceleration.timestampUs=2000;m.data.acceleration.acc.z=1;
  estimatorRamTraceMeasurement(&m,true);assert(trace.header.decimated==0);
  m.type=MeasurementTypeGyroscope;m.data.gyroscope.timestampUs=2900;clockUs=2910;
  estimatorRamTraceMeasurement(&m,true);m.type=MeasurementTypeAcceleration;m.data.acceleration.timestampUs=2900;
  estimatorRamTraceMeasurement(&m,false);assert(trace.header.decimated==0&&trace.header.queueRejected==1);
  assert(trace.records[(trace.next+3)%4].kind==0x101);
  clockUs=3000;estimatorRamTraceService(&state,3);assert(trace.header.status==2&&requestedMode==2);
  assert(readMemory(0,0,30,out));assert(!readMemory(0,0,31,out));
  requestedMode=1;clockUs=4000;estimatorRamTraceService(&state,3);
  assert(trace.header.status==2&&trace.header.session==1);
  requestedMode=1;requestedHz=250;durationMs=1;clockUs=5000;
  estimatorRamTraceService(&state,2);
  m.type=MeasurementTypeAcceleration;m.data.acceleration.timestampUs=5050;clockUs=5060;
  estimatorRamTraceMeasurement(&m,true);assert(trace.header.unmatchedImu==1);
  clockUs=6000;estimatorRamTraceService(&state,2);assert(trace.header.status==2);
  requestedMode=1;requestedHz=250;durationMs=0;clockUs=10000;trigger=0;postMs=1;
  estimatorRamTraceService(&state,2);
  for(unsigned i=0;i<2;i++){
    clockUs=11000+i*1000;m.type=MeasurementTypeGyroscope;m.data.gyroscope.timestampUs=clockUs;
    estimatorRamTraceMeasurement(&m,true);m.type=MeasurementTypeAcceleration;m.data.acceleration.timestampUs=clockUs;
    estimatorRamTraceMeasurement(&m,true);
  }
  assert(trace.header.decimated==1);trigger=1;clockUs=13000;
  estimatorRamTraceService(&state,2);assert(trace.header.status==1);
  clockUs=14000;estimatorRamTraceService(&state,2);assert(trace.header.status==2);
  assert(trace.records[(trace.next+3)%4].data[0]==7);
  return 0;
}
""".replace('ADAPTER',str(FIRMWARE/'src/modules/src/estimator_ram_trace.c'))
            (root/'test.c').write_text(harness)
            command=[shutil.which('cc'),'-std=c11','-Wall','-Wextra','-Werror','-DRAM_TRACE_CAPACITY=4',
                '-I'+str(root),'-I'+str(FIRMWARE/'src/modules/interface'),str(root/'test.c'),
                str(FIRMWARE/'src/modules/src/ram_trace_buffer.c'),'-o',str(root/'test')]
            subprocess.run(command,check=True,capture_output=True,text=True)
            subprocess.run([str(root/'test')],check=True,capture_output=True,text=True)

if __name__=='__main__':unittest.main()
