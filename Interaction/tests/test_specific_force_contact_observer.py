import numpy as np
from Interaction.specific_force_contact_observer import SpecificForceContactObserver

def test_tilted_actuated_flight_without_contact_is_zero():
    o=SpecificForceContactObserver(.17)
    q=np.array([.1,-.2,.05,.97]);q/=np.linalg.norm(q)
    for t in np.arange(0,.3,.01):r=o.update([0,0,10.7],q,10.7,t)
    assert r.ready and np.linalg.norm(r.external_force)<1e-12

def test_known_contact_clears_after_window_without_velocity():
    o=SpecificForceContactObserver(.17)
    for t in np.arange(0,.5,.01):
        f=.4 if .1<=t<.2 else 0
        r=o.update([f/.17,0,9.81],[0,0,0,1],9.81,t)
        if .18<t<.2:assert abs(r.external_force[0]-.4)<1e-8
        if t>=.3:assert np.linalg.norm(r.external_force)<1e-8

def test_rotating_offset_compensation_does_not_report_contact():
    offset=np.array([.02,-.01,0]);o=SpecificForceContactObserver(.17,imu_from_com_m=offset)
    for i,t in enumerate(np.arange(0,.3,.01)):
        gyro=np.array([0,0,2+t]);alpha=np.array([0,0,0 if i==0 else 1.])
        acc=np.array([0,0,9.81])+np.cross(alpha,offset)+np.cross(gyro,np.cross(gyro,offset))
        r=o.update(acc,[0,0,0,1],9.81,t,gyro)
    assert r.ready and np.linalg.norm(r.external_force)<1e-10

def test_gap_resets_history_and_duplicate_is_rejected():
    o=SpecificForceContactObserver(.17)
    for t in np.arange(0,.2,.01):o.update([1,0,9.81],[0,0,0,1],9.81,t)
    r=o.update([0,0,9.81],[0,0,0,1],9.81,.4)
    assert r.rejected and not r.ready
    r=o.update([0,0,9.81],[0,0,0,1],9.81,.4)
    assert r.rejected and not r.ready

def test_nonfinite_configuration_and_input_are_rejected():
    for kwargs in [{'mass':float('inf')},{'mass':.17,'max_dt_s':float('nan')}]:
        try:SpecificForceContactObserver(**kwargs)
        except ValueError:pass
        else:raise AssertionError('invalid configuration accepted')
    o=SpecificForceContactObserver(.17)
    for acc,q in [([float('nan'),0,9.81],[0,0,0,1]),([0,0,9.81],[0,0,0,0])]:
        try:o.update(acc,q,9.81,0.)
        except ValueError:pass
        else:raise AssertionError('invalid sensor sample accepted')
    r=o.update([0,0,9.81],[0,0,0,1],9.81,.4)
    assert not r.rejected and not r.ready
