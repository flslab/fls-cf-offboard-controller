"""Specific-force balance shared by Gazebo and opt-in offboard model detection.

The primitive itself has no command authority. Hardware thrust/IMU calibration
is independent of simulation; replay does not validate flight timing or noise.
"""
from collections import deque
from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True)
class SpecificForceEstimate:
    external_force: np.ndarray
    instantaneous_force: np.ndarray
    ready: bool
    window_s: float
    rejected: bool

class SpecificForceContactObserver:
    def __init__(self,mass,window_s=.08,minimum_window_s=.05,max_dt_s=.05,imu_from_com_m=(0.,0.,0.),averaging=True):
        self.mass=float(mass);self.window=float(window_s);self.minimum=float(minimum_window_s);self.max_dt=float(max_dt_s)
        self.offset=np.asarray(imu_from_com_m,float)
        self.averaging = bool(averaging)
        if not np.isfinite([self.mass,self.window,self.minimum,self.max_dt]).all() or not (self.mass>0 and self.window>0 and self.minimum>0 and self.max_dt>0) or (self.averaging and self.window<self.minimum) or self.offset.shape!=(3,) or not np.isfinite(self.offset).all():
            raise ValueError('invalid specific-force observer configuration')
        self.samples=deque();self.previous_gyro=None;self.previous_time=None

    def reset(self):
        self.samples.clear();self.previous_gyro=None;self.previous_time=None

    @staticmethod
    def rotation(q):
        q=np.asarray(q,float)
        if q.shape!=(4,) or not np.isfinite(q).all() or np.linalg.norm(q)<1e-12:raise ValueError('invalid quaternion')
        x,y,z,w=q/np.linalg.norm(q)
        return np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
            [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
            [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])

    def update(self,accel_m_s2,quaternion_xyzw,thrust_accel_m_s2,timestamp,gyro_rad_s=(0.,0.,0.)):
        t=float(timestamp);acc=np.asarray(accel_m_s2,float);gyro=np.asarray(gyro_rad_s,float);thrust=float(thrust_accel_m_s2)
        if acc.shape!=(3,) or gyro.shape!=(3,) or not np.isfinite(np.r_[acc,gyro,t,thrust]).all() or thrust<0:
            self.reset()
            raise ValueError('invalid IMU/thrust sample')
        dt=None if self.previous_time is None else t-self.previous_time
        if dt is not None and dt<=0:
            return SpecificForceEstimate(np.zeros(3),np.zeros(3),False,0.,True)
        discontinuity=dt is not None and dt>self.max_dt+1e-12
        if discontinuity:self.samples.clear();self.previous_gyro=None
        alpha=np.zeros(3) if self.previous_gyro is None else (gyro-self.previous_gyro)/dt
        rotational=np.cross(alpha,self.offset)+np.cross(gyro,np.cross(gyro,self.offset))
        # specific force = body thrust/m + body external force/m. Gravity
        # cancels algebraically; no differentiated filtered velocity enters.
        try:
            instantaneous=self.mass*self.rotation(quaternion_xyzw)@(acc-rotational-[0.,0.,thrust])
        except ValueError:
            self.reset()
            raise
        if not self.averaging:
            self.previous_gyro=gyro.copy();self.previous_time=t
            return SpecificForceEstimate(instantaneous,instantaneous.copy(),not discontinuity,0.,discontinuity)
        self.samples.append((t,instantaneous.copy()));self.previous_gyro=gyro.copy();self.previous_time=t
        while len(self.samples)>2 and self.samples[1][0]<=t-self.window:self.samples.popleft()
        duration=min(self.window,t-self.samples[0][0]);total=np.zeros(3)
        start=t-duration
        for (ta,fa),(tb,_) in zip(self.samples,list(self.samples)[1:]):
            total+=fa*max(0.,tb-max(ta,start))
        ready=duration>=self.minimum and not discontinuity
        return SpecificForceEstimate(total/duration if ready else np.zeros(3),instantaneous,ready,duration,discontinuity)
