"""Simulation candidate calibrated from no-contact IMU/motor captures.

Not used by controller.py or an active flight YAML. The voltage dependence
cannot be identified from constant-voltage SITL data and is fixed explicitly.
"""
import numpy as np


class PrototypeThrustCurve:
    def __init__(self, hover_pwm, hover_voltage, power=2., time_constant_s=0., acceleration_scale=1.):
        self.hover_pwm=float(hover_pwm);self.hover_voltage=float(hover_voltage)
        self.power=float(power);self.tau=float(time_constant_s);self.gain=float(acceleration_scale)
        if not np.isfinite([self.hover_pwm,self.hover_voltage,self.power,self.tau,self.gain]).all() or min(self.hover_pwm,self.hover_voltage,self.power,self.gain)<=0 or self.tau<0:
            raise ValueError('invalid prototype thrust curve')
        self.speed=None;self.previous_time=None

    def update(self,pwm,voltage,timestamp):
        p=np.asarray(pwm,float);t=float(timestamp);voltage=float(voltage)
        if p.shape!=(4,) or not np.isfinite(np.r_[p,t,voltage]).all() or np.min(p)<0 or voltage<=0:
            raise ValueError('invalid motor sample')
        target=(p/self.hover_pwm)**(self.power/2)*(voltage/self.hover_voltage)
        if self.previous_time is not None and t<self.previous_time:raise ValueError('motor timestamp reordered')
        dt=0. if self.previous_time is None else t-self.previous_time
        if self.speed is None or self.tau==0:self.speed=target
        elif dt>0:self.speed+=(-np.expm1(-dt/self.tau))*(target-self.speed)
        self.previous_time=t
        return self.gain*9.81*float(np.mean(self.speed**2))
