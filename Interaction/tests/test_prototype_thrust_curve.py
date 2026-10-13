import math
import numpy as np
from Interaction.prototype_thrust_curve import PrototypeThrustCurve


def test_linear_and_quadratic_motor_physics_are_distinct():
    linear=PrototypeThrustCurve(1000,4,power=1)
    quadratic=PrototypeThrustCurve(1000,4,power=2)
    assert abs(linear.update([500]*4,4,0)-9.81*.5)<1e-12
    assert abs(quadratic.update([500]*4,4,0)-9.81*.25)<1e-12
    assert abs(linear.update([1000]*4,8,1)-9.81*4)<1e-12


def test_first_order_motor_speed_produces_squared_thrust():
    curve=PrototypeThrustCurve(1000,4,power=1,time_constant_s=.04)
    assert curve.update([0]*4,4,0)==0
    assert abs(curve.update([1000]*4,4,.04*math.log(2))-9.81*.25)<1e-12


def test_reordered_and_invalid_motor_samples_are_rejected():
    curve=PrototypeThrustCurve(1000,4);curve.update([1000]*4,4,1)
    for pwm,t in [([1000]*4,0),([np.nan]*4,2),([-1]*4,2)]:
        try:curve.update(pwm,4,t)
        except ValueError:pass
        else:raise AssertionError('invalid motor sample accepted')
