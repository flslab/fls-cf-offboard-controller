import datetime
import json
import logging
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils import uri_helper
import matplotlib.pyplot as plt
import numpy as np


URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E701')
DEFAULT_HEIGHT = 0.5
deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)
_time = []
_thrust = []
_z = []


def param_deck_flow(_, value_str):
    value = int(value_str)
    print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(20)
        # mc.up(0.25)
        # time.sleep(5)
        mc.stop()


def move_circle(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        mc.circle_right(.5, velocity=1.6)


def set_pid_values():
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        # Set new PID gains for Z position
        default_P_gain = 2.0
        default_I_gain = 0.5
        default_D_gain = 0.0
        new_P_gain = 2.0
        new_I_gain = 0.0
        new_D_gain = 0.0

        # print(cf.param.get_value('posCtlPid.zKp'))
        print('posCtlPid.zKi', cf.param.get_value('posCtlPid.zKi'))
        # print(cf.param.get_value('posCtlPid.zKd'))
        # cf.param.set_value('posCtlPid.zKp', str(new_P_gain))
        cf.param.set_value('posCtlPid.zKi', str(new_I_gain))
        # cf.param.set_value('posCtlPid.zKd', str(new_D_gain))
        print('posCtlPid.zKi', cf.param.get_value('posCtlPid.zKi'))


def set_controller():
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # Controller type Auto select(0), PID(1), Mellinger(2), INDI(3), Brescianini(4)(Default: 0)

        print('stabilizer.controller', cf.param.get_value('stabilizer.controller'))
        cf.param.set_value('stabilizer.controller', 1)
        print('stabilizer.controller', cf.param.get_value('stabilizer.controller'))


def log_motor_callback(timestamp, data, logconf):
    # print(timestamp, data)
    global _time, _thrust, _z
    _time.append(timestamp)
    _thrust.append(data['stabilizer.thrust'])
    _z.append(data['stateEstimate.z'])

    # motor_thrusts[0].append(data['motor.m1'])
    # motor_thrusts[1].append(data['motor.m2'])
    # motor_thrusts[2].append(data['motor.m2'])
    # motor_thrusts[3].append(data['motor.m2'])


def plot_metrics():
    filename = f"{datetime.datetime.now():%Y_%m_%d_%H_%M_%S}"
    fig, ax = plt.subplots()
    ax.plot((np.array(_time) - _time[0]) / 1000, np.array(_thrust) / 0xffff * 100, label='thrust (%)')
    ax.plot((np.array(_time) - _time[0]) / 1000, np.array(_z) * 100, label='z (cm)')
    ax.set_xlabel('Time (s)')
    ax.set_ylim([0, 150])
    plt.legend()
    plt.savefig(f'metrics/{filename}.png', dpi=300)
    with open(f'metrics/{filename}.json', 'w') as f:
        json.dump([_time, _thrust, _z], f)


if __name__ == '__main__':
    cflib.crtp.init_drivers()

    # set_controller()
    # set_pid_values()

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

        cf = scf.cf

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                         cb=param_deck_flow)
        time.sleep(1)

        logconf = LogConfig(name='Motor', period_in_ms=10)
        logconf.add_variable('stabilizer.thrust', 'float')
        logconf.add_variable('stateEstimate.z', 'float')
        # logconf.add_variable('motor.m3', 'float')
        # logconf.add_variable('motor.m4', 'float')
        scf.cf.log.add_config(logconf)
        logconf.data_received_cb.add_callback(log_motor_callback)

        if not deck_attached_event.wait(timeout=5):
            print('No flow deck detected!')
            sys.exit(1)

        logconf.start()
        take_off_simple(scf)
        logconf.stop()

    plot_metrics()
