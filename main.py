# sample usage for plotting
# python3 main.py -p [path_to_json_file]


import datetime
import json
import logging
import os
import sys
import time
from threading import Event

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper
import matplotlib.pyplot as plt
import numpy as np
import argparse


URI = uri_helper.uri_from_env(default='radio://0/100/2M/E7E7E7E701')
DEFAULT_HEIGHT = 0.5
DURATION = 10
deck_attached_event = Event()
logging.basicConfig(level=logging.ERROR)
_time = []


log_vars = {
    "controller.cmd_roll": {
        "type": "float",
        "unit": "cmd",
        "data": [],
    },
    "controller.cmd_pitch": {
        "type": "float",
        "unit": "cmd",
        "data": [],
    },
    "controller.cmd_yaw": {
        "type": "float",
        "unit": "cmd",
        "data": [],
    },
}


def param_deck_flow(_, value_str):
    value = int(value_str)
    # print(value)
    if value:
        deck_attached_event.set()
        print('Deck is attached!')
    else:
        print('Deck is NOT attached!')


def take_off_simple(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(DURATION)
        # mc.up(0.25)
        # time.sleep(5)
        mc.stop()


def move_circle(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        mc.circle_right(.4)


def test(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(2)
        mc.forward(.5)
        time.sleep(2)


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
        print('pid_attitude.roll_kp', cf.param.get_value('pid_attitude.roll_kp'))
        # print(cf.param.get_value('posCtlPid.zKd'))
        # cf.param.set_value('posCtlPid.zKp', str(new_P_gain))
        # cf.param.set_value('posCtlPid.zKi', str(new_I_gain))
        # cf.param.set_value('posCtlPid.zKd', str(new_D_gain))
        # cf.param.set_value('pid_attitude.roll_kp', str(9))
        # cf.param.set_value('pid_attitude.pitch_kp', str(9))
        time.sleep(2)
        # print('pid_attitude.roll_kd', cf.param.get_value('pid_attitude.roll_kd'))


def set_controller():
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # Controller type Auto select(0), PID(1), Mellinger(2), INDI(3), Brescianini(4)(Default: 0)

        print('stabilizer.controller', cf.param.get_value('stabilizer.controller'))
        cf.param.set_value('stabilizer.controller', 1)
        print('stabilizer.controller', cf.param.get_value('stabilizer.controller'))


def log_motor_callback(timestamp, data, logconf):
    # print(timestamp, data)
    _time.append(timestamp)

    for par in log_vars.keys():
        log_vars[par]["data"].append(data[par])


def plot_metrics(file=""):
    global log_vars, _time
    if not os.path.exists('metrics'):
        os.makedirs('metrics', exist_ok=True)

    if file:
        with open(file) as f:
            json_data = json.load(f)
            _time = json_data["time"]
            log_vars = json_data["params"]

    filename = f"{datetime.datetime.now():%Y_%m_%d_%H_%M_%S}"
    fig, ax = plt.subplots()
    time_axis = (np.array(_time) - _time[0]) / 1000
    # ax.plot(time_axis, np.array(_thrust) / 0xffff * 100, label='thrust (%)')
    # ax.plot(time_axis, np.array(_roll), label='roll')
    # ax.plot(time_axis, np.array(_m1) / 0xffff * 100, label='m1 (%)')
    # ax.plot(time_axis, np.array(_m2) / 0xffff * 100, label='m2 (%)')
    # ax.plot(time_axis, np.array(_m3) / 0xffff * 100, label='m3 (%)')
    # ax.plot(time_axis, np.array(_m4) / 0xffff * 100, label='m4 (%)')
    # ax.plot(time_axis, np.array(_x) * 100, label='x (cm)')
    # ax.plot(time_axis, np.array(_y) * 100, label='y (cm)')
    # ax.plot(time_axis, np.array(_z) * 100, label='z (cm)')
    # ax.plot(time_axis, np.array(_delta_x), label='delta x (flow/fr)')
    # ax.plot(time_axis, np.array(_delta_y), label='delta y (flow/fr)')
    for par in log_vars.keys():
        ax.plot(time_axis, np.array(log_vars[par]["data"]), label=f"{par} ({log_vars[par]['unit']})")
    ax.set_xlabel('Time (s)')
    # ax.set_ylim([-15, 105])
    plt.legend()

    if not file:
        plt.savefig(f'metrics/{filename}.png', dpi=300)
        with open(f'metrics/{filename}.json', 'w') as f:
            json.dump(dict(zip(["time", "params"], [_time, log_vars])), f)
    else:
        image_name = "".join(file.split('.')[:-1])
        plt.savefig(f'{image_name}.png', dpi=300)


def is_close(range):
    print(range)
    MIN_DISTANCE = 0.3  # m

    if range is None:
        return False
    else:
        return max(min(1.25 * (range - MIN_DISTANCE), 1), -1)


def wall_spring(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as motion_commander:
        with Multiranger(scf) as multiranger:
            keep_flying = True

            while keep_flying:
                VELOCITY = 0.5
                velocity_x = 0.0
                velocity_y = 0.0

                # print(f"front:{multiranger.front}, back:{multiranger.back}, left:{multiranger.left}, right:{multiranger.right}")

                velocity_x = is_close(multiranger.front)

                # if is_close(multiranger.left):
                #     velocity_y -= VELOCITY
                # if is_close(multiranger.right):
                #     velocity_y += VELOCITY

                if multiranger.up is not None and multiranger.up < .20:
                    keep_flying = False

                motion_commander.start_linear_motion(
                    velocity_x, velocity_y, 0)

                time.sleep(0.01)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-p", "--plot", help="path to json log file")
    args = vars(ap.parse_args())

    if not args['plot']:

        cflib.crtp.init_drivers()

        # set_controller()
        # set_pid_values()

        with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:

            cf = scf.cf

            scf.cf.param.add_update_callback(group='deck', name='bcFlow2',
                                             cb=param_deck_flow)
            time.sleep(1)

            logconf = LogConfig(name='Motor', period_in_ms=10)

            for par, conf in log_vars.items():
                logconf.add_variable(par, conf["type"])
                # logconf.add_variable('motion.deltaY', 'int16_t')

            scf.cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(log_motor_callback)

            # if not deck_attached_event.wait(timeout=5):
            #     print('No flow deck detected!')
            #     sys.exit(1)

            logconf.start()
            take_off_simple(scf)
            # wall_spring(scf)
            # test(scf)
            logconf.stop()

        plot_metrics()

    else:
        plot_metrics(file=args['plot'])
