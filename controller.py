import datetime
import json
import logging
import os
import subprocess
import threading
import time
from threading import Event, Thread
import argparse
import mmap
import struct
import numpy as np
import motioncapture

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.positioning.position_hl_commander import PositionHlCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

from worker_socket import WorkerSocket

# URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E713')
# URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E711')
URI = uri_helper.uri_from_env(default='usb://0')  # uart pi5

DEFAULT_HEIGHT = 0.60
DURATION = 10
deck_attached_event = Event()

# The host name or ip address of the mocap system
host_name = '192.168.1.39'

# The type of the mocap system
# Valid options are: 'vicon', 'optitrack', 'optitrack_closed_source', 'qualisys', 'nokov', 'vrpn', 'motionanalysis'
mocap_system_type = 'vicon'

# The name of the rigid body that represents the Crazyflie
rigid_body_name = 'cffls'

# True: send position and orientation; False: send position only
send_full_pose = False

# When using full pose, the estimator can be sensitive to noise in the orientation data when yaw is close to +/- 90
# degrees. If this is a problem, increase orientation_std_dev a bit. The default value in the firmware is 4.5e-3.
orientation_std_dev = 8.0e-3

# The trajectory to fly
# See https://github.com/whoenig/uav_trajectories for a tool to generate
# trajectories

pos_update_time_log = []
pos_update_profile_log = []

failsafe = False
z_estimate = 0
z_filter_alpha = 0.3

_time = []
log_vars = {
    # "locSrv.x": {
    #     "type": "float",
    #     "unit": "m",
    #     "data": [],
    # },
    # "locSrv.y": {
    #     "type": "float",
    #     "unit": "m",
    #     "data": [],
    # },
    # "locSrv.z": {
    #     "type": "float",
    #     "unit": "m",
    #     "data": [],
    # },
    "kalman.stateX": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "kalman.stateY": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "kalman.stateZ": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    # "motor.m1": {
    #     "type": "uint16_t",
    #     "unit": "cmd",
    #     "data": [],
    # },
    # "motor.m2": {
    #     "type": "uint16_t",
    #     "unit": "cmd",
    #     "data": [],
    # },
    # "motor.m3": {
    #     "type": "uint16_t",
    #     "unit": "cmd",
    #     "data": [],
    # },
    # "motor.m4": {
    #     "type": "uint16_t",
    #     "unit": "cmd",
    #     "data": [],
    # },
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
    global failsafe
    with PositionHlCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        # time.sleep(DURATION)
        start_time = time.time()
        while time.time() - start_time < DURATION:
            if failsafe:
                break
            #     cf.commander.send_position_setpoint(0, 0, DEFAULT_HEIGHT, 0)
            #     # cf.commander.send_hover_setpoint(0, 0, 0, position[2])
            #     # cf.commander.send_zdistance_setpoint(0, 0, 0, position[2])
            time.sleep(1)


def trajectory(scf, trajectory):
    Z = args.takeoff_altitude

    with PositionHlCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(3)
        WAYPOINTS, fps = create_trajectory_from_file(trajectory, Z)
        cf = scf.cf

        trajectory_log = []
        for i, p in enumerate(WAYPOINTS):
            # cf.commander.send_position_setpoint(*p, yaw=0.0)
            mc.go_to(*p)

            time.sleep(1 / (fps))
        # start_time = time.time()
        # while time.time() - start_time < DURATION:
        #     cf.commander.send_position_setpoint(0, 0, DEFAULT_HEIGHT, 0)
        #     # cf.commander.send_hover_setpoint(0, 0, 0, position[2])
        #     # cf.commander.send_zdistance_setpoint(0, 0, 0, position[2])
        #     time.sleep(0.1)


def up_and_down(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        mc.down(0.3)
        # time.sleep(DURATION)
        mc.up(0.3)
        mc.down(0.3)
        # time.sleep(DURATION)
        mc.up(0.3)
        mc.down(0.6)
        # time.sleep(5)
        mc.stop()


def take_off(cf, position):
    take_off_time = 1.0
    sleep_time = 0.1
    steps = int(take_off_time / sleep_time)
    vz = position[2] / take_off_time

    print(f'take off at {position[2]}')

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, vz, 0)
        time.sleep(sleep_time)


def land(cf, position):
    landing_time = 1.0
    sleep_time = 0.1
    steps = int(landing_time / sleep_time)
    vz = position[2] / landing_time

    print(f'land from {position[2]}')

    for i in range(steps):
        cf.commander.send_velocity_world_setpoint(0, 0, -vz, 0)
        time.sleep(sleep_time)


def send_extpose_quat(cf, x, y, z, quat=None, send_full_pose=False, filter_z=False):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    global z_estimate, z_filter_alpha
    if filter_z:
        z_estimate = (1 - z_filter_alpha) * z_estimate + z_filter_alpha * z
    else:
        z_estimate = z
    start_time = time.time()
    if send_full_pose:
        cf.extpos.send_extpose(x, y, z_estimate, quat.x, quat.y, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(x, y, z_estimate)
        # print(f"sending {x, y, z_estimate}")
    end_time = time.time()
    pos_update_time_log.append(end_time)
    pos_update_profile_log.append(end_time - start_time)


def blender_animation(scf, frame_interval=1 / 24, led_on=False):
    if led_on:
        led = LED()
    yaw = 0
    with open("animation_data.json", "r") as f:
        animation_data = json.load(f)

    cf = scf.cf

    # Arm the Crazyflie
    cf.platform.send_arming_request(True)
    time.sleep(1.0)

    take_off(cf, animation_data['1']['pos'])
    # time.sleep(1.0)

    for i in range(2):
        for i in range(1, len(animation_data) + 1):
            start_time = time.time()
            position = animation_data[str(i)]['pos']
            if led_on:
                led.set_frame(animation_data[str(i)]['led'])
            # print('Setting position {}'.format(position))
            while time.time() - start_time < frame_interval:
                # cf.commander.send_position_setpoint(position[0],
                #                                     position[1],
                #                                     position[2],
                #                                     0)
                cf.commander.send_hover_setpoint(0, 0, 0, position[2])
                # cf.commander.send_zdistance_setpoint(0, 0, 0, position[2])
                time.sleep(0.01)

    print("Landing...")
    land(cf, animation_data['120']['pos'])

    cf.commander.send_stop_setpoint()
    # Hand control over to the high level commander to avoid timeout and locking of the Crazyflie
    cf.commander.send_notify_setpoint_stop()

    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)
    if led_on:
        led.clear()

    # with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:


def take_off_simple_network(scf):
    sock = WorkerSocket()
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(DURATION)
        start_time = time.time()
        while time.time() - start_time < DURATION:
            msg = sock.receive()
            print(msg)
        # mc.up(0.25)
        # time.sleep(5)
        mc.stop()


def move_circle(scf):
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        mc.circle_right(.4)


# def test(scf):
#     with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
#         time.sleep(2)
#         mc.forward(.5)
#         time.sleep(2)


def set_pid_values(scf, propeller_size=None, with_cage=False):
    # with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf

    cf.param.set_value('quadSysId.armLength', '0.04242')

    if propeller_size == 2:
        cf.param.set_value('posCtlPid.xKp', '1.8')
        cf.param.set_value('posCtlPid.xKi', '0.09')
        cf.param.set_value('posCtlPid.xKd', '0.0')
        cf.param.set_value('posCtlPid.yKp', '2.0')
        cf.param.set_value('posCtlPid.yKi', '0.1')
        cf.param.set_value('posCtlPid.yKd', '0.0')
        cf.param.set_value('posCtlPid.zKp', '2.0')
        cf.param.set_value('posCtlPid.zKi', '0.15')
        cf.param.set_value('posCtlPid.zKd', '0.15')
        cf.param.set_value('posCtlPid.thrustMin', '10000')
        cf.param.set_value('posCtlPid.thrustBase', '22000')

        cf.param.set_value('velCtlPid.vxKp', '27.0')
        cf.param.set_value('velCtlPid.vxKi', '18.0')
        cf.param.set_value('velCtlPid.vxKd', '0.0')
        cf.param.set_value('velCtlPid.vyKp', '30.0')
        cf.param.set_value('velCtlPid.vyKi', '20.0')
        cf.param.set_value('velCtlPid.vyKd', '0')
        cf.param.set_value('velCtlPid.vzKp', '30.0')
        cf.param.set_value('velCtlPid.vzKi', '2.0')
        cf.param.set_value('velCtlPid.vzKd', '0.5')

        cf.param.set_value('pid_attitude.roll_kp', '19.0')
        cf.param.set_value('pid_attitude.roll_ki', '0.001')
        cf.param.set_value('pid_attitude.roll_kd', '0.15')
        cf.param.set_value('pid_attitude.pitch_kp', '17.1')
        cf.param.set_value('pid_attitude.pitch_ki', '0.001')
        cf.param.set_value('pid_attitude.pitch_kd', '0.135')

        cf.param.set_value('pid_rate.roll_kp', '66.0')
        cf.param.set_value('pid_rate.roll_ki', '0.0')
        cf.param.set_value('pid_rate.roll_kd', '1.65')
        cf.param.set_value('pid_rate.pitch_kp', '65.0')
        cf.param.set_value('pid_rate.pitch_ki', '0.0')
        cf.param.set_value('pid_rate.pitch_kd', '1.485')

    elif propeller_size == 3:
        if with_cage:
            cf.param.set_value('pid_rate.roll_kp', '70')
            cf.param.set_value('pid_rate.roll_ki', '70')
            cf.param.set_value('pid_rate.roll_kd', '1')
            cf.param.set_value('pid_rate.pitch_kp', '70')
            cf.param.set_value('pid_rate.pitch_ki', '70')
            cf.param.set_value('pid_rate.pitch_kd', '1')

            cf.param.set_value('pid_attitude.roll_kp', '2.5')
            cf.param.set_value('pid_attitude.roll_ki', '0.25')
            cf.param.set_value('pid_attitude.roll_kd', '0')
            cf.param.set_value('pid_attitude.pitch_kp', '2.5')
            cf.param.set_value('pid_attitude.pitch_ki', '0.25')
            cf.param.set_value('pid_attitude.pitch_kd', '0')

            cf.param.set_value('velCtlPid.vxKp', '12')
            cf.param.set_value('velCtlPid.vxKi', '1')
            cf.param.set_value('velCtlPid.vxKd', '0')
            cf.param.set_value('velCtlPid.vyKp', '12')
            cf.param.set_value('velCtlPid.vyKi', '1')
            cf.param.set_value('velCtlPid.vyKd', '0')
            cf.param.set_value('velCtlPid.vzKp', '12')
            cf.param.set_value('velCtlPid.vzKi', '1')
            cf.param.set_value('velCtlPid.vzKd', '0')

            cf.param.set_value('posCtlPid.xKp', '2')
            cf.param.set_value('posCtlPid.yKp', '2')
            cf.param.set_value('posCtlPid.zKp', '2')
            cf.param.set_value('posCtlPid.zKi', '0.5')
            cf.param.set_value('posCtlPid.thrustMin', '10000')
            cf.param.set_value('posCtlPid.thrustBase', '22000')
        else:
            cf.param.set_value('pid_rate.roll_kp', '70')
            cf.param.set_value('pid_rate.roll_ki', '70')
            cf.param.set_value('pid_rate.roll_kd', '1')
            cf.param.set_value('pid_rate.pitch_kp', '70')
            cf.param.set_value('pid_rate.pitch_ki', '70')
            cf.param.set_value('pid_rate.pitch_kd', '1')

            cf.param.set_value('pid_attitude.roll_kp', '2.5')
            cf.param.set_value('pid_attitude.roll_ki', '0.25')
            cf.param.set_value('pid_attitude.roll_kd', '0')
            cf.param.set_value('pid_attitude.pitch_kp', '2.5')
            cf.param.set_value('pid_attitude.pitch_ki', '0.25')
            cf.param.set_value('pid_attitude.pitch_kd', '0')

            cf.param.set_value('velCtlPid.vxKp', '10')
            cf.param.set_value('velCtlPid.vxKi', '1')
            cf.param.set_value('velCtlPid.vxKd', '0')
            cf.param.set_value('velCtlPid.vyKp', '10')
            cf.param.set_value('velCtlPid.vyKi', '1')
            cf.param.set_value('velCtlPid.vyKd', '0')
            cf.param.set_value('velCtlPid.vzKp', '10')
            cf.param.set_value('velCtlPid.vzKi', '1')
            cf.param.set_value('velCtlPid.vzKd', '0')

            cf.param.set_value('posCtlPid.xKp', '2')
            cf.param.set_value('posCtlPid.yKp', '2')
            cf.param.set_value('posCtlPid.zKp', '2')
            cf.param.set_value('posCtlPid.zKi', '0.5')
            cf.param.set_value('posCtlPid.thrustMin', '10000')
            cf.param.set_value('posCtlPid.thrustBase', '22000')

    time.sleep(1)

    # print(cf.param.get_value('posCtlPid.zKp'))
    print('pid_attitude.roll_kp', cf.param.get_value('pid_attitude.roll_kp'))
    print('pid_attitude.roll_ki', cf.param.get_value('pid_attitude.roll_ki'))
    print('pid_attitude.roll_kd', cf.param.get_value('pid_attitude.roll_kd'))
    print('pid_attitude.pitch_kp', cf.param.get_value('pid_attitude.pitch_kp'))
    print('pid_attitude.pitch_ki', cf.param.get_value('pid_attitude.pitch_ki'))
    print('pid_attitude.pitch_kd', cf.param.get_value('pid_attitude.pitch_kd'))

    print('pid_rate.roll_kp', cf.param.get_value('pid_rate.roll_kp'))
    print('pid_rate.roll_ki', cf.param.get_value('pid_rate.roll_ki'))
    print('pid_rate.roll_kd', cf.param.get_value('pid_rate.roll_kd'))
    print('pid_rate.pitch_kp', cf.param.get_value('pid_rate.pitch_kp'))
    print('pid_rate.pitch_ki', cf.param.get_value('pid_rate.pitch_ki'))
    print('pid_rate.pitch_kd', cf.param.get_value('pid_rate.pitch_kd'))

    print('posCtlPid.xKp', cf.param.get_value('posCtlPid.xKp'))
    print('posCtlPid.xKi', cf.param.get_value('posCtlPid.xKi'))
    print('posCtlPid.xKd', cf.param.get_value('posCtlPid.xKd'))
    print('posCtlPid.yKp', cf.param.get_value('posCtlPid.yKp'))
    print('posCtlPid.yKi', cf.param.get_value('posCtlPid.yKi'))
    print('posCtlPid.yKd', cf.param.get_value('posCtlPid.yKd'))
    print('posCtlPid.zKp', cf.param.get_value('posCtlPid.zKp'))
    print('posCtlPid.zKi', cf.param.get_value('posCtlPid.zKi'))
    print('posCtlPid.zKd', cf.param.get_value('posCtlPid.zKd'))

    print('velCtlPid.vxKp', cf.param.get_value('velCtlPid.vxKp'))
    print('velCtlPid.vxKi', cf.param.get_value('velCtlPid.vxKi'))
    print('velCtlPid.vxKd', cf.param.get_value('velCtlPid.vxKd'))
    print('velCtlPid.vyKp', cf.param.get_value('velCtlPid.vyKp'))
    print('velCtlPid.vyKi', cf.param.get_value('velCtlPid.vyKi'))
    print('velCtlPid.vyKd', cf.param.get_value('velCtlPid.vyKd'))
    print('velCtlPid.vzKp', cf.param.get_value('velCtlPid.vzKp'))
    print('velCtlPid.vzKi', cf.param.get_value('velCtlPid.vzKi'))
    print('velCtlPid.vzKd', cf.param.get_value('velCtlPid.vzKd'))

    print('posCtlPid.thrustMin', cf.param.get_value('posCtlPid.thrustMin'))
    print('quadSysId.armLength', cf.param.get_value('quadSysId.armLength'))
    # print(cf.param.get_value('posCtlPid.zKd'))
    # cf.param.set_value('posCtlPid.zKp', str(new_P_gain))
    # cf.param.set_value('posCtlPid.zKi', str(new_I_gain))
    # cf.param.set_value('posCtlPid.zKd', str(new_D_gain))
    # cf.param.set_value('pid_attitude.roll_kp', str(9))
    # cf.param.set_value('pid_attitude.pitch_kp', str(9))
    # time.sleep(2)
    # print('pid_attitude.roll_kd', cf.param.get_value('pid_attitude.roll_kd'))


def set_controller(scf):
    cf = scf.cf

    # Controller type Auto select(0), PID(1), Mellinger(2), INDI(3), Brescianini(4)(Default: 0)

    print('stabilizer.controller', cf.param.get_value('stabilizer.controller'))
    print('stabilizer.estimator', cf.param.get_value('stabilizer.estimator'))
    print('commander.enHighLevel', cf.param.get_value('commander.enHighLevel'))
    print('locSrv.extPosStdDev', cf.param.get_value('locSrv.extPosStdDev'))
    print('locSrv.extQuatStdDev', cf.param.get_value('locSrv.extQuatStdDev'))
    cf.param.set_value('commander.enHighLevel', 1)
    cf.param.set_value('stabilizer.estimator', 2)
    cf.param.set_value('stabilizer.controller', 1)
    cf.param.set_value('locSrv.extPosStdDev', 0.001)
    cf.param.set_value('locSrv.extQuatStdDev', 0.05)
    cf.param.set_value('kalman.resetEstimation', 1)
    time.sleep(1)
    print('stabilizer.controller', cf.param.get_value('stabilizer.controller'))
    print('stabilizer.estimator', cf.param.get_value('stabilizer.estimator'))
    print('commander.enHighLevel', cf.param.get_value('commander.enHighLevel'))
    print('locSrv.extPosStdDev', cf.param.get_value('locSrv.extPosStdDev'))
    print('locSrv.extQuatStdDev', cf.param.get_value('locSrv.extQuatStdDev'))


def log_callback(timestamp, data, logconf):
    # print(timestamp, data)
    _time.append(time.time())

    for par in log_vars.keys():
        log_vars[par]["data"].append(data[par])

        if args.verbose:
            logging.info(f"{par} = {data[par]}")


def save_logs(log_dir='logs'):
    global log_vars, _time
    if not os.path.exists(log_dir):
        os.makedirs(log_dir, exist_ok=True)

    filename = f"{datetime.datetime.now():%Y_%m_%d_%H_%M_%S}"

    with open(f'{log_dir}/{filename}.json', 'w') as f:
        json.dump(dict(zip(["time", "params"], [_time, log_vars])), f)

    # fig, ax = plt.subplots()
    # time_axis = (np.array(_time) - _time[0]) / 1000

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
    # for par in log_vars.keys():
    #     ax.plot(time_axis, np.array(log_vars[par]["data"]), label=f"{par} ({log_vars[par]['unit']})")
    # ax.set_xlabel('Time (s)')
    # ax.set_ylim([-15, 105])
    # plt.legend()

    # plt.savefig(f'{log_dir}/{filename}.png', dpi=300)

    # else:
    # image_name = "".join(file.split('.')[:-1])
    # plt.savefig(f'{image_name}.png', dpi=300)


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


def send_vicon_position(cf):
    def func(x, y, z, timestamp):
        send_extpose_quat(cf, x / 1000, y / 1000, z / 1000)

    return func


def consume_vicon_data(cf, stop_event):
    """Run in separate thread: process new Vicon data as soon as it arrives."""
    last_version = 0
    while not stop_event.is_set():
        data, last_version = vicon_thread.wait_for_new(last_version)
        if data:
            x, y, z, timestamp = data
            send_extpose_quat(cf, x / 1000, y / 1000, z / 1000)


def create_trajectory_from_file(file_path, takeoff_altitude):
    waypoints = []
    with open(file_path, "r") as f:
        trajectory = json.load(f)

    fps = trajectory["fps"]
    start_position = trajectory["start_position"]
    segments = trajectory["segments"]

    #  go to start position
    x0, y0, z0 = 0, 0, takeoff_altitude
    x, y, z = start_position
    z = takeoff_altitude + z
    dx, dy, dz = x - x0, y - y0, z - z0

    # move to start point in 3 seconds
    N = 3 * fps
    for i in range(1, N + 1):
        waypoints.append([x0 + i * dx / N, y0 + i * dy / N, z0 + i * dz / N])

    # stay at start point for 1 second
    for i in range(fps):
        waypoints.append([x, y, z])

    for i in range(2):
        for segment in segments:
            positions = segment["position"]
            velocities = segment["velocity"]
            state = segment["state"]
            if state == "RETURN" and i == 2:
                break

            for p, v in zip(positions, velocities):
                x, y, z = p
                # vx, vy, vz = v
                # self.send_position_target(x, y, -self.takeoff_altitude-z)
                # self.send_position_velocity_target(x, y, -self.takeoff_altitude - z, vx, vy, -vz)
                waypoints.append([x, y, takeoff_altitude + z])

    #  go to start position
    last_p = waypoints[-1]
    for i in range(fps):
        waypoints.append([last_p[0] - i * last_p[0] / fps, last_p[1] - i * last_p[1] / fps,
                          last_p[2] - i * (last_p[2] - takeoff_altitude) / fps])

    return np.array(waypoints), fps


class LocalizationWrapper(Thread):
    def __init__(self, cf):
        super().__init__()
        self.cf = cf
        self.stopped = False
        self.position_size = 1 + 3 + 6 * 4  # 1 byte + 3 byte padding + 6 floats (4 bytes each)
        self.shm_name = "/pos_shared_mem"
        self.shm_fd = open(f"/dev/shm{self.shm_name}", "r+b")  # Open shared memory
        self.shm_map = mmap.mmap(self.shm_fd.fileno(), self.position_size, access=mmap.ACCESS_READ)

    def run(self):
        global failsafe
        last_valid = time.time()
        while not self.stopped:
            data = self.shm_map[:self.position_size]  # Read 8 bytes (bool + 7 floats = 1 byte + 28 bytes)
            # print("raw data:", data)
            valid = struct.unpack("<4?", data[:4])[0]  # Extract the validity flag (1 byte)

            if time.time() - last_valid > 1 and failsafe is False:
                failsafe = True
                print(f"Failsafe triggered due to lack of position estimation.")
            if valid:
                last_valid = time.time()
                left, forward, up, roll, pitch, yaw = struct.unpack("<6f", data[4:28])
                # print(f"Position: ({forward:.3f}, {left:.3f}, {up:.3f})")
                send_extpose_quat(self.cf, forward, left, up, filter_z=True)
            else:
                pass

            time.sleep(1 / 120)

    def stop(self):
        self.stopped = True


class MocapWrapper(Thread):
    def __init__(self, body_name):
        Thread.__init__(self)

        self.body_name = body_name
        self.on_pose = None
        self._stay_open = True
        self.all_frames = []

        self.start()

    def close(self):
        self._stay_open = False

        now = datetime.datetime.now()
        formatted = now.strftime("%H_%M_%S_%m_%d_%Y")
        file_path = os.path.join("logs", f"vicon_{formatted}.json")
        with open(file_path, "w") as f:
            json.dump({"frames": self.all_frames}, f)
        print(f"Vicon log saved in {file_path}")

    def run(self):
        mc = motioncapture.connect(mocap_system_type, {'hostname': host_name})
        i = 0
        while self._stay_open:
            mc.waitForNextFrame()
            for name, obj in mc.rigidBodies.items():
                if name == self.body_name:
                    if self.on_pose:
                        pos = obj.position
                        self.on_pose([pos[0], pos[1], pos[2], obj.rotation])
                        self.all_frames.append({
                            "frame_id": i,
                            "tvec": [float(pos[0]), float(pos[1]), float(pos[2])],
                            "time": time.time() * 1000
                        })
                        i += 1


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("--takeoff-altitude", help="takeoff altitude", default=1.0, type=float)
    ap.add_argument("-t", help="flight duration", default=10.0, type=float)
    ap.add_argument("--fps", type=int, default=120, help="position estimation rate, works with --localize")
    ap.add_argument("--led", help="Turn LEDs on", action="store_true", default=False)
    ap.add_argument("--log", help="Enable logging", action="store_true", default=False)
    ap.add_argument("--localize", help="Enable onboard marker localization", action="store_true", default=False)
    ap.add_argument("--vicon", action="store_true", help="localize using Vicon and save tracking data")
    ap.add_argument("--log-dir", help="Log variables to the given directory", type=str, default="./logs")
    ap.add_argument("-v", "--verbose", help="Print logs if logging is enabled", action="store_true", default=False)
    ap.add_argument("--trajectory", type=str, help="path to trajectory file to follow")
    ap.add_argument("--simple-takeoff", action="store_true", help="takeoff and land")
    ap.add_argument("--save-camera", action="store_true",
                    help="save camera at 1/10 of original fps, works with --localize")
    ap.add_argument("--stream-camera", action="store_true",
                    help="stream camera at 1/10 of original fps, works with --localize")

    args = ap.parse_args()

    DEFAULT_HEIGHT = args.takeoff_altitude
    DURATION = args.t

    log_level = logging.ERROR
    if args.verbose:
        log_level = logging.INFO

    if args.led:
        from led import LED

    # Initialize the low-level drivers including the serial driver
    # cflib.crtp.init_drivers()

    cflib.crtp.init_drivers(enable_serial_driver=True)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        if args.localize:
            localization_params = [
                "/home/fls/fls-marker-localization/build/eye",
                "-t", str(20 + args.t),
                "--config", "/home/fls/fls-marker-localization/build/camera_config.json",
                "--brightness", "0.5",  # 0.5
                "--contrast", "2.5",  # 0.75
                "--exposure", "500",
                "--fps", str(args.fps),
            ]

            if args.save_camera:
                localization_params.extend(["-s", "--save-rate", "10"])
            if args.stream_camera:
                localization_params.extend(["--stream", "--stream-rate", "10"])

            c_process = subprocess.Popen(localization_params)

            time.sleep(2)
            localization = LocalizationWrapper(cf)
            localization.start()

        if args.vicon:
            mocap_wrapper = MocapWrapper(rigid_body_name)
            mocap_wrapper.on_pose = lambda pose: send_extpose_quat(cf, pose[0], pose[1], pose[2], pose[3])

            # from vicon import ViconWrapper
            #
            # vicon_thread = ViconWrapper(log_level=log_level)
            # vicon_thread.start()
            # stop_event = threading.Event()
            # vicon_consumer_thread = threading.Thread(
            #     target=consume_vicon_data, args=(cf, stop_event), daemon=True
            # )
            # vicon_consumer_thread.start()

        # scf.cf.param.add_update_callback(group='deck', name='bcZRanger2', cb=param_deck_flow)
        time.sleep(.5)

        if args.log:
            logconf = LogConfig(name='Motor', period_in_ms=10)

            for par, conf in log_vars.items():
                logconf.add_variable(par, conf["type"])

            scf.cf.log.add_config(logconf)
            logconf.data_received_cb.add_callback(log_callback)

        # if not deck_attached_event.wait(timeout=5):
        #     print('No flow deck detected!')
        #     sys.exit(1)

        set_controller(scf)
        set_pid_values(scf, propeller_size=2)
        reset_estimator(scf.cf)
        time.sleep(2.0)

        if args.log:
            logconf.start()

        # blender_animation(scf, frame_interval=1/24, led_on=args.led)
        cf.platform.send_arming_request(True)
        time.sleep(1.0)

        if args.simple_takeoff:
            take_off_simple(scf)
        elif args.trajectory is not None:
            trajectory(scf, args.trajectory)
        else:
            time.sleep(args.t)
        # time.sleep(10)

        if args.vicon:
            mocap_wrapper.close()
            # stop_event.set()
            # vicon_consumer_thread.join()
            # vicon_thread.stop()
            # vicon_thread.join()

        if args.localize:
            localization.stop()
            localization.join()

        if args.log:
            logconf.stop()

    if args.log:
        save_logs(log_dir=args.log_dir)

    with open("pose_update_time.txt", "w") as f:
        for number in pos_update_time_log:
            f.write(f"{number}\n")

    with open("pose_update_profile.txt", "w") as f:
        for number in pos_update_profile_log:
            f.write(f"{number}\n")
