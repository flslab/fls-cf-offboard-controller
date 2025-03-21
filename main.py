import datetime
import json
import logging
import os
import subprocess
import time
from threading import Event, Thread
import argparse
import mmap
import struct

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.positioning.motion_commander import MotionCommander
from cflib.utils.multiranger import Multiranger
from cflib.utils import uri_helper
from cflib.utils.reset_estimator import reset_estimator

from led import LED
from worker_socket import WorkerSocket

#URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E713')
# URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E711')
URI = uri_helper.uri_from_env(default='usb://0') # uart pi5

DEFAULT_HEIGHT = 0.60
DURATION = 5
deck_attached_event = Event()


_time = []
log_vars = {
    "locSrv.x": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "locSrv.y": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    "locSrv.z": {
        "type": "float",
        "unit": "m",
        "data": [],
    },
    # "kalman.stateX": {
    #     "type": "float",
    #     "unit": "m",
    #     "data": [],
    # },
    # "kalman.stateY": {
    #     "type": "float",
    #     "unit": "m",
    #     "data": [],
    # },
    # "kalman.stateZ": {
    #     "type": "float",
    #     "unit": "m",
    #     "data": [],
    # },
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
    with MotionCommander(scf, default_height=DEFAULT_HEIGHT) as mc:
        time.sleep(DURATION)
        # mc.up(0.25)
        # time.sleep(5)
        mc.stop()


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


def send_extpose_quat(cf, x, y, z, quat=None, send_full_pose=False):
    """
    Send the current Crazyflie X, Y, Z position and attitude as a quaternion.
    This is going to be forwarded to the Crazyflie's position estimator.
    """
    if send_full_pose:
        cf.extpos.send_extpose(x, y, z, quat.x, quat.y, quat.z, quat.w)
    else:
        cf.extpos.send_extpos(x, y, z)


def blender_animation(scf, frame_interval=1/24, led_on=False):
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
        for i in range(1, len(animation_data)+1):
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


def set_pid_values(scf, propeller_size=3):
    # with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
    cf = scf.cf


    cf.param.set_value('quadSysId.armLength', '0.05656')

    if propeller_size == 2:
        cf.param.set_value('pid_rate.roll_kp', '75')
        cf.param.set_value('pid_rate.roll_ki', '75')
        cf.param.set_value('pid_rate.roll_kd', '1')
        cf.param.set_value('pid_rate.pitch_kp', '75')
        cf.param.set_value('pid_rate.pitch_ki', '75')
        cf.param.set_value('pid_rate.pitch_kd', '1')

        cf.param.set_value('velCtlPid.vxKp', '15')
        cf.param.set_value('velCtlPid.vxKi', '1')
        cf.param.set_value('velCtlPid.vxKd', '0')
        cf.param.set_value('velCtlPid.vyKp', '15')
        cf.param.set_value('velCtlPid.vyKi', '1')
        cf.param.set_value('velCtlPid.vyKd', '0')
        cf.param.set_value('velCtlPid.vzKp', '15')
        cf.param.set_value('velCtlPid.vzKi', '1')
        cf.param.set_value('velCtlPid.vzKd', '0')

        cf.param.set_value('pid_attitude.roll_kp', '3')
        cf.param.set_value('pid_attitude.roll_ki', '0.5')
        cf.param.set_value('pid_attitude.roll_kd', '0')
        cf.param.set_value('pid_attitude.pitch_kp', '3')
        cf.param.set_value('pid_attitude.pitch_ki', '0.5')
        cf.param.set_value('pid_attitude.pitch_kd', '0')

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


def set_controller():
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        # Controller type Auto select(0), PID(1), Mellinger(2), INDI(3), Brescianini(4)(Default: 0)

        print('stabilizer.controller', cf.param.get_value('stabilizer.controller'))
        cf.param.set_value('stabilizer.controller', 1)
        print('stabilizer.controller', cf.param.get_value('stabilizer.controller'))


def log_callback(timestamp, data, logconf):
    # print(timestamp, data)
    _time.append(timestamp)

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


class LocalizationWrapper(Thread):
    def __init__(self, cf):
        super().__init__()
        self.cf = cf
        self.stopped = False
        # Size of the Position structure: 1 byte for 'valid' + 7 floats (4 bytes each)
        self.position_size = 1 + 7 * 4  # 1 byte + 7 floats (4 bytes each)
        self.shm_name = "/pos_shared_mem"
        self.shm_fd = open(f"/dev/shm{self.shm_name}", "r+b")  # Open shared memory
        self.shm_map = mmap.mmap(self.shm_fd.fileno(), self.position_size, access=mmap.ACCESS_READ)

    def run(self):
        while not self.stopped:
            data = self.shm_map[:self.position_size] # Read 8 bytes (bool + 7 floats = 1 byte + 28 bytes)
            # print("raw data:", data)
            valid = struct.unpack("<?", data[:1])[0]  # Extract the validity flag (1 byte)

            if valid:
                x, y, z, qx, qy, qz, qw = struct.unpack("<7f", data[1:])
                print(f"Position: ({x}, {y}, {z}), Orientation: ({qx}, {qy}, {qz}, {qw})")
                send_extpose_quat(self.cf, x, y, z)
            else:
                print("Invalid data received")

            time.sleep(0.01)

    def stop(self):
        self.stopped = True


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("--led", help="Turn LEDs on", action="store_true", default=False)
    ap.add_argument("--log", help="Enable logging", action="store_true", default=False)
    ap.add_argument("--localize", help="Enable onboard marker localization", action="store_true", default=False)
    ap.add_argument("--log-dir", help="Log variables to the given directory", type=str, default="./logs")
    ap.add_argument("-v", "--verbose", help="Print logs if logging is enabled", action="store_true", default=False)
    args = ap.parse_args()

    if args.verbose:
        logging.basicConfig(level=logging.INFO)
    else:
        logging.basicConfig(level=logging.ERROR)

    # Initialize the low-level drivers including the serial driver
    # cflib.crtp.init_drivers()

    cflib.crtp.init_drivers(enable_serial_driver=True)

    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        if args.localize:
            # c_process = subprocess.Popen(["/home/fls/fls-marker-localization/build/eye", "-t", "30"])
            # time.sleep(1)
            localization = LocalizationWrapper(scf.cf)
            localization.start()

        cf = scf.cf

        scf.cf.param.add_update_callback(group='deck', name='bcFlow2', cb=param_deck_flow)
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

        set_pid_values(scf, propeller_size=3)
        reset_estimator(scf.cf)

        if args.log:
            logconf.start()

        blender_animation(scf, frame_interval=1/24, led_on=args.led)
        # time.sleep(10)

        if args.log:
            logconf.stop()

    if args.log:
        save_logs(log_dir=args.log_dir)

    if args.localize:
        localization.stop()
        localization.join()
