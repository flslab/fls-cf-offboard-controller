import logging
import queue
import threading
import time

from cflib.utils.power_switch import PowerSwitch

from log import LoggerFactory
from cflib.crazyflie.syncLogger import SyncLogger

from load_cell_test import sweep_thrust_with_loadcell

import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

from config import (
    DEFAULT_RADIO_URI
)


def put_latest(q: queue.Queue, item):
    try:
        q.put_nowait(item)
    except queue.Full:
        try:
            q.get_nowait()
        except queue.Empty:
            pass
        q.put_nowait(item)


def log_worker(scf, state_que, freq=10):
    """
    Log attitude/vel/acc, motor PWMs, and battery to CSV while feeding state_que.
    """
    logconf = LogConfig('FlightLog', period_in_ms=1000/freq)
    logconf.add_variable('stateEstimate.pitch', 'float')
    logconf.add_variable('stateEstimate.roll', 'float')
    # logconf.add_variable('stateEstimate.yaw', 'float')
    logconf.add_variable('motor.m1', 'uint16_t')
    logconf.add_variable('motor.m2', 'uint16_t')
    logconf.add_variable('motor.m3', 'uint16_t')
    logconf.add_variable('motor.m4', 'uint16_t')
    # logconf.add_variable('pm.vbatMV', 'uint16_t')  # use mV; convert to V

    with SyncLogger(scf, logconf) as log:

        for t_ms, data, _ in log:
            wall_t = time.time()

            pitch = float(data.get('stateEstimate.pitch', float('nan')))
            roll = float(data.get('stateEstimate.roll', float('nan')))
            yaw = float(data.get('stateEstimate.yaw', float('nan')))
            # vx = float(data.get('stateEstimate.vx', float('nan')))
            # ax = float(data.get('stateEstimate.ax', float('nan')))
            m1 = int(data.get('motor.m1', 0))
            m2 = int(data.get('motor.m2', 0))
            m3 = int(data.get('motor.m3', 0))
            m4 = int(data.get('motor.m4', 0))
            vbat_mV = int(data.get('pm.vbatMV', 0))
            vbat_V = vbat_mV / 1000.0 if vbat_mV else float('nan')

            logger.debug(",".join([
                f"{roll:.2f}",
                f"{pitch:.2f}",
                f"{yaw:.2f}",
                # f"{vx:.6f}",
                # f"{ax:.6f}",
                str(m1), str(m2), str(m3), str(m4),
                f"{vbat_V:.3f}" if vbat_mV else ""
            ]))

            # Feed controller queue
            put_latest(state_que, {
                "ts": t_ms,
                "pitch": pitch,
                "roll": roll,
                "yaw": yaw,
                # "vx": vx,
                # "ax": ax,
                "vbat": vbat_V,
                "m": (m1, m2, m3, m4),
            })

def restart(uri):
    if isinstance(uri, list) or isinstance(uri, set):
        for link in uri:
            PowerSwitch(link).stm_power_cycle()
    else:
        PowerSwitch(uri).stm_power_cycle()

if __name__ == "__main__":

    logger = LoggerFactory("Commander", level=logging.INFO).get_logger()
    URI = uri_helper.uri_from_env(default=DEFAULT_RADIO_URI)
    state_que = queue.Queue(maxsize=10)

    restart(URI)
    time.sleep(10)

    logger.info(f"Connecting to {URI}...")
    cflib.crtp.init_drivers(enable_debug_driver=False)
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf
        logger.info("Connection Established")
        cf.param.set_value('stabilizer.controller', '1')

        t_log = threading.Thread(target=log_worker, args=(scf, state_que), daemon=True)
        t_log.start()
        time.sleep(10.0)

        logger.info("Arming...")
        cf.platform.send_arming_request(True)
        time.sleep(1.0)

        logger.info("Starting thrust sweep with load-cell logging (tare before each step)")
        sweep_thrust_with_loadcell(cf, state_que, 0, 100, 10, logger)

        # thrust_percentage = 60
        # logger.info("Starting pitch sweep with load-cell logging (tare before each step)")
        # manual_sweep_steps_with_loadcell(cf, state_que, logger, thrust_percentage)

