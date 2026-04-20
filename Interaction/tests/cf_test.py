#!/usr/bin/env python3
import logging
import os
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper
from cflib.utils.power_switch import PowerSwitch

from Interaction.live_logger import LiveLogger
from log import LoggerFactory
from functools import partial

def restart(uri):
    if isinstance(uri, list) or isinstance(uri, set):
        for link in uri:
            PowerSwitch(link).stm_power_cycle()
    else:
        PowerSwitch(uri).stm_power_cycle()


def log_callback(timestamp, data, log_conf, live_logger):
    cur_time = time.time()
    data['time'] = cur_time
    live_logger.write({"type": 'state', "data": data})
def log_worker(cf, callback=log_callback):
    """
    Log attitude/vel/acc, motor PWMs, and battery to CSV while feeding state_que.
    """
    var_logger = LogConfig('FlightLog', period_in_ms=100)
    var_logger.add_variable('stateEstimate.pitch', 'float')
    var_logger.add_variable('stateEstimate.roll', 'float')
    var_logger.add_variable('stateEstimate.yaw', 'float')


    cf.log.add_config(var_logger)
    var_logger.data_received_cb.add_callback(callback)
    var_logger.start()

    return var_logger

logger = LoggerFactory("Commander", level=logging.INFO).get_logger()
def test_main():
    cflib.crtp.init_drivers(enable_debug_driver=False)

    URI = 'radio://0/100/2M/E7E7E7E706'
    live_logger = LiveLogger(os.path.join('../../logs', f"cf_test.json"))

    restart(URI)
    logger.info("Restarting")
    time.sleep(7)

    uri = uri_helper.uri_from_env(default=URI)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:

        logger.info("Connection Established")


        cf = scf.cf
        commander = cf.commander
        callback_with_params = partial(log_callback, live_logger=live_logger)
        var_logger = log_worker(cf, callback=callback_with_params)

        logger.info("Setting Up EKF")
        cf.param.set_value('stabilizer.estimator', '2')
        cf.platform.send_arming_request(True)
        time.sleep(1.0)
        dt = 0.05
        start_t = time.time()

        commander.send_setpoint(0.0, 0.0, 0.0, 0)
        while time.time() - start_t < 20:
            commander.send_setpoint(0.0, 0.0, 0.0, 30000)
            time.sleep(dt)

        logger.info("Gimbal Test Finished")
        var_logger.stop()
        live_logger.close()

if __name__ == "__main__":
    test_main()
