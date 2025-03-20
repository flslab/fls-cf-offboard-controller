import time
import logging
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Use USB connection
URI = uri_helper.uri_from_env(default='usb://0')

# Initialize Crazyflie drivers
logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers()

def restart_crazyflie(scf):
    """Requests a system reset via Crazyflie parameters."""
    print("Restarting Crazyflie...")
    scf.cf.param.set_value('system.reset', '1')  # Send reset command
    time.sleep(5)  # Wait for reboot
    print("Crazyflie restarted.")

# Connect and restart
if __name__ == "__main__":
    print("Connecting to Crazyflie via USB...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        restart_crazyflie(scf)