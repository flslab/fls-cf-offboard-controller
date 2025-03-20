import time
import logging
import cflib
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.utils import uri_helper

# Use USB connection
URI = uri_helper.uri_from_env(default='usb://0')

# Initialize the Crazyflie drivers
logging.basicConfig(level=logging.ERROR)
cflib.crtp.init_drivers()

def restart_crazyflie(scf):
    """Sends a system reset command to reboot the Crazyflie."""
    print("Restarting Crazyflie...")
    scf.cf.system.resetsystem()
    time.sleep(5)  # Wait a few seconds for reboot
    print("Crazyflie restarted.")

# Connect and restart
if __name__ == "__main__":
    print("Connecting to Crazyflie via USB...")
    with SyncCrazyflie(URI, cf=Crazyflie(rw_cache="./cache")) as scf:
        restart_crazyflie(scf)