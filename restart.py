import logging
import time
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie


logging.basicConfig(level=logging.ERROR)

URI = 'usb://0'  # or 'radio://0/80/2M'


def reboot_crazyflie(uri):
    cflib.crtp.init_drivers(enable_serial_driver=True)

    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        cf = scf.cf

        print("Connected to", uri)
        # Wait a bit for parameters to load
        time.sleep(1.0)
        print("Sending reboot command...")
        cf.param.set_value('sys.resetToFirmware', '1')
        time.sleep(0.2)
        cf.close_link()


if __name__ == '__main__':
    reboot_crazyflie(URI)
