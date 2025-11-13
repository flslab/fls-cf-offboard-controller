import logging
import time
import cflib.crtp
from cflib.utils.power_switch import PowerSwitch

logging.basicConfig(level=logging.ERROR)

# URI = 'radio://0/80/2M/E7E7E7E712'  # or 'radio://0/80/2M'
URI = 'radio://0/80/2M/E7E7E7E701'  # or 'radio://0/80/2M'


def reboot_crazyflie(uri):
    cflib.crtp.init_drivers(enable_serial_driver=True)
    PowerSwitch(uri).stm_power_cycle()


if __name__ == '__main__':
    reboot_crazyflie(URI)
