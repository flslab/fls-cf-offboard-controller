import logging
import argparse
import cflib.crtp
from cflib.utils.power_switch import PowerSwitch

logging.basicConfig(level=logging.ERROR)


def reboot_crazyflie(uris):
    cflib.crtp.init_drivers(enable_serial_driver=True)
    for uri in uris:
        print(f"Rebooting: {uri}")
        try:
            PowerSwitch(uri).stm_power_cycle()
        except Exception as e:
            print(f"Failed to reboot {uri}: {e}")


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Reboot one or more Crazyflies.")
    parser.add_argument('uris', metavar='URI', type=str, nargs='+',
                        help='One or more URIs to reboot')
    args = parser.parse_args()
    
    reboot_crazyflie(args.uris)
