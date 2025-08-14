import logging
import time
from cflib.crazyflie import Crazyflie
import cflib.crtp

logging.basicConfig(level=logging.ERROR)

URI = 'usb://0'  # or 'radio://0/80/2M'


def reboot_crazyflie(uri):
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie()

    def param_toc_updated():
        print("Parameter TOC received, sending reboot command...")
        cf.param.set_value('sys.resetToFirmware', '1')
        time.sleep(0.5)
        cf.close_link()

    def connected(link_uri):
        print("Connected to", link_uri)
        cf.param_toc_updated.add_callback(param_toc_updated)

    def connection_failed(link_uri, msg):
        print("Connection failed:", msg)

    def disconnected(link_uri):
        print("Disconnected from", link_uri)

    cf.connected.add_callback(connected)
    cf.connection_failed.add_callback(connection_failed)
    cf.disconnected.add_callback(disconnected)

    print("Connecting to", uri)
    cf.open_link(uri)

    # Give enough time for param TOC + reboot before script exits
    time.sleep(5)


if __name__ == '__main__':
    reboot_crazyflie(URI)
