import logging
import time
from cflib.crazyflie import Crazyflie
import cflib.crtp

logging.basicConfig(level=logging.ERROR)

URI = 'usb://0'  # Replace with your Crazyflie's URI


def reboot_crazyflie(uri):
    cflib.crtp.init_drivers(enable_debug_driver=False)
    cf = Crazyflie()

    def connected(link_uri):
        print("Connected to", link_uri)
        # Send reboot command
        cf.param.set_value('sys.resetToFirmware', '1')
        print("Reboot command sent")
        # Wait a moment for command to be sent before disconnect
        time.sleep(0.5)
        cf.close_link()

    def connection_failed(link_uri, msg):
        print("Connection failed:", msg)

    def disconnected(link_uri):
        print("Disconnected from", link_uri)

    cf.connected.add_callback(connected)
    cf.connection_failed.add_callback(connection_failed)
    cf.disconnected.add_callback(disconnected)

    print("Connecting to", uri)
    cf.open_link(uri)

    time.sleep(2)  # Allow enough time for connection and reboot command


if __name__ == '__main__':
    reboot_crazyflie(URI)
