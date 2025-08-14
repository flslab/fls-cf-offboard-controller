import logging
import time
from cflib.crazyflie import Crazyflie
import cflib.crtp

logging.basicConfig(level=logging.ERROR)

URI = 'usb://0'  # Change to 'radio://0/80/2M' for radio link


class Rebooter:
    def __init__(self, uri):
        self.cf = Crazyflie()
        self.uri = uri

        self.cf.connected.add_callback(self.connected)
        self.cf.connection_failed.add_callback(self.connection_failed)
        self.cf.disconnected.add_callback(self.disconnected)

        cflib.crtp.init_drivers(enable_debug_driver=False)
        print(f"Connecting to {uri}")
        self.cf.open_link(uri)

    def connected(self, link_uri):
        print(f"Connected to {link_uri}")
        # Wait for parameter TOC to be downloaded
        self.cf.param.add_update_callback(group='sys', name='uptime', cb=self.on_params_ready)

    def on_params_ready(self, name, value):
        # Called once we know the parameter system is alive
        print("Parameter system ready, sending reboot command...")
        self.cf.param.set_value('sys.resetToFirmware', '1')
        time.sleep(0.5)
        self.cf.close_link()

    def connection_failed(self, link_uri, msg):
        print(f"Connection failed to {link_uri}: {msg}")

    def disconnected(self, link_uri):
        print(f"Disconnected from {link_uri}")


if __name__ == '__main__':
    Rebooter(URI)
    time.sleep(5)  # Allow time for connection/reboot before script exits
