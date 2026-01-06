from cflib.crazyflie import Crazyflie
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie

uri = 'usb://0'
cf = Crazyflie()
scf = SyncCrazyflie(uri, cf)
battery = scf.cf.battery
print(battery.voltage)
print(battery.percentage)
