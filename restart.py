from cflib.utils.power_switch import PowerSwitch
from cflib.utils import uri_helper

URI = uri_helper.uri_from_env(default='radio://0/80/2M/E7E7E7E711')

PowerSwitch(URI).stm_power_cycle()
