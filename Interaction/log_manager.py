import copy
import json
import os
import re
import subprocess

from Interaction.Kalman_Filter import VelocityKalmanFilter
from Interaction.live_logger import LiveLogger
from log_manager_abs import LogManager
from cflib.crazyflie.log import LogConfig
import time
import logging

logger = logging.getLogger(__name__)


class InteractionLogger(LogManager):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cf_var_logger = None
        self.cf_log_times = []
        self.cf_log_data = None
        self.args = kwargs.get('controller_args', False)
        self.verbose = self.args.verbose

        dt = 1 / self.args.fps
        self.kf = {'x': VelocityKalmanFilter(dt=dt, process_noise=1.0, measurement_noise=0.001 ** 2),
                   'y': VelocityKalmanFilter(dt=dt, process_noise=1.0, measurement_noise=0.001 ** 2),
                   'z': VelocityKalmanFilter(dt=dt, process_noise=1.0, measurement_noise=0.001 ** 2)}

        log_dir = self.args.log_dir
        if not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)
        self.live_logger = LiveLogger(os.path.join(log_dir, f"{self.args.tag}.json"))
        self.get_git_version()

        # self.extra_markers = {}
        # for marker in self.args.extra_marker:
        #     self.extra_markers[marker['name']] = []

    def start(self, *args, **kwargs):
        self.live_logger.mark_start()

    def stop(self, *args, **kwargs):
        if self.cf_var_logger is not None:
            for log_config in self.cf_var_logger:
                log_config.stop()

        self.live_logger.close()

    def init_cf_logger(self, cf, cf_log_vars, cf_log_period=100):
        self.cf_log_data = copy.deepcopy(cf_log_vars)

        self.cf_var_logger = []
        for name, log_group in cf_log_vars.items():
            var_logger = LogConfig(name=f'{name}', period_in_ms=cf_log_period)

            for par, conf in log_group.items():
                var_logger.add_variable(par, conf["type"])

            cf.log.add_config(var_logger)
            var_logger.data_received_cb.add_callback(self._cf_log_group_callback)
            var_logger.start()

            self.cf_var_logger.append(var_logger)

        logger.debug("logging activated")

    def add_log_group(self, name, *args, **kwargs):
        self.groups[name] = []

    def add_log_entry(self, group_name, entry, *args, **kwargs):
        if group_name == 'frames' and self.kf is not None and entry is not None and entry.get('tvec', None) is not None:
            entry['vel'] = self._update_kf(entry['tvec'])

        self.groups[group_name].append(entry)

        if self.live_logger:
            self.live_logger.write({"type": group_name, 'name': kwargs.get('name', None), "data": entry})

    def get_latest_group_log_data(self, log_group=None):
        if log_group is None:
            log_group = list(self.cf_log_data.keys())[0]

        latest_values = {}
        for par, info in self.cf_log_data[log_group].items():
            data_list = info.get("data", [])
            if data_list:
                latest_values[par] = data_list[-1]
            else:
                latest_values[par] = None
        return latest_values

    def _cf_log_group_callback(self, timestamp, data, log_conf):
        cur_time = time.time()
        group_name = log_conf.name
        data['time'] = cur_time
        if group_name in self.cf_log_data.keys():
            # Append data to each variable in the group
            for var_name, var_info in self.cf_log_data[group_name].items():
                if var_name in data:
                    var_info['data'].append(data[var_name])

        if self.live_logger:
            self.live_logger.write({"type": 'state', "group": group_name, "data": data})

    def _update_kf(self, pos):
        vel = []
        if self.kf:
            for p, kf in zip(pos, self.kf.values()):
                vel.append(kf.update(p))
        return vel

    import subprocess

    def get_git_version(self):
        try:
            self.add_log_group("git")
            output = subprocess.check_output(['git', 'rev-parse', 'HEAD']).decode('ascii').strip()

            if self.live_logger:
                self.live_logger.write({"type": 'git', "data": {'version': output}})
            return
        except (subprocess.CalledProcessError, FileNotFoundError):
            return "Git is not installed or not found in PATH."
