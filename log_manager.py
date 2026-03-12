import copy
import json
import os

from log_manager_abs import LogManager
from cflib.crazyflie.log import LogConfig
import time
import logging
logger = logging.getLogger(__name__)


class IlluminationLogger(LogManager):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.cf_var_logger = None
        self.cf_log_times = []
        self.cf_log_data = None
        self.verbose = kwargs.get('verbose', False)

    def start(self, *args, **kwargs):
        pass

    def stop(self, *args, **kwargs):
        if self.cf_var_logger is not None:
            self.cf_var_logger.stop()

        self.save_logs(kwargs['log_dir'], kwargs['tag'], kwargs['animation_start_time'], kwargs['animation_stop_time'])

    def save_logs(self, log_dir, tag, start_time, end_time):
        if not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        output_data = {
            "start_time": start_time,
            "stop_time": end_time,
        }

        for group, entries in self.groups:
            output_data[group] = entries

        if self.cf_var_logger:
            output_data["cf"] = {
                "time": self.cf_log_times,
                "params": self.cf_log_data
            }

        filename = os.path.join(log_dir, f"{tag}.json")
        with open(filename, 'w') as f:
            json.dump(output_data, f)
        logger.info(f"Logs saved to {filename}")

    def init_cf_logger(self, cf, cf_log_vars, cf_log_period=100):
        self.cf_log_data = copy.deepcopy(cf_log_vars)
        self.cf_var_logger = LogConfig(name='Controller', period_in_ms=cf_log_period)

        for par, conf in cf_log_vars.items():
            self.cf_var_logger.add_variable(par, conf["type"])

        cf.log.add_config(self.cf_var_logger)
        self.cf_var_logger.data_received_cb.add_callback(self.cf_log_callback)
        self.cf_var_logger.start()

    def cf_log_callback(self, timestamp, data, log_conf):
        self.cf_log_times.append(time.time())
        for par in self.cf_log_data.keys():
            self.cf_log_data[par]["data"].append(data[par])
            if self.verbose:
                logger.info(f"{par} = {data[par]}")

    def add_log_group(self, name, *args, **kwargs):
        self.groups[name] = []

    def add_log_entry(self, group_name, entry, *args, **kwargs):
        self.groups[group_name].append(entry)

