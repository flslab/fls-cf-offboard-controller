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
        self.cf_var_loggers = []
        self.cf_log_times = {}
        self.cf_log_data = None
        self.verbose = kwargs.get('verbose', False)

    def start(self, *args, **kwargs):
        pass

    def stop(self, *args, **kwargs):
        if len(self.cf_var_loggers) > 0:
            for log_config in self.cf_var_loggers:
                log_config.stop()

        self.save_logs(**kwargs)

    def save_logs(self, **kwargs):
        log_dir = kwargs.get('log_dir')
        tag = kwargs.get('tag')
        start_times = kwargs.get('start_times')
        end_times = kwargs.get('end_times')
        viewpoint_offsets = kwargs.get('viewpoint_offsets')
        mission_start_time = kwargs.get('mission_start_time')
        mission_duration = kwargs.get('mission_duration')
        args = kwargs.get('args')

        if not os.path.exists(log_dir):
            os.makedirs(log_dir, exist_ok=True)

        s, e = (start_times[0], end_times[-1]) if len(end_times) > 0 else (0, 0)

        import subprocess

        output_data = {
            "start_time": s,
            "stop_time": e,
            "mission_start_time": mission_start_time,
            "mission_duration": mission_duration,
            "start_times": start_times,
            "stop_times": end_times,
            "viewpoint_offsets": viewpoint_offsets,
            "args": args,
            "git_ver": subprocess.check_output(['git', 'rev-parse', 'HEAD']).decode('ascii').strip()
        }

        for group, entries in self.groups.items():
            output_data[group] = entries

        if len(self.cf_var_loggers) > 0:
            for group_name, group_vars in self.cf_log_data.items():
                output_data[f"cf_{group_name}"] = {
                    "time": self.cf_log_times[group_name],
                    "params": group_vars
                }

        filename = os.path.join(log_dir, f"{tag}.json")
        with open(filename, 'w') as f:
            json.dump(output_data, f)
        logger.info(f"Logs saved to {filename}")

    def init_cf_logger(self, cf, cf_log_vars, cf_log_period=100):
        self.cf_log_data = copy.deepcopy(cf_log_vars)

        for group_name, group_vars in self.cf_log_data.items():
            log_period = cf_log_period
            if "log_period_ms" in group_vars:
                log_period = group_vars["log_period_ms"]
                group_vars.pop("log_period_ms")

            cf_var_logger = LogConfig(name=group_name, period_in_ms=log_period)
            for par, conf in group_vars.items():
                cf_var_logger.add_variable(par, conf["type"])

            cf.log.add_config(cf_var_logger)
            cf_var_logger.data_received_cb.add_callback(self.cf_log_callback)
            self.cf_var_loggers.append(cf_var_logger)
            self.cf_log_times[group_name] = []
            cf_var_logger.start()
            time.sleep(0.1)

    def cf_log_callback(self, timestamp, data, log_conf):
        cur_time = time.time()
        group_name = log_conf.name
        self.cf_log_times[group_name].append(cur_time)
        for par in self.cf_log_data[group_name].keys():
            self.cf_log_data[group_name][par]["data"].append(data[par])
            if self.verbose:
                logger.info(f"{par} = {data[par]}")

    def add_log_group(self, name, *args, **kwargs):
        self.groups[name] = []

    def add_log_entry(self, group_name, entry, *args, **kwargs):
        self.groups[group_name].append(entry | kwargs)

