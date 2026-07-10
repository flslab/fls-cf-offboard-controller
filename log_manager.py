import copy
import json
import os
import bisect

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
        self.cf_log_callbacks = {}
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
        reference_offsets = kwargs.get('reference_offsets')
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
            "reference_offsets": reference_offsets,
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
            log_period = cf_log_period * 10
            if "log_period_ms" in group_vars:
                log_period = group_vars["log_period_ms"] * 10
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

        if group_name in self.cf_log_callbacks:
            for callback in self.cf_log_callbacks[group_name]:
                callback(timestamp, data, log_conf)

    def register_cf_log_callback(self, group_name, callback):
        if group_name not in self.cf_log_callbacks:
            self.cf_log_callbacks[group_name] = []
        self.cf_log_callbacks[group_name].append(callback)

    def add_log_group(self, name, *args, **kwargs):
        self.groups[name] = []

    def add_log_entry(self, group_name, entry, *args, **kwargs):
        self.groups[group_name].append(entry | kwargs)

    def get_latest_cf_log_data(self, group_name, param_name):
        if group_name in self.cf_log_data and param_name in self.cf_log_data[group_name]:
            return self.cf_log_data[group_name][param_name]["data"][-1]
        else:
            return None

    def get_cf_log_data_at_timestamp(self, group_name, target_timestamp, search_range=1.0):
        if group_name not in self.cf_log_data:
            return None
            
        times = self.cf_log_times.get(group_name)
        if not times:
            return None
            
        group_data = self.cf_log_data[group_name]
        
        idx = bisect.bisect_left(times, target_timestamp)
        
        smaller_res = None
        larger_res = None
        logger.info(f"target timestamp = {target_timestamp}")
        
        if idx < len(times):
            t_larger = times[idx]
            if t_larger == target_timestamp:
                data_at_idx = {par: group_data[par]["data"][idx] for par in group_data.keys()}
                return (t_larger, data_at_idx), (t_larger, data_at_idx)
            if t_larger - target_timestamp <= search_range:
                data_at_idx = {par: group_data[par]["data"][idx] for par in group_data.keys()}
                larger_res = (t_larger, data_at_idx)
                
        if idx > 0:
            t_smaller = times[idx - 1]
            if target_timestamp - t_smaller <= search_range:
                data_at_idx_minus_1 = {par: group_data[par]["data"][idx - 1] for par in group_data.keys()}
                smaller_res = (t_smaller, data_at_idx_minus_1)
                
        if smaller_res is None and larger_res is None:
            return None
            
        
        return smaller_res, larger_res

