import copy
import json
import os
import bisect
from threading import Lock, local
from fc_clock import CrazyflieLogClockMapper

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
        self.cf_log_monotonic_times = {}
        self.cf_log_timing = {}
        self._cf_clock = CrazyflieLogClockMapper()
        self._cf_clock_group = None
        self._cf_clock_lock = Lock()
        self._cf_clock_event_sequence = 0
        self._cf_last_valid_source_time = {}
        self._packet_context = local()
        self.cf_log_data = None
        self.cf_log_callbacks = {}
        self._cf_log_callbacks_lock = Lock()
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
        output_data['fc_clock'] = {
            **self._cf_clock.configuration(), 'reference_group': self._cf_clock_group,
            'time_monotonic_s_basis': 'host callback receipt CLOCK_MONOTONIC',
            'source_time_monotonic_s_basis': 'FC device tick mapped onto host CLOCK_MONOTONIC; null when invalid',
            'other_groups': 'projected onto the reference group clock, not independently fitted',
        }

        for group, entries in self.groups.items():
            output_data[group] = entries

        if len(self.cf_var_loggers) > 0:
            # One snapshot keeps measurement and timing arrays parallel even
            # if a final in-flight callback races with logger shutdown.
            with self._cf_clock_lock:
                for group_name, group_vars in self.cf_log_data.items():
                    output_data[f"cf_{group_name}"] = {
                        "time": list(self.cf_log_times[group_name]),
                        "time_monotonic_s": list(self.cf_log_monotonic_times[group_name]),
                        **{key:list(values) for key,values in self.cf_log_timing[group_name].items()},
                        "params": copy.deepcopy(group_vars)
                    }

        filename = os.path.join(log_dir, f"{tag}.json")
        with open(filename, 'w') as f:
            json.dump(output_data, f)
        logger.info(f"Logs saved to {filename}")

    def init_cf_logger(self, cf, cf_log_vars, cf_log_period=100):
        self.cf_log_data = copy.deepcopy(cf_log_vars)
        self._cf_clock_group = next((name for name in ('QUAT', 'QUATERNION')
                                    if name in self.cf_log_data), next(iter(self.cf_log_data), None))

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
            self.cf_log_monotonic_times[group_name] = []
            self.cf_log_timing[group_name] = {key: [] for key in (
                'cf_timestamp_ms', 'source_time_monotonic_s', 'clock_revision',
                'clock_valid', 'clock_event_sequence')}
            cf_var_logger.start()
            time.sleep(0.1)

    def cf_log_callback(self, timestamp, data, log_conf):
        cur_time = time.time()
        cur_monotonic_time = time.monotonic()
        group_name = log_conf.name
        with self._cf_clock_lock:
            try:
                raw_tick = self._cf_clock._validate_raw_timestamp(timestamp)
                mapped = (self._cf_clock.map(raw_tick, cur_monotonic_time)
                          if group_name == self._cf_clock_group
                          else self._cf_clock.project(raw_tick, cur_monotonic_time))
            except (ValueError, TypeError, OverflowError):
                raw_tick, mapped = None, None
            revision = self._cf_clock.history_revision
            previous = self.cf_log_timing[group_name]
            valid = mapped is not None
            last = self._cf_last_valid_source_time.get(group_name)
            if valid and last is not None and mapped <= last[1]:
                valid = False
                mapped = None
            if valid:
                self._cf_last_valid_source_time[group_name] = (revision, mapped)
            timing = {'cf_timestamp_ms': raw_tick, 'source_time_monotonic_s': mapped,
                      'clock_revision': revision, 'clock_valid': valid,
                      'clock_event_sequence': self._cf_clock_event_sequence}
            self._cf_clock_event_sequence += 1
            for key, value in timing.items():
                previous[key].append(value)
            self.cf_log_times[group_name].append(cur_time)
            self.cf_log_monotonic_times[group_name].append(cur_monotonic_time)
            for par in self.cf_log_data[group_name].keys():
                self.cf_log_data[group_name][par]["data"].append(data[par])

        with self._cf_log_callbacks_lock:
            callbacks = tuple(self.cf_log_callbacks.get(group_name, ()))
        self._packet_context.current = (group_name, timestamp,
                                        {**timing, 'time_monotonic_s': cur_monotonic_time})
        try:
            for callback in callbacks:
                try:
                    callback(timestamp, data, log_conf)
                except Exception:
                    logger.exception("Crazyflie log callback failed for %s; ignored", group_name)
        finally:
            self._packet_context.current = None

        # A subscribed group can be part of a real-time control path. Avoid
        # synchronous per-variable logging on that callback thread.
        if self.verbose and not callbacks:
            for par in self.cf_log_data[group_name].keys():
                logger.info(f"{par} = {data[par]}")

    def get_current_cf_packet_timing(self, group_name, timestamp):
        """Timing for a synchronous subscriber, with no callback ABI change."""
        current = getattr(self._packet_context, 'current', None)
        if current is not None and current[:2] == (group_name, timestamp):
            return dict(current[2])
        return None

    def register_cf_log_callback(self, group_name, callback):
        """Subscribe to complete packets from one configured log group.

        The returned function is safe to call more than once.  The log block
        remains owned by this manager; consumers only own their subscription.
        """
        if not callable(callback):
            raise TypeError("callback must be callable")
        if self.cf_log_data is None or group_name not in self.cf_log_data:
            raise ValueError(f"Crazyflie log group is not configured: {group_name}")

        with self._cf_log_callbacks_lock:
            self.cf_log_callbacks.setdefault(group_name, []).append(callback)

        def unsubscribe():
            with self._cf_log_callbacks_lock:
                callbacks = self.cf_log_callbacks.get(group_name)
                if callbacks is not None and callback in callbacks:
                    callbacks.remove(callback)
                    if not callbacks:
                        self.cf_log_callbacks.pop(group_name, None)

        return unsubscribe

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
            return None, None
            
        times = self.cf_log_times.get(group_name)
        if not times:
            return None, None
            
        group_data = self.cf_log_data[group_name]
        
        idx = bisect.bisect_left(times, target_timestamp)
        
        smaller_res = None
        larger_res = None
        
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
            return None, None
            
        
        return smaller_res, larger_res
