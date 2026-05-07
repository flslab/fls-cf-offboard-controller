import time
import logging
import math

logger = logging.getLogger(__name__)

class PIDAutotuner:
    def __init__(self, controller, config):
        self.controller = controller
        self.current_params = self.controller.cfg.PID_VALUES.copy()
        self.config = config if config else {'z_pos': True, 'z_vel': True, 'xy_pos': True, 'xy_vel': True, 'xy_att': True, 'xy_rate': True}
        
    def set_pid_values(self, params_dict):
        """Send PID values to Crazyflie and store them locally."""
        for param, value in params_dict.items():
            self.controller.cf.param.set_value(param, str(value))
            self.current_params[param] = str(value)
            
    def _get_log_array_lengths(self):
        """Returns the current lengths of the log data arrays to isolate a specific maneuver."""
        lengths = {}
        if not self.controller.log_manager or not self.controller.log_manager.cf_log_data:
            return lengths
        for group, vars in self.controller.log_manager.cf_log_data.items():
            for var_name, var_info in vars.items():
                lengths[var_name] = len(var_info['data'])
        return lengths
        
    def _calculate_itae(self, target_var, state_var, start_indices):
        """Calculates Integral Time Absolute Error (ITAE) for the specified variables."""
        data = self.controller.log_manager.cf_log_data
        
        target_group = None
        state_group = None
        for group, vars in data.items():
            if target_var in vars: target_group = group
            if state_var in vars: state_group = group
            
        if not target_group or not state_group:
            logger.error(f"Could not find log variables {target_var} or {state_var}")
            return float('inf')
            
        t_info = data[target_group][target_var]
        s_info = data[state_group][state_var]
        
        t_data = t_info['data'][start_indices.get(target_var, 0):]
        s_data = s_info['data'][start_indices.get(state_var, 0):]
        
        t_scale = t_info.get("scale", 1.0)
        s_scale = s_info.get("scale", 1.0)
        
        min_len = min(len(t_data), len(s_data))
        if min_len == 0:
            return float('inf')
            
        itae = 0.0
        dt = self.controller.args.cf_log_period / 1000.0  # seconds
        for i in range(min_len):
            t = i * dt
            tv = t_data[i] * t_scale if t_data[i] is not None else 0.0
            sv = s_data[i] * s_scale if s_data[i] is not None else 0.0
            err = abs(tv - sv)
            itae += t * err * dt
            
        return itae

    def run_maneuver(self, axis="z"):
        """Executes a step response maneuver and returns the initial data indices."""
        start_indices = self._get_log_array_lengths()
        
        if axis == "z":
            self.controller.hl_commander.go_to(0, 0, 1.5, 0, 2.0, relative=False)
            self.controller._safe_sleep(2.5)
            self.controller.hl_commander.go_to(0, 0, 1.0, 0, 2.0, relative=False)
            self.controller._safe_sleep(2.5)
        elif axis == "x":
            self.controller.hl_commander.go_to(1.0, 0, 1.0, 0, 2.0, relative=False)
            self.controller._safe_sleep(2.5)
            self.controller.hl_commander.go_to(0.0, 0, 1.0, 0, 2.0, relative=False)
            self.controller._safe_sleep(2.5)
        elif axis == "y":
            self.controller.hl_commander.go_to(0, 1.0, 1.0, 0, 2.0, relative=False)
            self.controller._safe_sleep(2.5)
            self.controller.hl_commander.go_to(0, 0.0, 1.0, 0, 2.0, relative=False)
            self.controller._safe_sleep(2.5)
            
        return start_indices

    def twiddle(self, params_to_tune, axis, target_var, state_var, tol=0.05, max_iters=10):
        """Coordinate descent optimization for a set of PID parameters."""
        p = [float(self.current_params[param]) for param in params_to_tune]
        # dp is initially 10% of the parameter, but not less than 0.01
        dp = [max(0.01, abs(val) * 0.1) for val in p]
        
        def eval_p(p_new):
            new_params = {param: str(val) for param, val in zip(params_to_tune, p_new)}
            self.set_pid_values(new_params)
            start_indices = self.run_maneuver(axis)
            err = self._calculate_itae(target_var, state_var, start_indices)
            return err

        logger.info(f"Twiddle starting for {params_to_tune} on axis {axis}")
        logger.info(f"Initial params: {p}")
        best_err = eval_p(p)
        logger.info(f"Initial Error (ITAE): {best_err:.4f}")
        
        iteration = 0
        while sum(dp) > tol and iteration < max_iters:
            for i in range(len(p)):
                p[i] += dp[i]
                err = eval_p(p)
                
                if err < best_err:
                    best_err = err
                    dp[i] *= 1.1
                else:
                    p[i] -= 2 * dp[i]
                    if p[i] < 0:
                        p[i] = 0.0  # Prevent negative gains
                    
                    err = eval_p(p)
                    
                    if err < best_err:
                        best_err = err
                        dp[i] *= 1.1
                    else:
                        p[i] += dp[i]
                        dp[i] *= 0.9
                        
            iteration += 1
            logger.info(f"Iter {iteration}, Best Error: {best_err:.4f}, params: {[round(x, 4) for x in p]}, dp: {[round(x, 4) for x in dp]}")
            
        logger.info(f"Twiddle completed. Best parameters: {p}")
        
        # Apply the best found parameters
        final_params = {param: str(val) for param, val in zip(params_to_tune, p)}
        self.set_pid_values(final_params)

    def run_autotune(self):
        logger.info("Starting Cascade PID Autotuning Sequence...")
        
        # Ensure log manager is running
        if not self.controller.log_manager:
            logger.error("Logging is disabled. Autotuning requires logging. Please run with --interaction or --illumination and ensure cf_log_vars are active.")
            return
        
        # --- 1. Z-axis Cascade Tuning ---
        logger.info("=== Tuning Z-Axis Cascade ===")
        
        if self.config['z_vel']:
            # Z Velocity Loop
            self.twiddle(['velCtlPid.vzKp', 'velCtlPid.vzKi', 'velCtlPid.vzKd'], 
                         axis="z", target_var="posCtl.targetVZ", state_var="stateEstimate.vz")
        if self.config['z_pos']:
            # Z Position Loop
            self.twiddle(['posCtlPid.zKp', 'posCtlPid.zKi', 'posCtlPid.zKd'], 
                        axis="z", target_var="posCtl.targetZ", state_var="stateEstimate.z")

        # --- 2. X-axis Cascade Tuning ---
        logger.info("=== Tuning X-Axis Cascade ===")

        if self.config['x_rate']:
            # Pitch Rate Loop
            self.twiddle(['pid_rate.pitch_kp', 'pid_rate.pitch_ki', 'pid_rate.pitch_kd'], 
                        axis="x", target_var="controller.pitchRate", state_var="stateEstimateZ.ratePitch")
        
        if self.config['x_att']:
            # Pitch Attitude Loop
            self.twiddle(['pid_attitude.pitch_kp', 'pid_attitude.pitch_ki', 'pid_attitude.pitch_kd'], 
                        axis="x", target_var="controller.pitch", state_var="stateEstimate.pitch")

        if self.config['x_vel']:
            # X Velocity Loop
            self.twiddle(['velCtlPid.vxKp', 'velCtlPid.vxKi', 'velCtlPid.vxKd'], 
                        axis="x", target_var="posCtl.targetVX", state_var="stateEstimate.vx")
        
        if self.config['x_pos']:
            # X Position Loop
            self.twiddle(['posCtlPid.xKp', 'posCtlPid.xKi', 'posCtlPid.xKd'], 
                        axis="x", target_var="posCtl.targetX", state_var="stateEstimate.x")

        # --- 3. Y-axis Cascade Tuning ---
        logger.info("=== Tuning Y-Axis Cascade ===")

        if self.config['y_rate']:
            # Roll Rate Loop
            self.twiddle(['pid_rate.roll_kp', 'pid_rate.roll_ki', 'pid_rate.roll_kd'], 
                        axis="y", target_var="controller.rollRate", state_var="stateEstimateZ.rateRoll")
        
        if self.config['y_att']:
            # Roll Attitude Loop
            self.twiddle(['pid_attitude.roll_kp', 'pid_attitude.roll_ki', 'pid_attitude.roll_kd'], 
                        axis="y", target_var="controller.roll", state_var="stateEstimate.roll")
        
        if self.config['y_vel']:
            # Y Velocity Loop
            self.twiddle(['velCtlPid.vyKp', 'velCtlPid.vyKi', 'velCtlPid.vyKd'], 
                        axis="y", target_var="posCtl.targetVY", state_var="stateEstimate.vy")
        
        if self.config['y_pos']:
            # Y Position Loop
            self.twiddle(['posCtlPid.yKp', 'posCtlPid.yKi', 'posCtlPid.yKd'], 
                     axis="y", target_var="posCtl.targetY", state_var="stateEstimate.y")

        logger.info("Autotuning Sequence Completed!")
        logger.info(f"Final Optimized PID Values: {self.current_params}")
