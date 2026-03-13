import logging
import time
import traceback

import cflib.crazyflie
import numpy as np

from Interaction.CommandLogger import CommandLogger
from Interaction.flight_behaviors import load_commands


logger = logging.getLogger(__name__)


class InteractionsControl:

    def __init__(self, cf, sleep_function, log_manager, mission, ctrl_rate, log_command=True, execute=True, *args, **kwargs):
        self.cf = cf
        self.log_manager = log_manager
        self.mission = mission
        self.ctrl_rate = ctrl_rate
        if log_command:
            self.hl_commander = CommandLogger(self.cf.high_level_commander, log_function=log_manager.add_log_entry, execute=execute)
            self.lo_commander = CommandLogger(self.cf.commander, log_function=log_manager.add_log_entry, execute=execute)
        else:
            self.hl_commander = self.cf.high_level_commander
            self.lo_commander = self.cf.commander
        self._safe_sleep = sleep_function

    def run(self) -> None:
        self._run_translation()

    def _run_force_render(self) -> None:
        """Execute the force-render haptic interaction."""
        try:
            # self.hover()
            # self.test_rotation_limit()
            self.force_render()
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Render Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_recap(self) -> None:
        """Replay a recorded command log specified by --recap <file>."""
        try:
            cmds = load_commands(self.args.recap)
            self.execute_commands(cmds)
            # self.hover(10)
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Recap Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_translation(self) -> None:
        """Run the velocity-based translation interaction."""
        try:
            translation_setting = self.mission['translation']
            self.interaction_translation_vel(
                vel_threshold=translation_setting['delta_v'],
                z=translation_setting['z'],
                fric_coe=translation_setting['friction_coefficient'],
                base_attitude=translation_setting['base_attitude'],
                duration=translation_setting['duration'],
                v_scalar=translation_setting['v_scalar']
            )
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Translation Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_gimbal(self) -> None:
        """Run the gimbal test."""
        try:
            # self.motor_test()
            self.gimbal_test()
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Gimbal Test Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_HRI_tunnel(self) -> None:
        """Run the HRI tunnel test."""
        try:
            self.test_HRI_tunnel()
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"HRI Tunnel Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()


    def _log_event(self, event_name, data=None):
        if data is None:
            data = {}
        data["time"] = round(time.time(), 6)

        self.log_manager.add_log_entry('events', data, name=event_name)

    def _get_latest_drone_state(self):
        return self.log_manager.get_latest_group_log_data()

    def _get_latest_drone_pos(self, vel=False):
        if vel:
            return np.array(self.log_manager.groups['frames'][-1]["tvec"]), np.array(self.log_manager.groups['frames'][-1].get("vel", None))
        else:
            return np.array(self.log_manager.groups['frames'][-1]["tvec"])

    def test_HRI_tunnel(self):
        hover_pos = [1, 2, 3]
        self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, 5, relative=False)
        self._safe_sleep(5)

    def interaction_translation_vel(
        self,
        vel_threshold=0.01,
        z=1,
        fric_coe=-1.0,
        base_attitude=1,
        duration=60,
        grace_time=0,
        v_scalar=100,
    ):
        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
        self.log_manager.add_log_entry(group_name="configs", entry={'delta_v': vel_threshold, 'Delta': dt, 'delta': v_scalar * dt}, name='Translation Config')
        status = 0

        def check_external_force(vel_vec, pitch, roll):
            tilt_vec = np.array([-np.sin(np.radians(pitch)), np.sin(np.radians(roll))])

            vel_vec = np.array(vel_vec[:2])

            dot_product = np.dot(tilt_vec, vel_vec)

            if dot_product < -0.1:  # Threshold to ignore noise
                return True
            return False

        def detect_speed_threshold(s):
            if s > vel_threshold:
                return True
            return False

        def calculate_braking_angles(v_x, v_y, base_att=base_attitude):
            pitch = np.sign(v_x) * base_att
            roll = -np.sign(v_y) * base_att
            pitch = max(min(pitch, 20), -20)
            roll = max(min(roll, 20), -20)
            return pitch, roll

        last_pos = self._get_latest_drone_pos()
        hover_pos = [last_pos[0], last_pos[1], z]

        self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, 2)
        self._safe_sleep(2)

        logger.info("Starting Force Feedback Interaction mode...")
        self._log_event('Waiting For User Interaction')

        prev_interact_vel = np.zeros(3)
        start_time = time.time()
        while time.time() - start_time < duration:

            state = self._get_latest_drone_state()
            if not state:
                state = {}

            current_pitch = state.get('stateEstimate.pitch', 0.0)
            current_roll = state.get('stateEstimate.roll', 0.0)

            pos, vel = self._get_latest_drone_pos(vel=True)
            pos[2] = z
            vel[2] = 0

            speed = np.linalg.norm(vel)

            if status == 0:  # wait for user interaction
                if detect_speed_threshold(speed):
                    logger.info(f"Switching to Translation From {status}.")
                    # self._log_event('Translation')
                    status = 1
                    prev_interact_vel = vel
                    continue
                else:
                    self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], z, 0)

            elif status == 1:  # pushed by user
                # interact_vel = (vel / speed) * min((speed - 0.01), 0)
                if np.linalg.norm(prev_interact_vel) > 0 > np.dot(vel, prev_interact_vel):
                    logger.info("Ignoring interaction: Direction change > 90 degrees.")
                    interact_vel = np.array([0.0, 0.0, 0.0])
                    speed = 0
                else:
                    # prev_interact_vel = vel
                    interact_vel = vel

                target_pos = pos + interact_vel * dt * v_scalar

                if not detect_speed_threshold(speed):
                    prev_interact_vel = np.zeros(3)
                    if fric_coe > 0:
                        logger.info(f"Switching to Coasting From {status}.")
                        # self._log_event('Coasting')
                        status = 2
                        continue
                    elif grace_time > 0:
                        logger.info(f"Switching to Grace Hover From {status}.")
                        # self._log_event('Grace Hover')
                        hover_pos = target_pos
                        status = 3

                        log_data = {
                            "speed": round(speed, 3),
                            "vel": [round(x, 3) for x in vel],
                            "Pos": [round(x, 3) for x in pos],
                            "Target": [round(x, 3) for x in hover_pos]
                        }
                        self._log_event("User Disengage", log_data)
                        continue
                    else:
                        logger.info(f"Switching to Hover From {status}.")
                        hover_pos = target_pos
                        status = 3

                        log_data = {
                            "speed": round(speed, 3),
                            "vel": [round(x, 3) for x in vel],
                            "Pos": [round(x, 3) for x in pos],
                            "Target": [round(x, 3) for x in target_pos]
                        }
                        self._log_event("User Disengage", log_data)
                        continue

                log_data = {
                    "speed": round(speed, 3),
                    "vel": [round(x, 3) for x in vel],
                    "Pos": [round(x, 3) for x in pos],
                    "Target": [round(x, 3) for x in target_pos]
                }
                self._log_event("User Pushing", log_data)

                if base_attitude < 0:
                    self.lo_commander.send_position_setpoint(target_pos[0], target_pos[1], z, 0)
                else:
                    target_pitch, target_roll = calculate_braking_angles(*interact_vel[:2])
                    self.lo_commander.send_zdistance_setpoint(target_pitch, target_roll, 0, z)

            elif status == 2:  # coasting
                end_pos, coast_t = self.calculate_coasting(pos, vel, fric_coe)
                self.lo_commander.send_notify_setpoint_stop()
                self.hl_commander.go_to(end_pos[0], end_pos[1], z, 0, coast_t, relative=False)
                self._safe_sleep(coast_t)
                logger.info(f"Switching to Hover From {status}.")
                hover_pos = end_pos
                status = 0
                continue

            elif status == 3:  # grace period

                grace_start = time.time()
                while time.time() < grace_time + grace_start:
                    self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], z, 0)
                    self._safe_sleep(dt)

                hover_pos = pos
                status = 0


            self._safe_sleep(dt)

        self.lo_commander.send_notify_setpoint_stop()

    def calculate_coasting(self, cur_pos, cur_vel, deceleration, fixZ=True):
        vx, vy, vz = cur_vel

        if fixZ:
            vz = 0
        speed = np.linalg.norm(cur_vel)

        if speed == 0:
            return cur_pos

        stopping_distance = (speed ** 2) / (2 * deceleration)
        time_to_stop = speed / deceleration

        ux = vx / speed
        uy = vy / speed
        uz = vz / speed

        end_x = cur_pos[0] + (ux * stopping_distance)
        end_y = cur_pos[1] + (uy * stopping_distance)
        if fixZ:
            end_z = cur_pos[2]
        else:
            end_z = cur_pos[2] + (uz * stopping_distance)

        return [end_x, end_y, end_z], time_to_stop

    def execute_commands(self, cmds):
        """
        Reads a JSON command log and executes it, routing to either
        the standard Commander or the HighLevelCommander.
        """
        commander_map = {
            "Commander": self.lo_commander,
            "HighLevelCommander": self.hl_commander,
        }

        logger.info(f"Executing log with {len(cmds)} entries...")
        start_real_time = time.time()

        for entry in cmds:
            target_log_time = entry['time']
            command_full_name = entry['command']
            args = entry['args']
            kwargs = entry['kwargs']

            current_elapsed = time.time() - start_real_time
            wait_duration = target_log_time - current_elapsed
            if wait_duration > 0:
                self._safe_sleep(wait_duration)

            parts = command_full_name.split(".")
            if len(parts) != 2:
                logger.info(f"Skipping malformed command: {command_full_name}")
                continue

            prefix, method_name = parts

            if prefix in commander_map:
                target_obj = commander_map[prefix]
                try:
                    method = getattr(target_obj, method_name)
                    method(*args, **kwargs)
                except AttributeError:
                    logger.info(f"Error: Method '{method_name}' not found on {prefix}")
            else:
                logger.info(f"Error: Unknown commander type '{prefix}'")

        logger.info("Execution finished.")


    # @Todo
    # def force_render(self):
    #     mission_setting = self.mission['drones'][self.args.drone_id]
    #     hover_pos = np.array(mission_setting['target'])
    #
    #     untracked_extra_marker = []
    #     for marker_name, marker_frame in self.extra_markers.items():
    #         if len(marker_frame) == 0:
    #             untracked_extra_marker.append(marker_name)
    #
    #     if untracked_extra_marker:
    #         untrack_info = ""
    #         for m_name in untracked_extra_marker:
    #             untrack_info += f"{m_name} "
    #         logger.info(f"Markers Not Captured: {untrack_info}")
    #         return
    #
    #     logger.info("Starting Force Render sequence")
    #
    #     dt = 1.0 / self.ctrl_rate if self.ctrl_rate else 0.01
    #     ZKp = 1
    #
    #     apparatus_pos = self._get_latest_extra_marker_center()
    #
    #     self.lo_commander.send_notify_setpoint_stop()
    #     self.hl_commander.go_to(0, 0, 1, 0, 5, relative=False)
    #     self._safe_sleep(5)
    #
    # def gimbal_test(self, test_time=30):
    #     dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
    #     start_t = time.time()
    #
    #     self.lo_commander.send_setpoint(0.0, 0.0, 0.0, 0)
    #     while time.time() - start_t < test_time:
    #         # self.lo_commander.send_hover_setpoint(0.0, 0.0, 0.0, 0.9295)
    #         self.lo_commander.send_setpoint(0.0, 0.0, 0.0, 35000)
    #         # self.lo_commander.send_position_setpoint(target_x, target_y, target_z, 0)
    #         self._safe_sleep(dt)
    #
    #     logger.info("Gimbal Test Finished")
    #
    # def pwm_swift(self, wait_time=0.5):
    #     start_time = time.time()
    #
    #     for PWM in np.linspace(10000, 60000, 6, endpoint=True):
    #         self._set_pwm_all(PWM)
    #         self._safe_sleep(wait_time)
    #     self._stop_pwm_override()
    #
    # def motor_test(self, test_time=10):
    #     self._set_pwm_all(20000)
    #     self._safe_sleep(test_time)
    #     self._stop_pwm_override()
    #

    # def test_movement_threshold(self, test_pwm=10000, pwm_step=1000, duration=1.0):
    #     """
    #     Applies a fixed PWM and reports if the marker moved more than 1mm.
    #     """
    #     logger.info(f"Starting movement test: PWM={test_pwm} for {duration}s")
    #
    #     # 1. Record starting position
    #     initial_pos = np.array(self._get_latest_extra_marker_center())
    #
    #     try:
    #         start_time = time.time()
    #         while (time.time() - start_time) < duration:
    #             # Apply the constant test signal
    #
    #             cur_pos = np.array(self._get_latest_extra_marker_center())
    #             if np.linalg.norm(cur_pos - initial_pos) > 0.01:
    #                 break
    #
    #             self._set_pwm_all(test_pwm)
    #             test_pwm += pwm_step
    #             self._safe_sleep(0.1)  # 100Hz update
    #
    #
    #     finally:
    #         # Always stop motors after the test
    #         self._stop_pwm_override()
    #
    #     logger.info(f"Movement Detected by PWM: {test_pwm}")
    #     return
    #
    # def render_stiffness(self, K, duration, sys_friction=0.7164, direction_vec=None, displacement_threshold=0.0004,
    #                      alpha=0.8, ground_test=False):
    #     if direction_vec is None:
    #         direction_vec = [-1, 0, 0]
    #
    #     logger.info(f"Starting stiffness rendering: K={K} for {duration}s")
    #     direction_unit = np.array(direction_vec) / np.linalg.norm(direction_vec)
    #
    #     # 1. Record the initial position of the extra marker center
    #     prev_pos, prev_time = self._get_latest_extra_marker_center(timestamp=True)
    #     init_pos = prev_pos.copy()
    #     start_time = time.time()
    #     # Determine loop rate based on mocap FPS
    #     dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
    #     v_filtered = 0
    #     try:
    #         while (time.time() - start_time) < duration:
    #             loop_start = time.time()
    #
    #             # Get current drone position from mocap
    #             cur_pos, cur_time = self._get_latest_extra_marker_center(timestamp=True)
    #
    #             # --- Calculate Stiffness Force ---
    #             # F = K * Δx (Linear displacement from the recorded initial center)
    #             displacement_vec = cur_pos - init_pos
    #             proj_dist = np.dot(displacement_vec, direction_unit)
    #
    #             time_step = cur_time - prev_time
    #             if time_step <= 0:
    #                 continue
    #             v_raw = proj_dist / time_step
    #
    #             v_scalar = (alpha * v_raw) + ((1 - alpha) * v_filtered)
    #             v_filtered = v_scalar
    #
    #             # if proj_dist - last_displacement <= displacement_threshold:
    #             #     friction_compensation = sys_friction
    #             # elif proj_dist - last_displacement > displacement_threshold:
    #             #     friction_compensation = -sys_friction
    #             # else:
    #             #     friction_compensation = 0
    #
    #             friction_compensation = sys_friction
    #
    #             if proj_dist > displacement_threshold:
    #                 f_stiffness = K * proj_dist + friction_compensation
    #             else:
    #                 f_stiffness = 0.0
    #
    #             # Convert the calculated force to PWM
    #             pwm_value = self.force_to_pwm(f_stiffness)
    #
    #             # Apply PWM to all motors via the controller
    #
    #             if not ground_test:
    #                 self._set_pwm_all(pwm_value)
    #
    #             now = time.time()
    #             self._log_event("Rendering Force",
    #                             {'type': 'stiffness', 'force': f_stiffness, 'displacement': proj_dist,
    #                              'vel': v_filtered, 'time': now})
    #
    #             # Maintain consistent update frequency
    #             elapsed = now - loop_start
    #             if elapsed < dt:
    #                 self._safe_sleep(dt - elapsed)
    #
    #     finally:
    #         # 2. Stop PWM override at the end of the duration
    #         self._stop_pwm_override()
    #         logger.info("Stiffness rendering complete. PWM override stopped.")
    #
    # def render_Karnopp_friction(self, Cp, Dp, delta_v, duration, sys_friction=0.9, direction_vec=None, wait_time=-1,
    #                             alpha=0.3, min_displacement=0.0006, ground_test=False):
    #     """
    #     Simulates friction following the Karnopp model with a conditional Low Pass Filter.
    #     alpha: Smoothing factor (0.0 to 1.0). Lower is smoother/slower.
    #     """
    #     if direction_vec is None:
    #         direction_vec = [-1, 0, 0]
    #
    #     logger.info(f"Starting Karnopp friction: Cp={Cp}, Dp={Dp}, dv={delta_v}")
    #     direction_unit = np.array(direction_vec) / np.linalg.norm(direction_vec)
    #
    #     prev_pos, prev_time = self._get_latest_extra_marker_center(timestamp=True)
    #     init_pos = prev_pos.copy()
    #     start_time = time.time()
    #
    #     dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
    #     wait_threshold = self.ctrl_rate * wait_time
    #     wait_count = 0
    #
    #     v_filtered = 0.0
    #     has_broken_static = False
    #
    #     try:
    #         while (time.time() - start_time) < duration:
    #             loop_start = time.time()
    #
    #             cur_pos, cur_time = self._get_latest_extra_marker_center(timestamp=True)
    #             time_step = cur_time - prev_time
    #             if time_step <= 0:
    #                 continue
    #
    #             total_displacement_vec = cur_pos - init_pos
    #             total_displacement = np.dot(total_displacement_vec, direction_unit)
    #
    #             # 1. Calculate raw velocity
    #             displacement_vec = cur_pos - prev_pos
    #             displacement = np.dot(displacement_vec, direction_unit)
    #             v_raw = displacement / time_step
    #
    #             # 2. Check for first-time activation
    #             if not has_broken_static and total_displacement > 0.01:
    #                 has_broken_static = True
    #                 v_filtered = v_raw  # Seed the filter with the current velocity
    #
    #             # 3. Apply Low Pass Filter only if activated
    #             if has_broken_static:
    #                 v_scalar = (alpha * v_raw) + ((1 - alpha) * v_filtered)
    #                 v_filtered = v_scalar
    #             else:
    #                 v_filtered = v_raw
    #
    #             # 4. Apply Karnopp Model logic
    #             if total_displacement > 0.01 and displacement > min_displacement:
    #                 if abs(v_filtered) > delta_v:
    #                     f_friction = Cp
    #                     wait_count = 0
    #                 else:
    #                     f_friction = Dp if abs(v_filtered) > 0 else 0
    #
    #             else:
    #                 v_filtered = 0
    #                 if 0 < wait_threshold <= wait_count:
    #                     if displacement > min_displacement:
    #                         f_friction = 0
    #                         wait_count = 0
    #                     else:
    #                         f_friction = sys_friction
    #                 else:
    #                     f_friction = 0
    #                     if has_broken_static:
    #                         wait_count += 1
    #
    #             # 5. Apply directional mask and render
    #             final_force = max(0, f_friction)
    #             pwm_value = self.force_to_pwm(final_force)
    #
    #             if not ground_test:
    #                 self._set_pwm_all(pwm_value)  # Uncomment to apply
    #             logger.info(
    #                 f"Displacement: {total_displacement:6.4f}, Delta P: {displacement:6.4f}, V_filt: {v_filtered:6.4f}, Force: {final_force:4.3f}")
    #
    #             now = time.time()
    #             self._log_event("Rendering Force",
    #                             {'type': 'friction', 'force': final_force, 'displacement': total_displacement,
    #                              'vel': v_filtered, 'time': now})
    #
    #             prev_pos, prev_time = cur_pos, cur_time
    #             elapsed = now - loop_start
    #             if elapsed < dt:
    #                 self._safe_sleep(dt - elapsed)
    #
    #     finally:
    #         self._stop_pwm_override()
    #         logger.info("Friction rendering complete.")