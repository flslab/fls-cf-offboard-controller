import logging
import threading
import time
import traceback

import cflib.crazyflie
import numpy as np
import zmq

from Interaction.CommandWrapper import CommandWrapper
from Interaction.flight_behaviors import load_commands

# from Interaction.collision_avoidance.simulation import apf_velocity

logger = logging.getLogger(__name__)


class BoundaryExceededError(Exception):
    """Exception raised when the drone leaves the defined interaction space."""
    pass


def calculate_tilt(roll, pitch, degrees=True):
    if degrees:
        roll = np.radians(roll)
        pitch = np.radians(pitch)

    # Calculate the cosine of the total tilt
    cos_tilt = np.cos(roll) * np.cos(pitch)
    tilt_rad = np.arccos(np.clip(cos_tilt, -1.0, 1.0))

    return np.degrees(tilt_rad) if degrees else tilt_rad


class InteractionsControl:

    def __init__(self, cf, sleep_function, log_manager, mission, ctrl_rate, log_command=True, execute=True,
                 leader_info=None, pub_socket=None, sub_socket=None, drone_id=None, set_color=None,
                 orchestrator_ip=None, *args, **kwargs):
        self.cf = cf
        self.log_manager = log_manager
        self.mission = mission
        self.ctrl_rate = ctrl_rate
        self.pub_socket = pub_socket
        self.sub_socket = sub_socket
        self.drone_id = drone_id
        self.set_color = set_color
        self.orchestrator_ip = orchestrator_ip
        # Network followers use their own 'frames' position, not the leader's mocap group
        self.pos_group_name = 'frames' if (leader_info is None or sub_socket is not None) else f"{leader_info['id']}"

        log_function = log_manager.add_log_entry if log_command else None
        offset = np.zeros(3) if leader_info is None else np.array(leader_info['offset'])
        self.hl_commander = CommandWrapper(self.cf.high_level_commander, log_function=log_function, execute=execute,
                                           offset=offset)
        self.lo_commander = CommandWrapper(self.cf.commander, log_function=log_function, execute=execute, offset=offset)
        self._safe_sleep = sleep_function
        self.bounds = self.mission.get('boundary_limits', None)

    def run(self) -> None:

        action = self.mission.get('Interaction', {}).get('action')

        if action == 'peer_latency_test':
            self._run_peer_latency_test()
            return

        # When both sockets are present the behaviour depends on whether avoidance
        # is configured.  With avoidance: UI-LB runs translation + APF broadcast.
        # Without: symmetric peer translation (all drones equal).
        if self.pub_socket is not None and self.sub_socket is not None:
            if self.mission.get('avoidance'):
                # self.run_translation_broadcast()
                pass
            else:
                self._run_peer_translation()
            return

        if action == 'rotation_test':
            self._run_rotation_limit()
        elif action == 'translation':
            self._run_translation()

    def check_interaction_boundary(self, pos=None):
        if self.bounds is None:
            return

        if pos is None:
            pos = self._get_latest_pos()

        if pos is None or len(pos) < 3:
            logger.warning("Could not retrieve position for boundary check.")
            return

        x, y, z = pos[0], pos[1], pos[2]

        if not (self.bounds['x_min'] <= x <= self.bounds['x_max']):
            raise BoundaryExceededError(
                f"X position ({x:.3f}) breached bounds [{self.bounds['x_min']}, {self.bounds['x_max']}]")

        if not (self.bounds['y_min'] <= y <= self.bounds['y_max']):
            raise BoundaryExceededError(
                f"Y position ({y:.3f}) breached bounds [{self.bounds['y_min']}, {self.bounds['y_max']}]")

        if not (self.bounds['z_min'] <= z <= self.bounds['z_max']):
            raise BoundaryExceededError(
                f"Z position ({z:.3f}) breached bounds [{self.bounds['z_min']}, {self.bounds['z_max']}]")

    def test_flight(self):

        try:
            st = time.time()
            while time.time() < st + 10:
                self.lo_commander.send_position_setpoint(1, 1, 1, 0)
                self._safe_sleep(0.01)
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Test Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_rotation_limit(self) -> None:
        """Execute the force-render haptic interaction."""
        try:
            setting = self.mission['Interaction']['config']
            rads_to_deg = 57.3
            yawrate = setting['rads_per_sec'] * rads_to_deg
            self.test_rotation_limit(yawrate=yawrate, duration=setting['duration'])
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Render Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

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

    def run_unit_test(self, command_type='lo'):
        distance_to_test = [0.01, 0.02, 0.2, 0.5, 1, 2]
        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01

        self._safe_sleep(2)

        for d in distance_to_test:
            pos, vel = self._get_latest_pos(vel=True)
            hover_pos = [pos[0], pos[1] + d, 1]
            travel_time = d * 2

            if command_type == 'hi':
                self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, travel_time, relative=False)
                self._safe_sleep(travel_time + 5)
            else:
                start_time = time.time()
                while time.time() < start_time + travel_time + 5:
                    self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                    self._safe_sleep(dt)

        self.lo_commander.send_notify_setpoint_stop()
        return

    def run_recap(self, file) -> None:
        """Replay a single recorded command log. File selection and takeoff/land
        orchestration are handled by controller.py before calling IC.run()."""
        try:
            cmds = load_commands(file)
            self.execute_commands(cmds)
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Recap Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_translation(self) -> None:
        """Run the velocity-based translation interaction."""
        try:
            translation_setting = self.mission['Interaction']['config']
            mass_lb = translation_setting.get('mass_lightbender', 1.0)
            mass_virtual = translation_setting.get('mass_virtual', 1.0)
            self.interaction_translation_vel(
                vel_threshold=translation_setting['delta_v'],
                z=translation_setting['z'],
                fric_coe=translation_setting['friction_coefficient'],
                base_attitude=translation_setting['base_attitude'],
                duration=translation_setting['duration'],
                v_scalar=translation_setting['v_scalar'],
                grace_time=translation_setting['grace_time'],
                alpha_vel=translation_setting.get('alpha_vel', 1),
                pub_socket=self.pub_socket,
                mass_ratio=mass_lb / mass_virtual,
                init_hover=self.mission['drones'][self.drone_id]['target'][:3],
                blender_port=translation_setting.get('blender_port', None)
            )
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Translation Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    def _run_peer_translation(self) -> None:
        """Run symmetric peer interaction — every drone can push and follow."""
        try:
            translation_setting = self.mission['Interaction']['config']
            self.interaction_peer_translation_vel(
                drone_id=self.drone_id,
                vel_threshold=translation_setting['delta_v'],
                z=translation_setting['z'],
                fric_coe=translation_setting['friction_coefficient'],
                base_attitude=translation_setting['base_attitude'],
                duration=translation_setting['duration'],
                v_scalar=translation_setting['v_scalar'],
                grace_time=translation_setting['grace_time'],
                pub_socket=self.pub_socket,
                sub_socket=self.sub_socket,
            )
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Peer Translation Error: {e}\nTraceback:\n{tb_info}")
        finally:
            self.lo_commander.send_notify_setpoint_stop()

    # def run_translation_broadcast(self) -> None:
    #     """UI-LB mode: run translation interaction while broadcasting APF avoidance
    #     commands to all passive I-LBs.
    #
    #     The translation loop runs in a background thread (the user-facing interaction).
    #     The main thread runs the APF loop at the avoidance control rate:
    #       1. Drain sub_socket for the latest position report from each I-LB.
    #       2. Fetch own position from log_manager (UI-LB position).
    #       3. Call apf_velocity() for every I-LB — no PID, no simulation step.
    #       4. Integrate v_cmd * dt to get the desired absolute position.
    #       5. Broadcast {"type": "avoid_cmd", "commands": {lb_id: [x,y,z], ...}}
    #          via pub_socket so each I-LB can apply the setpoint directly.
    #
    #     Stops when the translation thread finishes (interaction duration elapsed).
    #     """
    #     avoidance_cfg = self.mission.get('avoidance', {})
    #     eta      = avoidance_cfg.get('eta',       0.5)
    #     zeta     = avoidance_cfg.get('zeta',      0.0)
    #     d_detect = avoidance_cfg.get('d_detect',  0.47)
    #     v_max    = avoidance_cfg.get('v_max',     2.0)
    #     rate     = avoidance_cfg.get('ctrl_rate', self.ctrl_rate if self.ctrl_rate > 0 else 50)
    #     dt       = 1.0 / rate
    #
    #     # Goal positions for each passive I-LB (their static hover targets)
    #     drone_mission = self.mission.get('drones', {})
    #     lb_goals = {
    #         lb_id: np.array(cfg['target'][:3])
    #         for lb_id, cfg in drone_mission.items()
    #         if cfg.get('interaction') == 'avoid'
    #     }
    #     # Last known positions start at goal (I-LBs are initially at target)
    #     lb_positions = {lb_id: goal.copy() for lb_id, goal in lb_goals.items()}
    #     pos_lock = threading.Lock()
    #
    #     translation_thread = threading.Thread(target=self._run_translation, daemon=True)
    #     translation_thread.start()
    #
    #     try:
    #         while translation_thread.is_alive():
    #             # 1. Receive latest position reports from passive I-LBs
    #             if self.sub_socket is not None:
    #                 msg = self.sub_socket.recv_latest()
    #                 if msg is not None and msg.get('type') == 'position':
    #                     lb_id = msg.get('drone_id')
    #                     pos   = msg.get('pos')
    #                     if lb_id in lb_positions and pos is not None:
    #                         with pos_lock:
    #                             lb_positions[lb_id] = np.array(pos)
    #
    #             # 2. Own (UI-LB) position
    #             try:
    #                 ui_pos = self._get_latest_pos()
    #             except Exception:
    #                 time.sleep(dt)
    #                 continue
    #
    #             # 3-4. APF velocity → position offset for each I-LB
    #             cmds = {}
    #             with pos_lock:
    #                 for lb_id, lb_pos in lb_positions.items():
    #                     v_cmd = apf_velocity(lb_pos, lb_goals[lb_id], ui_pos,
    #                                          eta, zeta, d_detect, v_max)
    #                     offset = v_cmd * dt
    #                     cmds[lb_id] = offset.tolist()
    #                     # Advance local position estimate for next APF step
    #                     lb_positions[lb_id] = lb_pos + offset
    #
    #             # 5. Broadcast to all passive I-LBs
    #             if self.pub_socket is not None and cmds:
    #                 self.pub_socket.send_json({'type': 'avoid_cmd', 'commands': cmds})
    #
    #             time.sleep(dt)
    #     except Exception as e:
    #         tb_info = traceback.format_exc()
    #         logging.error(f"Avoidance Broadcast Error: {e}\nTraceback:\n{tb_info}")
    #     finally:
    #         translation_thread.join(timeout=2.0)
    #
    # def run_passive_avoidance(self) -> None:
    #     """Passive I-LB mode: publish own position to UI-LB and execute APF commands.
    #
    #     Receiving is non-blocking (recv_latest drains the ZMQ/UDP buffer and
    #     returns the newest message, or None if nothing arrived).
    #
    #     Message protocol:
    #       publish  → {"type": "position",  "drone_id": <id>,  "pos": [x, y, z]}
    #       receive  ← {"type": "avoid_cmd", "commands": {<id>: [dx, dy, dz], ...}}
    #
    #     Each command is a position *offset* (delta) that is added to the current
    #     desired hover position, not an absolute setpoint.
    #     """
    #     avoidance_cfg = self.mission.get('avoidance', {})
    #     rate     = avoidance_cfg.get('ctrl_rate', self.ctrl_rate if self.ctrl_rate > 0 else 50)
    #     dt       = 1.0 / rate
    #     duration = self.mission.get('drones', {}).get(self.drone_id, {}).get('delta_t', 60)
    #
    #     drone_cfg = self.mission['drones'][self.drone_id]
    #     hover_pos = np.array(drone_cfg['target'][:3], dtype=float)
    #
    #     start_t = time.time()
    #     try:
    #         while time.time() - start_t < duration:
    #             # Publish own position so the UI-LB can run APF for this drone
    #             try:
    #                 my_pos = self._get_latest_pos()
    #                 if self.pub_socket is not None:
    #                     self.pub_socket.send_json({
    #                         'type':     'position',
    #                         'drone_id': self.drone_id,
    #                         'pos':      my_pos.tolist(),
    #                     })
    #             except Exception:
    #                 pass
    #
    #             # Apply latest avoidance offset if one has arrived (non-blocking)
    #             if self.sub_socket is not None:
    #                 msg = self.sub_socket.recv_latest()
    #                 if msg is not None and msg.get('type') == 'avoid_cmd':
    #                     cmds = msg.get('commands', {})
    #                     if self.drone_id in cmds:
    #                         hover_pos += np.array(cmds[self.drone_id], dtype=float)
    #
    #             self.lo_commander.send_position_setpoint(
    #                 hover_pos[0], hover_pos[1], hover_pos[2], 0)
    #             time.sleep(dt)
    #     except Exception as e:
    #         tb_info = traceback.format_exc()
    #         logging.error(f"Passive Avoidance Error: {e}\nTraceback:\n{tb_info}")
    #     finally:
    #         self.lo_commander.send_notify_setpoint_stop()

    def _run_peer_latency_test(self) -> None:
        """Peer TCP latency test — comparable to live interaction transport."""
        cfg = self.mission['Interaction']['config']
        num_packets = cfg.get('num_packets', 1_000_000)
        role = self.mission['drones'][self.drone_id].get('role', 'sender')

        payload = {
            "type": "push",
            "drone_id": self.drone_id,
            "accumulated_offset": [0.0, 0.0, 0.0],
            "push_start_time": 0.0,
        }

        try:
            if role == 'receiver':
                logger.info(f"[Latency Test] Receiver — waiting for {num_packets:,} packets...")
                first_arrival = None
                last_arrival = None
                received = 0
                timed_out = False
                while received < num_packets:
                    msg = self.sub_socket.recv_one_timeout(10.0)
                    if msg is None:
                        logger.warning(
                            f"[Latency Test] No packet for 10s — terminating early "
                            f"({received:,}/{num_packets:,} received, "
                            f"{num_packets - received:,} lost)."
                        )
                        timed_out = True
                        break
                    t = time.perf_counter()
                    if first_arrival is None:
                        first_arrival = t
                    last_arrival = t
                    received += 1

                total_time = (last_arrival - first_arrival) if (first_arrival and last_arrival) else 0.0
                avg_iat = total_time / (received - 1) * 1e6 if received > 1 else 0.0
                logger.info(f"[Latency Test] Receiver Results:")
                logger.info(f"  Packets received             : {received:,} / {num_packets:,}"
                            + (" (INCOMPLETE — packet loss)" if timed_out else ""))
                logger.info(f"  Total reception time         : {total_time:.4f} s")
                logger.info(f"  Avg packet inter-arrival time: {avg_iat:.4f} us")

            else:
                logger.info(f"[Latency Test] Sender — sending {num_packets:,} packets...")
                time.sleep(1)  # give receiver SUB socket time to connect
                s = time.perf_counter()
                for i in range(num_packets):
                    payload["push_start_time"] = i
                    self.pub_socket.send_json(payload)
                e = time.perf_counter() - s

                logger.info(f"[Latency Test] Sender Results:")
                logger.info(f"  Total elapsed time           : {e:.4f} s")
                logger.info(f"  Avg per-packet send time     : {e / num_packets * 1e6:.4f} us")
                logger.info(f"  Throughput                   : {num_packets / e:.0f} msgs/s")

        except Exception as ex:
            tb_info = traceback.format_exc()
            logging.error(f"Latency Test Error: {ex}\nTraceback:\n{tb_info}")

    def _run_network_follow(self) -> None:
        """Run as a network follower, mirroring the interaction drone's state."""
        try:
            translation_setting = self.mission['Interaction']['config']
            self.interaction_follow_network(
                sub_socket=self.sub_socket,
                z=translation_setting['z'],
                fric_coe=translation_setting['friction_coefficient'],
                base_attitude=translation_setting['base_attitude'],
                duration=translation_setting['duration'],
                v_scalar=translation_setting['v_scalar'],
            )
        except Exception as e:
            tb_info = traceback.format_exc()
            logging.error(f"Network Follow Error: {e}\nTraceback:\n{tb_info}")
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

    def _get_latest_pos(self, vel=False):
        if vel:
            return np.array(self.log_manager.groups[self.pos_group_name][-1]["tvec"]), np.array(
                self.log_manager.groups[self.pos_group_name][-1].get("vel", None))
        else:
            return np.array(self.log_manager.groups[self.pos_group_name][-1]["tvec"])

    def test_HRI_tunnel(self):
        hover_pos = [1, 2, 3]
        self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, 5, relative=False)
        self._safe_sleep(5)

    def test_rotation_limit(self, yawrate, duration=5):
        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
        start_t = time.time()

        while time.time() - start_t < 2:
            self.lo_commander.send_position_setpoint(0.0, 0.0, 1.0, 0)
            self._safe_sleep(dt)

        start_t = time.time()
        while time.time() - start_t < duration:
            self.lo_commander.send_hover_setpoint(0.0, 0.0, yawrate, 1.0)
            self._safe_sleep(dt)

        while time.time() - start_t < duration + 2:
            self.lo_commander.send_position_setpoint(0.0, 0.0, 1.0, 0)
            self._safe_sleep(dt)

    def _get_drone_by_id(self, drone_id):
        for drone in self.manifest['drones']:
            if drone['id'] == drone_id:
                return drone

    def interaction_translation_vel(
            self,
            vel_threshold=0.01,
            z=1,
            fric_coe=-1.0,
            base_attitude=1,
            duration=60,
            grace_time=1,
            v_scalar=None,
            alpha_vel=1,
            pub_socket=None,
            mass_ratio=1.0,
            init_hover=None,
            blender_port=None
    ):
        if v_scalar is None:
            v_scalar = np.array([10, 10, 2])
        else:
            v_scalar = np.array(v_scalar)
        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01
        if isinstance(grace_time, (int, float)):
            get_grace_time = lambda a, v: grace_time
            stabilize_time = grace_time

        else:
            import joblib
            # Load the dictionary containing both the transformer and the model
            saved_poly_data = joblib.load(grace_time)

            poly_transformer = saved_poly_data['poly']  # Assuming 'poly' is the key for PolynomialFeatures
            loaded_model = saved_poly_data['model']

            # Use the transformer to expand features before predicting
            get_grace_time = lambda a, v: loaded_model.predict(
                poly_transformer.transform([[a, v]])
            )[0] + 0.2
            stabilize_time = 'dynamic'

        self.log_manager.add_log_entry(group_name="configs",
                                       entry={'delta_v': vel_threshold, 'Delta': dt, 'delta': v_scalar[0] * dt,
                                              "Orientation CMD": base_attitude, 'Stabilize Time': stabilize_time,
                                              'mass_ratio': mass_ratio},
                                       name='Translation Config')

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

        _G = 9.81  # m/s²

        def calculate_braking_angles(dv_x, dv_y, yaw_deg=0.0, max_attitude=20.0):
            """Compute pitch/roll to emulate a virtual mass, considering global yaw.

            The change in velocity is caused by external force minus resistance generated by the drone:
              m_lb * a = F_ext - F_res
            To emulate m_virtual, the user should feel F_ext = m_virtual * a.
            Thus, F_res = (m_virtual - m_lb) * a.
            The drone generates aerodynamic force F_aero = -F_res = (m_lb - m_virtual) * a.

            Assuming thrust along body Z equals the drone's weight (T = m_lb * g),
            the horizontal force is T * sin(angle).

              F_aero_x = m_lb * g * sin(pitch) = (m_lb - m_virtual) * (dv_x / dt)
              sin(pitch) = (1 - m_virtual / m_lb) * (dv_x / (g * dt))
                         = (1 - 1 / mass_ratio) * (dv_x / (g * dt))

            Sign convention (Crazyflie):
              +pitch tilts nose up  → force in +Xsi
              +roll  tilts left     → force in −Y
            """
            # Rotate global dv to body frame based on yaw
            yaw_rad = np.radians(yaw_deg)
            cos_y = np.cos(yaw_rad)
            sin_y = np.sin(yaw_rad)
            body_dv_x = dv_x * cos_y + dv_y * sin_y
            body_dv_y = -dv_x * sin_y + dv_y * cos_y

            inv_mass_ratio = 1.0 / mass_ratio if mass_ratio != 0 else 1.0
            sin_pitch = -(1 - inv_mass_ratio) * body_dv_x / (_G * dt)
            sin_roll = -(1 - inv_mass_ratio) * body_dv_y / (_G * dt)

            pitch = np.degrees(np.arcsin(np.clip(sin_pitch, -1.0, 1.0)))
            roll = np.degrees(np.arcsin(np.clip(sin_roll, -1.0, 1.0)))

            pitch = max(min(pitch, max_attitude), -max_attitude)
            roll = max(min(roll, max_attitude), -max_attitude)
            return pitch, roll

        if init_hover:
            last_pos = init_hover
        else:
            while True:
                try:
                    last_pos = self._get_latest_pos()
                    break
                except Exception as e:
                    time.sleep(0.001)

        if z is not None:
            if z < 0:
                z = last_pos[2]
            hover_pos = [last_pos[0], last_pos[1], z]
        else:
            hover_pos = [last_pos[0], last_pos[1], last_pos[2]]

        self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, 2)
        self._safe_sleep(2)

        logger.info("Starting Force Feedback Interaction mode...")
        self._log_event('Waiting For User Interaction')

        interaction_heading = np.zeros(3)
        v_virtual = np.zeros(3)  # virtual-object velocity, integrated from F/m_virtual
        prev_interact_vel = np.zeros(3)  # previous tick's interact_vel, for differentiation

        if blender_port:
            try:
                blender_port = int(blender_port)
            except (TypeError, ValueError):
                logger.error(f"Invalid Blender TCP port: {blender_port}")
                blender_port = None
        blender_state = None
        if blender_port:
            import json as _json
            import socket as _socket

            blender_state = {
                'edit_active': False,
                'edit_end_time': 0.0,
                'finish_requested': False,
                'stop_at_next_zero': False,
                'status': 0,
                'sending_positions': False,
                'sock': None,
                'sock_lock': threading.Lock(),
            }
            worker_done = threading.Event()

            def blender_worker():
                host_candidates = []
                if self.orchestrator_ip:
                    host_candidates.append(self.orchestrator_ip)
                for fallback_host in ("127.0.0.1", "localhost"):
                    if fallback_host not in host_candidates:
                        host_candidates.append(fallback_host)

                recv_buffer = ""
                next_log_time = 0.0

                try:
                    while not worker_done.is_set():
                        with blender_state['sock_lock']:
                            sock = blender_state['sock']

                        if sock is None:
                            for host in host_candidates:
                                try:
                                    candidate = _socket.create_connection((host, blender_port), timeout=1.0)
                                    candidate.settimeout(0.1)
                                    with blender_state['sock_lock']:
                                        blender_state['sock'] = candidate
                                    recv_buffer = ""
                                    logger.info(f"Connected to Blender at {host}:{blender_port}")
                                    break
                                except OSError as exc:
                                    now = time.time()
                                    if now >= next_log_time:
                                        logger.info(f"Waiting for Blender at {host}:{blender_port} ({exc})")
                                        next_log_time = now + 2.0

                            with blender_state['sock_lock']:
                                if blender_state['sock'] is None:
                                    time.sleep(0.25)
                                    continue
                                sock = blender_state['sock']

                        try:
                            data = sock.recv(1024)
                            if not data:
                                logger.info("Blender connection closed. Retrying.")
                                with blender_state['sock_lock']:
                                    blender_state['sock'] = None
                                try:
                                    sock.close()
                                except OSError:
                                    pass
                                recv_buffer = ""
                                time.sleep(0.1)
                                continue

                            recv_buffer += data.decode('utf-8', errors='ignore')
                            while "\n" in recv_buffer:
                                raw_line, recv_buffer = recv_buffer.split("\n", 1)
                                raw_line = raw_line.strip()
                                if not raw_line:
                                    continue
                                try:
                                    msg = _json.loads(raw_line)
                                except _json.JSONDecodeError:
                                    logger.debug(f"Ignoring malformed Blender message: {raw_line!r}")
                                    continue
                                if not isinstance(msg, dict):
                                    continue
                                cmd = msg.get("cmd")
                                if cmd == "start_edit":
                                    edit_dur = float(msg.get("duration", 10.0))
                                    blender_state['edit_active'] = True
                                    blender_state['edit_end_time'] = time.time() + edit_dur
                                    blender_state['finish_requested'] = False
                                    blender_state['stop_at_next_zero'] = False
                                    blender_state['sending_positions'] = True
                                    logger.info(f"Edit mode started for {edit_dur}s, streaming positions.")
                                elif cmd == "finish_edit":
                                    blender_state['finish_requested'] = True
                                    blender_state['sending_positions'] = True
                                    blender_state['stop_at_next_zero'] = True
                                    logger.info("Finish edit received; will stream until next status 0.")

                        except _socket.timeout:
                            pass
                        except (BlockingIOError, InterruptedError):
                            pass
                        except OSError as exc:
                            logger.info(f"Blender socket error, reconnecting: {exc}")
                            with blender_state['sock_lock']:
                                try:
                                    blender_state['sock'].close()
                                except OSError:
                                    pass
                                blender_state['sock'] = None
                            recv_buffer = ""

                finally:
                    with blender_state['sock_lock']:
                        if blender_state['sock'] is not None:
                            try:
                                blender_state['sock'].close()
                            except OSError:
                                pass
                            blender_state['sock'] = None

            blender_thread = threading.Thread(target=blender_worker, daemon=True)
            blender_thread.start()

            def send_blender_position(pos_to_send):
                with blender_state['sock_lock']:
                    sock = blender_state['sock']
                if sock is None:
                    return
                try:
                    resp = {
                        "id": self.drone_id,
                        "position": [round(float(x), 3) for x in pos_to_send],
                    }
                    sock.sendall((_json.dumps(resp) + "\n").encode('utf-8'))
                except Exception:
                    with blender_state['sock_lock']:
                        try:
                            if blender_state['sock'] is not None:
                                blender_state['sock'].close()
                        except OSError:
                            pass
                        blender_state['sock'] = None

        # --- Unified main loop ---
        # When blender_state is None the edit-mode pause is skipped and elapsed
        # time accumulates every tick, matching the original no-blender behaviour.
        elapsed_non_edit = 0.0
        loop_tick_time = time.time()
        last_blender_send_time = 0.0

        while elapsed_non_edit < duration:
            now = time.time()
            tick = now - loop_tick_time
            loop_tick_time = now

            if blender_state is not None:
                if blender_state['edit_active'] and now >= blender_state['edit_end_time']:
                    blender_state['finish_requested'] = True
                    blender_state['sending_positions'] = True
                    blender_state['stop_at_next_zero'] = True
                    blender_state['edit_end_time'] = float('inf')
                    logger.info("Edit duration expired; will stream until next status 0.")

                if blender_state['finish_requested'] and status == 0:
                    send_blender_position(hover_pos)
                    last_blender_send_time = now
                    blender_state['sending_positions'] = False
                    blender_state['finish_requested'] = False
                    blender_state['stop_at_next_zero'] = False
                    blender_state['edit_active'] = False
                    logger.info("Sent final hover position, stopped streaming.")

                if not blender_state['edit_active']:
                    # Pause interaction during Blender edit; only accumulate non-edit time.
                    elapsed_non_edit += tick
                    status = 0
                    self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                    self._safe_sleep(dt)
                    continue
            else:
                elapsed_non_edit += tick

            state = self._get_latest_drone_state()
            if not state:
                state = {}

            current_pitch = state.get('stateEstimate.pitch', 0.0)
            current_roll = state.get('stateEstimate.roll', 0.0)
            current_yaw = state.get('stateEstimate.yaw', 0.0)

            state_vx = state.get('stateEstimate.vx', 0.0)
            state_vy = state.get('stateEstimate.vy', 0.0)
            state_vz = state.get('stateEstimate.vz', 0.0)
            state_vel = np.array([state_vx, state_vy, state_vz])

            pos, vel = self._get_latest_pos(vel=True)

            vel = (alpha_vel * vel) + ((1.0 - alpha_vel) * state_vel)
            self.check_interaction_boundary(pos)
            if z is not None:
                pos[2] = z
                vel[2] = 0

            speed = np.linalg.norm(vel)

            if status == 0:  # wait for user interaction
                if blender_state is not None:
                    blender_state['status'] = 0

                if detect_speed_threshold(speed):
                    logger.info(f"Switching to Translation From {status}.")
                    if self.set_color:
                        self.set_color([0, 255, 0])
                    status = 1
                    interaction_heading = vel
                    v_virtual = np.zeros(3)
                    prev_interact_vel = vel.copy()
                    continue
                else:
                    self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)

            elif status == 1:  # pushed by user
                if blender_state is not None:
                    blender_state['status'] = 1
                if np.linalg.norm(interaction_heading) > 0 > np.dot(vel, interaction_heading):
                    logger.info("Ignoring interaction: Direction change > 90 degrees.")
                    interact_vel = np.array([0.0, 0.0, 0.0])
                    speed = 0
                else:
                    interact_vel = vel

                dv_lb = interact_vel - prev_interact_vel
                v_virtual = interact_vel + dv_lb * (mass_ratio - 1)
                prev_interact_vel = interact_vel.copy()
                target_pos = pos + v_virtual * dt * v_scalar

                if not detect_speed_threshold(speed):
                    interaction_heading = np.zeros(3)
                    v_virtual = np.zeros(3)
                    prev_interact_vel = np.zeros(3)
                    if fric_coe > 0:
                        logger.info(f"Switching to Coasting From {status}.")
                        if self.set_color:
                            self.set_color([255, 255, 0])
                        status = 2
                        continue
                    else:
                        logger.info(f"Switching to Grace Hover From {status}.")
                        hover_pos = pos + interact_vel * dt * v_scalar * 5
                        status = 3

                        tilt_angle = calculate_tilt(current_roll, current_pitch)
                        grace_time = get_grace_time(abs(tilt_angle), speed)
                        log_data = {
                            "speed": round(speed, 3),
                            "vel": [round(x, 3) for x in vel],
                            "Pos": [round(x, 3) for x in pos],
                            "Target": [round(x, 3) for x in hover_pos],
                            "Stabilize Time": grace_time
                        }
                        if self.set_color:
                            self.set_color([255, 255, 0])
                        self._log_event("User Disengage", log_data)
                        continue

                if base_attitude < 0:
                    log_data = {
                        "speed": round(speed, 3),
                        "vel": [round(x, 3) for x in vel],
                        "heading": [round(x, 3) for x in interaction_heading],
                        "Pos": [round(x, 3) for x in pos],
                        "Target": [round(x, 3) for x in target_pos]
                    }
                    self._log_event("User Pushing", log_data)
                    self.lo_commander.send_position_setpoint(target_pos[0], target_pos[1], target_pos[2], 0)
                else:
                    is_decelerating = np.dot(dv_lb, interact_vel) < 0
                    if is_decelerating:
                        # When decelerating, output a given value (defaulting to 0.0)
                        given_decel_value = 0.0
                        target_pitch, target_roll = given_decel_value, given_decel_value
                    else:
                        target_pitch, target_roll = calculate_braking_angles(*dv_lb[:2], yaw_deg=current_yaw)

                    log_data = {
                        "speed": round(speed, 3),
                        "vel": [round(x, 3) for x in vel],
                        "heading": [round(x, 3) for x in interaction_heading],
                        "Pos": [round(x, 3) for x in pos],
                        "Target_Attitude": [round(target_pitch, 3), round(target_roll, 3)],
                    }
                    self._log_event("User Pushing", log_data)
                    yaw_rate_cmd = max(min(-5.0 * current_yaw, 50.0), -50.0)
                    self.lo_commander.send_zdistance_setpoint(target_roll, target_pitch, yaw_rate_cmd, target_pos[2])

            elif status == 2:  # coasting
                if blender_state is not None:
                    blender_state['status'] = 2
                end_pos, coast_t = self.calculate_coasting(pos, vel, fric_coe)

                self.lo_commander.send_notify_setpoint_stop()
                self.hl_commander.go_to(end_pos[0], end_pos[1], end_pos[2], 0, coast_t, relative=False)
                self._safe_sleep(coast_t)
                logger.info(f"Switching to Hover From {status}.")
                hover_pos = end_pos
                if self.set_color:
                    self.set_color([255, 157, 0])
                status = 0
                continue

            elif status == 3:  # grace period
                if blender_state is not None:
                    blender_state['status'] = 3

                grace_start = time.time()

                self.lo_commander.send_notify_setpoint_stop()
                self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, grace_time, relative=False)

                while time.time() < grace_time + grace_start:
                    self._safe_sleep(dt)

                if self.set_color:
                    self.set_color([255, 157, 0])
                status = 0
                if blender_state is not None:
                    blender_state['status'] = 0
                continue

            if blender_state is not None and blender_state['sending_positions']:
                now_t = time.time()
                if now_t - last_blender_send_time >= 0.1:
                    send_pos = hover_pos if status == 0 else list(pos)
                    send_blender_position(send_pos)
                    last_blender_send_time = now_t
                    if status == 0 and blender_state['finish_requested']:
                        blender_state['sending_positions'] = False
                        blender_state['finish_requested'] = False
                        blender_state['stop_at_next_zero'] = False
                        blender_state['edit_active'] = False
                        logger.info("Sent final hover position, stopped streaming.")

            self._safe_sleep(dt)

        if blender_state is not None:
            worker_done.set()

        self.lo_commander.send_notify_setpoint_stop()

    def interaction_peer_translation_vel(
            self,
            drone_id,
            vel_threshold=0.01,
            z=1,
            fric_coe=-1.0,
            base_attitude=1,
            duration=60,
            grace_time=1,
            v_scalar=None,
            pub_socket=None,
            sub_socket=None,
    ):
        """Symmetric peer interaction: every drone can be pushed and mirrors others.

        When this drone detects a user push it broadcasts per-step offsets to peers.
        When it receives a push message from a peer it applies the offset to its own
        hover position. If both happen simultaneously the push with the latest
        push_start_time wins; the loser reverts to its pre-push hover position.
        """
        if v_scalar is None:
            v_scalar = np.array([10, 10, 2])
        else:
            v_scalar = np.array(v_scalar)

        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01

        if isinstance(grace_time, (int, float)):
            get_grace_time = lambda a, v: grace_time
            stabilize_time = grace_time
        else:
            import joblib
            saved_poly_data = joblib.load(grace_time)
            poly_transformer = saved_poly_data['poly']
            loaded_model = saved_poly_data['model']
            get_grace_time = lambda a, v: loaded_model.predict(
                poly_transformer.transform([[a, v]])
            )[0] + 0.2
            stabilize_time = 'dynamic'

        grace_time_val = grace_time if isinstance(grace_time, (int, float)) else 2.0

        def drain_sub(duration):
            if sub_socket is None:
                return None
            else:
                # monitoring stays active, while still reacting immediately to peer messages
                start_time = time.time()
                while time.time() - start_time < duration:
                    self._safe_sleep(0)
                    msg = sub_socket.recv_latest()
                    if msg is not None:
                        return msg
                return None

        def detect_speed_threshold(s):
            return s > vel_threshold

        def calculate_braking_angles(v_x, v_y, yaw_deg=0.0, base_att=base_attitude):
            yaw_rad = np.radians(yaw_deg)
            cos_y = np.cos(yaw_rad)
            sin_y = np.sin(yaw_rad)
            body_v_x = v_x * cos_y + v_y * sin_y
            body_v_y = -v_x * sin_y + v_y * cos_y

            pitch = np.sign(body_v_x) * base_att
            roll = -np.sign(body_v_y) * base_att
            pitch = max(min(pitch, 20), -20)
            roll = max(min(roll, 20), -20)
            return pitch, roll

        while True:
            try:
                last_pos = self._get_latest_pos()
                break
            except Exception:
                time.sleep(0.001)

        if z is not None:
            hover_pos = np.array([last_pos[0], last_pos[1], z], dtype=float)
        else:
            hover_pos = last_pos.copy().astype(float)

        self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, 2)
        self._safe_sleep(2)

        logger.info("Starting Peer Interaction mode...")
        self._log_event('Waiting For User Interaction')

        self.log_manager.add_log_entry(
            group_name="configs",
            entry={'delta_v': vel_threshold, 'Delta': dt, 'Stabilize Time': stabilize_time},
            name='Peer Config',
        )

        status = 0
        push_start_time = None
        hover_pos_before_push = None
        interaction_heading = np.zeros(3)
        # Accumulated displacement (target_pos - hover_pos_before_push) sent to peers
        accumulated_offset = np.zeros(3)
        # Receiver side: own hover position at the moment the peer's push began
        peer_hover_start = None
        peer_push_start_time = None
        # Suppress local interaction detection while a peer is actively pushing
        receiving_peer_push = False
        # Time of last received peer push msg (for timeout detection)
        last_peer_push_time = None
        # Time when follower entered grace (after peer user_disengage), None if not in grace
        peer_grace_start = None

        start_time = time.time()
        while time.time() - start_time < duration:

            peer_msg = drain_sub(dt)

            state = self._get_latest_drone_state()
            if not state:
                state = {}

            current_pitch = state.get('stateEstimate.pitch', 0.0)
            current_roll = state.get('stateEstimate.roll', 0.0)
            current_yaw = state.get('stateEstimate.yaw', 0.0)

            pos, vel = self._get_latest_pos(vel=True)
            self.check_interaction_boundary(pos)
            if z is not None:
                pos[2] = z
                vel[2] = 0.0

            speed = np.linalg.norm(vel)
            if status == 0:
                if receiving_peer_push:
                    if peer_grace_start is not None:
                        # Follower grace: hold position until leader signals done or timer expires
                        if peer_msg and peer_msg.get('type') == 'grace_done':
                            leader_id = peer_msg.get('drone_id')
                            self._log_event("Peer Grace Done Received", {
                                "leader_id": leader_id,
                                "Pos": [round(x, 3) for x in pos],
                            })
                            receiving_peer_push = False
                            peer_grace_start = None
                            last_peer_push_time = None
                            peer_push_start_time = None
                            peer_hover_start = None
                        elif time.time() - peer_grace_start > grace_time_val:
                            logger.info("Peer mode: follower grace timeout — resuming detection.")
                            receiving_peer_push = False
                            peer_grace_start = None
                            last_peer_push_time = None
                            peer_push_start_time = None
                            peer_hover_start = None
                        else:
                            self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                    else:
                        # Active following
                        if peer_msg and peer_msg.get('type') == 'push':
                            leader_id = peer_msg.get('drone_id')
                            if peer_msg['push_start_time'] != peer_push_start_time:
                                peer_push_start_time = peer_msg['push_start_time']
                                peer_hover_start = hover_pos.copy()
                                self._log_event("Peer Push Received", {
                                    "leader_id": leader_id,
                                    "push_start_time": peer_push_start_time,
                                    "Pos": [round(x, 3) for x in pos],
                                })
                                if self.set_color:
                                    self.set_color([0, 255, 0])
                            last_peer_push_time = time.time()
                            accumulated = np.array(peer_msg['accumulated_offset'])
                            if z is not None:
                                accumulated[2] = 0.0
                            hover_pos = peer_hover_start + accumulated
                            if z is not None:
                                hover_pos[2] = z
                            self.check_interaction_boundary(hover_pos)
                            self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                            self._log_event("Peer Pushing", {
                                "leader_id": leader_id,
                                "push_start_time": peer_push_start_time,
                                "accumulated_offset": accumulated.tolist(),
                                "Pos": [round(x, 3) for x in pos],
                                "Target": [round(x, 3) for x in hover_pos],
                            })
                        elif peer_msg and peer_msg.get('type') == 'user_disengage':
                            leader_id = peer_msg.get('drone_id')
                            self._log_event("Peer Disengage Received", {
                                "leader_id": leader_id,
                                "Pos": [round(x, 3) for x in pos],
                            })
                            peer_grace_start = time.time()
                            self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                        elif last_peer_push_time is not None and time.time() - last_peer_push_time > grace_time_val:
                            # No push for too long — give up following
                            logger.info("Peer mode: no push received — giving up following.")
                            receiving_peer_push = False
                            last_peer_push_time = None
                            peer_push_start_time = None
                            peer_hover_start = None
                        else:
                            self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                else:
                    if peer_msg and peer_msg.get('type') == 'push':
                        leader_id = peer_msg.get('drone_id')
                        peer_push_start_time = peer_msg['push_start_time']
                        peer_hover_start = hover_pos.copy()
                        last_peer_push_time = time.time()
                        peer_grace_start = None
                        receiving_peer_push = True
                        self._log_event("Peer Push Received", {
                            "leader_id": leader_id,
                            "push_start_time": peer_push_start_time,
                            "Pos": [round(x, 3) for x in pos],
                        })
                        if self.set_color:
                            self.set_color([0, 255, 0])
                        accumulated = np.array(peer_msg['accumulated_offset'])
                        if z is not None:
                            accumulated[2] = 0.0
                        hover_pos = peer_hover_start + accumulated
                        if z is not None:
                            hover_pos[2] = z
                        self.check_interaction_boundary(hover_pos)
                        self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                        self._log_event("Peer Pushing", {
                            "leader_id": leader_id,
                            "push_start_time": peer_push_start_time,
                            "accumulated_offset": accumulated.tolist(),
                            "Pos": [round(x, 3) for x in pos],
                            "Target": [round(x, 3) for x in hover_pos],
                        })
                    elif detect_speed_threshold(speed):
                        logger.info("Peer mode: local user push detected.")
                        if self.set_color:
                            self.set_color([0, 255, 0])
                        status = 1
                        push_start_time = time.time()
                        hover_pos_before_push = hover_pos.copy()
                        accumulated_offset = np.zeros(3)
                        interaction_heading = vel.copy()
                        continue
                    else:
                        self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)

            elif status == 1:
                # Peer push is newer → it wins; revert and follow peer
                if peer_msg and peer_msg.get('type') == 'push':
                    peer_start = peer_msg.get('push_start_time', 0.0)
                    if peer_start >= push_start_time:
                        leader_id = peer_msg.get('drone_id')
                        logger.info("Peer push is newer — abandoning local push, reverting position.")
                        send_time = time.time()
                        pub_socket.send_json({"type": "grace_done", "drone_id": drone_id})
                        self._log_event("User Disengage", {
                            "leader_id": drone_id,
                            "reason": "peer_push_won",
                            "Pos": [round(x, 3) for x in pos],
                            "latency_ms": round((time.time() - send_time) * 1000, 3),
                        })
                        if self.set_color:
                            self.set_color([255, 255, 0])
                        hover_pos = hover_pos_before_push.copy()
                        push_start_time = None
                        hover_pos_before_push = None
                        accumulated_offset = np.zeros(3)
                        interaction_heading = np.zeros(3)
                        status = 0
                        receiving_peer_push = True
                        peer_grace_start = None
                        last_peer_push_time = time.time()
                        peer_push_start_time = peer_start
                        peer_hover_start = hover_pos.copy()
                        self._log_event("Peer Push Received", {
                            "leader_id": leader_id,
                            "push_start_time": peer_push_start_time,
                            "Pos": [round(x, 3) for x in pos],
                        })
                        if self.set_color:
                            self.set_color([0, 255, 0])
                        accumulated = np.array(peer_msg['accumulated_offset'])
                        if z is not None:
                            accumulated[2] = 0.0
                        hover_pos = peer_hover_start + accumulated
                        if z is not None:
                            hover_pos[2] = z
                        self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                        continue

                if np.linalg.norm(interaction_heading) > 0 > np.dot(vel, interaction_heading):
                    interact_vel = np.zeros(3)
                    speed = 0.0
                else:
                    interact_vel = vel

                target_pos = pos + interact_vel * dt * v_scalar
                accumulated_offset = target_pos - hover_pos_before_push

                if not detect_speed_threshold(speed):
                    interaction_heading = np.zeros(3)
                    send_time = time.time()
                    pub_socket.send_json({"type": "user_disengage", "drone_id": drone_id})
                    disengage_latency_ms = round((time.time() - send_time) * 1000, 2)
                    accumulated_offset = np.zeros(3)
                    if fric_coe > 0:
                        logger.info("Peer mode: switching to coasting.")
                        self._log_event("User Disengage", {
                            "leader_id": drone_id,
                            "reason": "coasting",
                            "Pos": [round(x, 3) for x in pos],
                            "latency_ms": disengage_latency_ms,
                        })
                        if self.set_color:
                            self.set_color([255, 255, 0])
                        status = 2
                    else:
                        logger.info("Peer mode: switching to grace hover.")
                        hover_pos = pos + interact_vel * dt
                        status = 3
                        tilt_angle = calculate_tilt(current_roll, current_pitch)
                        grace_time_val = get_grace_time(abs(tilt_angle), speed)
                        self._log_event("User Disengage", {
                            "leader_id": drone_id,
                            "speed": round(speed, 3),
                            "vel": [round(x, 3) for x in vel],
                            "Pos": [round(x, 3) for x in pos],
                            "Target": [round(x, 3) for x in hover_pos],
                            "Stabilize Time": grace_time_val,
                            "latency_ms": disengage_latency_ms,
                        })
                        if self.set_color:
                            self.set_color([255, 255, 0])
                    continue

                send_time = time.time()
                pub_socket.send_json({
                    "type": "push",
                    "drone_id": drone_id,
                    "accumulated_offset": accumulated_offset.tolist(),
                    "push_start_time": push_start_time,
                })
                push_latency_ms = round((time.time() - send_time) * 1000, 2)

                if base_attitude < 0:
                    self._log_event("User Pushing", {
                        "leader_id": drone_id,
                        "speed": round(speed, 3),
                        "vel": [round(x, 3) for x in vel],
                        "Pos": [round(x, 3) for x in pos],
                        "Target": [round(x, 3) for x in target_pos],
                        "latency_ms": push_latency_ms,
                    })
                    self.lo_commander.send_position_setpoint(target_pos[0], target_pos[1], target_pos[2], 0)
                else:
                    target_pitch, target_roll = base_attitude * np.array(
                        calculate_braking_angles(*interact_vel[:2], yaw_deg=current_yaw))
                    self._log_event("User Pushing", {
                        "leader_id": drone_id,
                        "speed": round(speed, 3),
                        "vel": [round(x, 3) for x in vel],
                        "Pos": [round(x, 3) for x in pos],
                        "latency_ms": push_latency_ms,
                    })
                    yaw_rate_cmd = max(min(-5.0 * current_yaw, 50.0), -50.0)
                    self.lo_commander.send_zdistance_setpoint(target_roll, target_pitch, yaw_rate_cmd, target_pos[2])

            elif status == 2:  # coasting
                end_pos, coast_t = self.calculate_coasting(pos, vel, fric_coe)
                self.lo_commander.send_notify_setpoint_stop()
                self.hl_commander.go_to(end_pos[0], end_pos[1], end_pos[2], 0, coast_t, relative=False)
                self._safe_sleep(coast_t)
                hover_pos = np.array(end_pos, dtype=float)
                send_time = time.time()
                pub_socket.send_json({"type": "grace_done", "drone_id": drone_id})
                self._log_event("Grace Done", {"leader_id": drone_id, "Pos": [round(x, 3) for x in hover_pos.tolist()],
                                               "latency_ms": round((time.time() - send_time) * 1000, 2)})
                if self.set_color:
                    self.set_color([227, 253, 255])
                status = 0
                continue

            elif status == 3:  # grace period
                grace_start = time.time()
                self.lo_commander.send_notify_setpoint_stop()
                self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, grace_time_val, relative=False)
                while time.time() < grace_time_val + grace_start:
                    self._safe_sleep(dt)
                send_time = time.time()
                pub_socket.send_json({"type": "grace_done", "drone_id": drone_id})
                self._log_event("Grace Done", {"leader_id": drone_id, "Pos": [round(x, 3) for x in hover_pos.tolist()],
                                               "latency_ms": round((time.time() - send_time) * 1000, 2)})
                if self.set_color:
                    self.set_color([227, 253, 255])
                status = 0

        self.lo_commander.send_notify_setpoint_stop()

    def interaction_follow_network(
            self,
            sub_socket,
            z=1,
            fric_coe=-1.0,
            base_attitude=1,
            duration=60,
            v_scalar=None,
    ):
        """Mirror the interaction drone's push state received over ZMQ.

        alpha_vel is always 1 for followers — velocity comes entirely from the
        network message, not from the drone's own state estimator.
        """
        if v_scalar is None:
            v_scalar = np.array([10, 10, 2])
        else:
            v_scalar = np.array(v_scalar)

        dt = 1.0 / self.ctrl_rate if self.ctrl_rate > 0 else 0.01

        def calculate_braking_angles(v_x, v_y, yaw_deg=0.0, base_att=base_attitude):
            yaw_rad = np.radians(yaw_deg)
            cos_y = np.cos(yaw_rad)
            sin_y = np.sin(yaw_rad)
            body_v_x = v_x * cos_y + v_y * sin_y
            body_v_y = -v_x * sin_y + v_y * cos_y

            pitch = np.sign(body_v_x) * base_att
            roll = -np.sign(body_v_y) * base_att
            pitch = max(min(pitch, 20), -20)
            roll = max(min(roll, 20), -20)
            return pitch, roll

        # Wait for first own position fix
        while True:
            try:
                last_pos = self._get_latest_pos()
                break
            except Exception:
                time.sleep(0.001)

        if z is not None:
            hover_pos = np.array([last_pos[0], last_pos[1], z], dtype=float)
        else:
            hover_pos = np.array([last_pos[0], last_pos[1], last_pos[2]], dtype=float)

        self.hl_commander.go_to(hover_pos[0], hover_pos[1], hover_pos[2], 0, 2)
        self._safe_sleep(2)

        logger.info("Starting Network Follow mode...")

        poller = zmq.Poller()
        poller.register(sub_socket, zmq.POLLIN)

        prev_remote_status = 0
        coasting_triggered = False
        start_time = time.time()

        while time.time() - start_time < duration:
            # Drain the socket and keep only the latest message
            msg = None
            while dict(poller.poll(0)).get(sub_socket):
                try:
                    msg = sub_socket.recv_json(flags=zmq.NOBLOCK)
                except zmq.Again:
                    break

            if msg is not None:
                remote_status = msg.get('status', 0)
                remote_vel = np.array(msg.get('vel', [0.0, 0.0, 0.0]))
            else:
                remote_status = prev_remote_status
                remote_vel = np.zeros(3)

            state = self._get_latest_drone_state() or {}
            current_yaw = state.get('stateEstimate.yaw', 0.0)

            pos = self._get_latest_pos()
            self.check_interaction_boundary(pos)
            if z is not None:
                pos[2] = z
                remote_vel[2] = 0.0

            if remote_status in (0, 3):
                # Interaction drone is hovering/in grace — hold own hover position
                if prev_remote_status == 1:
                    # Transition out of push: update hover to current position
                    hover_pos = pos.copy()
                self.lo_commander.send_position_setpoint(hover_pos[0], hover_pos[1], hover_pos[2], 0)
                coasting_triggered = False

            elif remote_status == 1:
                target_pos = pos + remote_vel * dt * v_scalar
                if base_attitude < 0:
                    self.lo_commander.send_position_setpoint(target_pos[0], target_pos[1], target_pos[2], 0)
                else:
                    target_pitch, target_roll = base_attitude * np.array(
                        calculate_braking_angles(*remote_vel[:2], yaw_deg=current_yaw))
                    yaw_rate_cmd = max(min(-5.0 * current_yaw, 50.0), -50.0)
                    self.lo_commander.send_zdistance_setpoint(target_roll, target_pitch, yaw_rate_cmd, target_pos[2])

            elif remote_status == 2 and not coasting_triggered:
                # Execute coasting once per coasting phase
                end_pos, coast_t = self.calculate_coasting(pos, remote_vel, fric_coe)
                self.lo_commander.send_notify_setpoint_stop()
                self.hl_commander.go_to(end_pos[0], end_pos[1], end_pos[2], 0, coast_t, relative=False)
                hover_pos = np.array(end_pos, dtype=float)
                coasting_triggered = True
                self._safe_sleep(coast_t)

            prev_remote_status = remote_status
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
