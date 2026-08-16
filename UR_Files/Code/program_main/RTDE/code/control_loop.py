#!/usr/bin/env python
# Copyright (c) 2016-2022, Universal Robots A/S,
# All rights reserved.
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Universal Robots A/S nor the names of its
#      contributors may be used to endorse or promote products derived
#      from this software without specific prior written permission.
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL UNIVERSAL ROBOTS A/S BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE

import sys
import logging
import time
import threading
import numpy as np
from pathlib import Path

program_main_path = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(program_main_path))

import config
import arm_control.arm_config as arm_config

rtde_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(rtde_path))

import rtde.rtde as rtde
import rtde.rtde_config as rtde_config

class RTDEInterface:
    # Motion Commands
    mode_idle = 0
    mode_movel = 1
    mode_movej = 2
    mode_servoj = 3

    # Force Mode
    force_off = 0
    force_on = 1

    def __init__(self):
        self.robot_host = config.host_ip
        self.robot_port = 30004

        self.con = None
        self.setp = None
        self.watchdog = None

        self.state_names = None
        self.state_types = None

        self.setp_names = None
        self.setp_types = None

        self.watchdog_names = None
        self.watchdog_types = None

        self.move_completed = True
        self.move_started = False

        self.base_dir = Path(__file__).resolve().parent
        self.config_filename = self.base_dir/"control_loop_configuration.xml"

        self.lock = threading.Lock()

        self.heartbeat_thread = None
        self.heartbeat_stop = threading.Event()
        self.heartbeat_hz = 50.0

    # Connection
    def connect(self):
        logging.getLogger().setLevel(logging.INFO)

        print(f"Connecting to {self.robot_host}:{self.robot_port}...")

        conf = rtde_config.ConfigFile(
            str(self.config_filename)
        )

        self.state_names, self.state_types = conf.get_recipe("state")
        self.setp_names, self.setp_types = conf.get_recipe("setp")
        self.watchdog_names, self.watchdog_types = conf.get_recipe("watchdog")

        print("Connecting to Robot")
        print(f"Robot: {self.robot_host}:{self.robot_port}")

        self.con = rtde.RTDE(
            self.robot_host,
            self.robot_port
        )

        self.con.connect()

        print(f"Connected to Robot at {self.robot_host}")

        version = self.con.get_controller_version()
        print(f"Controller Version: {version}")
        
        self.con.send_output_setup(
            self.state_names,
            self.state_types
        )

        self.setp = self.con.send_input_setup(
            self.setp_names,
            self.setp_types
        )

        for i in range(36):
            setattr(
                self.setp,
                f"input_double_register_{i}",
                0.0
            )

        self.setp.input_int_register_2 = int(arm_config.force_type)

        self.watchdog = self.con.send_input_setup(
            self.watchdog_names,
            self.watchdog_types
        )

        self.watchdog.input_int_register_0 = self.mode_idle
        self.watchdog.input_int_register_1 = self.force_off
        self.watchdog.input_int_register_2 = int(arm_config.force_type)
        

        if not self.con.send_start():
            self.con.disconnect()
            self.con = None
            raise RuntimeError("Failed to Start RTDE")

        print("RTDE Started")

        with self.lock:
            if not self.con.send(self.watchdog):
                raise RuntimeError("Failed to Send Initial Watchdog")

        self.start_heartbeat()
        print("Initial Watchdog Sent")

    def start_heartbeat(self):
        self.heartbeat_stop.clear()

        self.heartbeat_thread = threading.Thread(
            target = self.heartbeat_loop,
            daemon = True
        )
        self.heartbeat_thread.start()

    def heartbeat_loop(self):
        period = 1.0 / self.heartbeat_hz

        while not self.heartbeat_stop.is_set():
            try:
                if self.con is not None:
                    with self.lock:
                        result = self.con.send(self.watchdog)
                        if not result:
                            print(f"Heartbeat Error: {e}")
            except Exception as e:
                print("========== HEARTBEAT ERROR ==========")
                print("Type:", type(e).__name__)
                print("Error:", repr(e))
                print("=====================================")
                time.sleep(0.1)

            time.sleep(period)
                
    def stop_heartbeat(self):
        self.heartbeat_stop.set()

        if self.heartbeat_thread is not None:
            self.heartbeat_thread.join(timeout = 1.0)
            self.heartbeat_thread = None

    # RTDE Communication
    def receive(self):
        if self.con is None:
            raise RuntimeError("RTDE Isn't Connected")

        return self.con.receive()

    def send_motion_command(self, mode = None):
        if self.con is None:
            raise RuntimeError("RTDE Isn't Connected")

        with self.lock:
            if mode is not None:
                self.watchdog.input_int_register_0 = int(mode)
            if not self.con.send(self.watchdog):
                raise RuntimeError("Failed to Send Watchdog")

    def set_force_mode(self, enabled):
        if self.con is None:
            raise RuntimeError("RTDE Isn't Connected")

        with self.lock:
            if enabled:
                self.watchdog.input_int_register_1 = self.force_on
                print("RTDE: Force Mode ON")

            else:
                self.watchdog.input_int_register_1 = self.force_off
                print("RTDE: Force Mode OFF")

            if not self.con.send(self.watchdog):
                raise RuntimeError("Failed to Send Force Mode State")

    def set_force_parameters(
            self,
            task_frame,
            selection_vector,
            wrench,
            force_type,
            limits
    ):
        if len(task_frame) != 6:
            raise ValueError("Task Frame Must Contain 6 Values")

        if len(selection_vector) != 6:
            raise ValueError("Selection Vecotr Must Contain 6 Values")

        if len(wrench) != 6:
            raise ValueError("Wrench Must Contain 6 Values")

        if len(limits) != 6:
            raise ValueError("Limits Must Contain 6 Values")

        for i in range(6):
            setattr(
                self.setp,
                f"input_double_register_{12+i}",
                float(task_frame[i])
            )

        for i in range(6):
            setattr(
                self.setp,
                f"input_double_register_{18+i}",
                float(selection_vector[i])
            )

        for i in range(6):
            setattr(
                self.setp,
                f"input_double_register_{24+i}",
                float(wrench[i])
            )

        for i in range(6):
            setattr(
                self.setp,
                f"input_double_register_{30+i}",
                float(limits[i])
            )

        self.setp.input_int_register_2 = int(force_type)

        with self.lock:
            if not self.con.send(self.setp):
                raise RuntimeError("Failed to Send Force Parameters")

        print("RTDE: Force Parameters Updated")
        
# Robot State
    def getActualTCPPose(self):
        state = self.receive()

        if state is None:
            raise RuntimeError("Couldn't Receive Robot State")

        return list(state.actual_TCP_pose)

    def getActualQ(self):
        state = self.receive()

        if state is None:
            raise RuntimeError("Couldn't Receive Robot State")

        return list(state.actual_q)

    # Setpoints
    def set_motion_parameters(
            self,
            acceleration,
            velocity,
            dt,
            lookahead_time,
            gain
    ):
        self.setp.input_double_register_6 = float(velocity)
        self.setp.input_double_register_7 = float(acceleration)
        self.setp.input_double_register_8 = float(dt)
        self.setp.input_double_register_9 = float(lookahead_time)
        self.setp.input_double_register_10 = float(gain)

        self.setp.input_double_register_11 = 0.0

    
    def set_pose(self, pose):
        if len(pose) != 6:
            raise ValueError("TCP Pose Must Contain 6 Values")

        for i in range(6):
            setattr(
                self.setp,
                f"input_double_register_{i}",
                float(pose[i])
            )

    def set_joints(self, joints):
        if len(joints) != 6:
            raise ValueError("Joint Position Must Contain 6 Values")

        for i in range(6):
            setattr(
                self.setp,
                f"input_double_register_{i}",
                float(joints[i])
            )

    def get_setpoint(self):
        return [
            getattr(
                self.setp,
                f"input_double_register_{i}"
            )
            for i in range(6)
        ]

    # Movement
    def moveJ(
            self,
            joints,
            speed,
            acceleration
    ): 
        if len(joints) != 6:
            raise ValueError("moveJ Requires 6 Joint Values")

        target = np.array(joints, dtype = float)
        current = np.array(self.getActualQ(), dtype = float)

        target_rad = np.deg2rad(target)

        difference_deg = np.abs(
            np.degrees(current) - target
        )

        if np.all(difference_deg < 1.0):
            print("Already at Target Joing Position")
            self.move_completed = True
            self.move_started = False
            return

        self.move_completed = False
        self.move_started = False

        self.set_joints(target_rad.tolist())

        self.set_motion_parameters(
            speed,
            acceleration,
            0,
            0,
            0
        )

        with self.lock:
            self.watchdog.input_int_register_0 = self.mode_idle

            if not self.con.send(self.watchdog):
                raise RuntimeError("Failed to Reset Motion Mode")

        time.sleep(0.1)

        with self.lock:
            if not self.con.send(self.setp):
                raise RuntimeError("Failed to Send moveJ Setpoint")

            self.watchdog.input_int_register_0 = self.mode_movej

            if not self.con.send(self.watchdog):
                raise RuntimeError("Failed to Send moveJ Setpoint")

        print("moveJ Command Sent")

        self.wait_for_move()

           
    def moveL(
            self,
            pose,
            speed,
            acceleration
    ): 
        if len(pose) != 6:
            raise ValueError("moveL Requires a 6-Value TCP Pose")

        target = np.array(pose, dtype = float)

        current = np.array(self.getActualTCPPose(), dtype = float)

        position_error = np.linalg.norm(
            current[:3] - target[:3]
        )

        orientation_error = np.linalg.norm(
            current[3:] - target[3:]
        )

        if(
            position_error < 0.002
            and orientation_error < np.deg2rad(2.0)
        ):
            print("Already at Target TCP Pose")

            self.move_completed = True
            self.move_started = False

            return

        self.move_completed = False
        self.move_started = False

        self.set_pose(target.tolist())

        self.set_motion_parameters(
            speed,
            acceleration,
            0,
            0,
            0
        )

        with self.lock:
            self.watchdog.input_int_register_0 = self.mode_idle

            if not self.con.send(self.watchdog):
                raise RuntimeError("Failed to Reset Motion Mode")

        time.sleep(0.1)

        with self.lock:
            if not self.con.send(self.setp):
                raise RuntimeError("Failed to Send moveL Command")

        with self.lock:
            self.watchdog.input_int_register_0 = self.mode_movel

            if not self.con.send(self.watchdog):
                raise RuntimeError("Failed to Send moveL Command")

        print("moveL Command Sent")
        self.wait_for_move()

    def servoJ(
            self,
            joints,
            acceleration,
            velocity,
            dt,
            lookahead_time,
            gain
    ): 
        if len(joints) != 6:
            raise ValueError("servoJ Requires 6 Joint Values")

        joints_rad = np.deg2rad(joints, dtype = float)

        self.set_joints(joints_rad.tolist())

        self.set_motion_parameters(
            acceleration,
            velocity,
            dt,
            lookahead_time,
            gain
        )

        with self.lock:
            self.watchdog.input_int_register_0 = self.mode_servoj

            if not self.con.send(self.setp):
                raise RuntimeError("Failed to Send servoJ Setpoint")

            if not self.con.send(self.watchdog):
                raise RuntimeError("Failed to Send servoJ Command")

    
    # Timing
    def initPeriod(self):
        return time.perf_counter()

    def waitPeriod(self, t_start):
        elapsed = time.perf_counter() - t_start
        period = 1.0 / 500.0

        remaining = period - elapsed

        if remaining > 0:
            time.sleep(remaining)

    # Stop
    def servoStop(self):
        print("RTDE: Motion Stop")

        self.watchdog.input_int_register_0 = self.mode_idle

        with self.lock:
            if not self.con.send(self.watchdog):
                raise RuntimeError("Failed to Stop Motion")

    def forceModeStop(self):
        print("RTDE: Force Mode Stop") 

        self.watchdog.input_int_register_1 = self.force_off

        with self.lock:
            if not self.con.send(self.watchdog):
                raise RuntimeError("Failed to Disable Force Mode")

    def stop(self):
        self.servoStop()
        self.forceModeStop()
       

    # Monitoring Movement
    def update(self):
        if self.con is None:
            raise RuntimeError("RTDE Isn't Connected")

        state = self.receive()

        if state is None:
            print("No State Received")
            return False

        if not self.move_started:
            if state.output_int_register_0 == 1:
                print("Move Started")
                self.move_started = True

            return False

        if state.output_int_register_0 == 0:
            print("Move Completed")

            self.move_completed = True
            self.move_started = False

            with self.lock:
                self.watchdog.input_int_register_0 = self.mode_idle

                if not self.con.send(self.watchdog):
                    raise RuntimeError("Failed to Send Watchdog")

            time.sleep(0.1)

            return True

        return False

    def wait_for_move(self):
        while not self.move_completed:
            self.update()
            time.sleep(0.001)

    # Utilities
    def isConnected(self):
        return self.con is not None

    def reconnect(self):
        if self.isConnected():
            return True

        try:
            self.connect()
            return True

        except Exception as e:
            print(f"Reconnect Failed: {e}")
            return False
    
    def disconnect(self):
        self.heartbeat_stop.set()
        if self.con is not None:
            try:
                self.watchdog.input_int_register_0 = self.mode_idle
                self.watchdog.input_int_register_1 = self.force_off
                with self.lock:
                    self.con.send(self.watchdog)
            except Exception:
                pass
            try: 
                self.con.send_pause()
            except Exception:
                pass
            try:
                self.con.disconnect()
            except Exception:
                pass

            self.con = None
            print("RTDE Disconnected")