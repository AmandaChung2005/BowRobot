#!/usr/bin/env python
# Copyright (c) 2016, Universal Robots A/S,
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
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

import sys
import logging
from connector import RTDEConnect
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))
sys.path.append('..')

import config

# Use 127.0.0.1 when testing against PolyScope X in Docker.
# Use 192.168.0.12 for the real robot on the LAN cable.
ROBOT_HOST = config.arm_ip
# ROBOT_HOST = '192.168.0.12'
ROBOT_PORT = 30004

from pathlib import Path
base_dir = Path(__file__).resolve().parent

config_filename = base_dir/"control_loop_configuration.xml"

keep_running = True

logging.getLogger().setLevel(logging.INFO)


def setp_to_list(output):
    setp = [output.input_double_register_0, output.input_double_register_1, output.input_double_register_2,
            output.input_double_register_3, output.input_double_register_4, output.input_double_register_5]
    #set_list = [format(elem, '.2f') for elem in setp]

    #return [float(x) for x in set_list]
    # Users running 5.11.5 or later can simply return "setp" instead of set_list.
    return setp


# control loop
monitor = RTDEConnect(ROBOT_HOST, config_filename)

setp1 = [-0.316, -0.475, 0.15, 2.94, 0.0, 0.0]
setp2 = [-0.316, -0.475, 0.25, 2.94, 0.0, 0.0]
while keep_running:
    # receive the current state
    state = monitor.receive()
    if state is None:
        break

    # The UR program requests a new setpoint through integer output register 0.
    if state.output_int_register_0 != 0:
        new_setp = setp1 if setp_to_list(state) == setp2 else setp2
        monitor.sendall("setp", new_setp)

    # kick watchdog
    monitor.send("watchdog", "input_int_register_0", 0)

monitor.shutdown()
