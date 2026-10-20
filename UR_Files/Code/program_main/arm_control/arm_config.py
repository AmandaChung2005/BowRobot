from pathlib import Path

def save_bow_loaded(value):
    global bow_loaded

    bow_loaded = bool(value)
    path = __file__

    with open(path, "r", encoding = "utf-8") as file:
        contents = file.read()

    import re
    new_line = f"bow_loaded = {bow_loaded}"
    contents = re.sub(
        r"^bow_loaded\s*=.*$",
        new_line,
        contents,
        count = 1,
        flags = re.MULTILINE
    )

    with open(path, "w", encoding = "utf-8") as file:
        file.write(contents)

# Program Parameters (set in "run.py")
current_string = ""
start_pos = ""
start_dir = ""
bow_loaded = None
bowing_cycles = 3
rosin_cycles = 1

# General Movement Parameters
speed = 0.1                # m/s
acceleration = 0.1         # m/s^2
joint_speed = 0.5          # rad/s
joint_acceleration = 0.1   # rad/s^2
lift_height = 3            # mm
step_xyz = 5               # mm
step_rot = 10              # deg

# Bowing Parameters
bow_speed = 0.05         # mm/s
bow_acceleration = 0.1   # rad/s^2
bow_force = 1.0         # N
force_constant = -1.0    # > 0 Increases force applied
force_gain_scaling = 1.0
force_damping = 0.7
force_update_step_mm = 5.0
force_prepare_speed = 0.01
force_z_speed_limit = 0.002
force_prepare_timeout = 5.0
force_tolerance = 0.1
force_stop_register = 24


# Rosin Parameters
rosin_hover = 50     # mm
rosin_selection_vector = [0, 0, 1, 0, 0, 0]
rosin_wrench = [
    0, 0, -2,   # N
    0, 0, 0     # Nm
]
rosin_limits = [
    2, 2, 2,    # m/s
    1, 1, 1     #rad/s
] 

# Spiccato Parameters
spiccato_offset = 100    # mm from frog
spiccato_length = 20     # mm
spiccato_height = 5      # mm
spiccato_frequency = 4   # Hz
spiccato_force = 3       # N
spiccato_cycles = 10

# Control Parameters
lookahead_time = 0.1   # s
gain = 300             # unitless
dt = 1.0/500.0         # s

# Force Control
useForce = None
force_type = 2
max_force = 10
selection_vector = [0, 0, 1, 0, 0, 0]  # Z-Axis Force Control
wrench = [
    0, 0, 1,  # N
    0, 0, 0    # Nm
]
limits = [
    0.05, 0.05, force_z_speed_limit,   # m/s
    1, 1, 1    # rad/s
] 

# PID Control
manual_force_pid = {
    "speed_mm_s": None,
    "tolerance": None,
    "control_period": None,
    "max_adjustment_mm": None,
    "acceleration": None,

    "kp": None,
    "ki": None,
    "kd": None,

    "integral_limit": None,
    "derivative_filter_alpha": None,
    "force_direction": None
}

# Collision Avoidance Points
import numpy as np

bow_length = 750           # mm  

collision_radius = 50      # mm
avoidance_distance = 100   # mm
waypoint_offset = 100      # mm
sample_spacing = 50.0
tcp_radius = 50

# Calibration
import arm_control.calibration_data as calibration_data

import config

if config.simulation:
    data = calibration_data.simulation
else:
    data = calibration_data.real

home_position = [-491.950, 133.300, 287.900, 180.000, 0.000, 0.000]          # mm
home_joints = [180.00, -90.00, -90.00, -90.00, 90.00, 90.00]         # deg

load_position = [-491.950, 432.900, 597.500, -90.000, 0.000, 0.000]  # mm
load_joints = [180.00, -90.00, -90.00, -90.000, 0.00, 90.00]        # deg

violin_hover_position = data["violin_hover_position"]
violin_hover_joints = data["violin_hover_joints"]

rosin_position = data["rosin_position"]
rosin_joints = data["rosin_joints"]

string_paths = data["string_paths"]
joint_paths = data["joint_paths"]


task_frame = {}
