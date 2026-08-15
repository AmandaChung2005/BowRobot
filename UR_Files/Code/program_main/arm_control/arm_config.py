# Program Parameters
current_string = "E"   # G, D, A, E
start_pos = "frog"     # frog, middle, tip
start_dir = "upbow"    # upbow, downbow
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
bow_speed = 0.10         # m/s
bow_acceleration = 0.2   # m/s^2
bow_force = -3.0         # N

# Rosin Parameters
rosin_hover = 50                             # mm
rosin_selection_vector = [0, 0, 1, 0, 0, 0]
rosin_wrench = [
    0, 0, 2,   # N
    0, 0, 0     # Nm
]
rosin_limits = [
    2, 2, 2,    # m/s
    1, 1, 1     #rad/s
] 


# rosin_fraction > 0.865 < 0.87

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
force_type = 2
selection_vector = [1, 0, 1, 0, 0, 0]  # Z-Axis Force Control
wrench = [
    0, 0, 1,  # N
    0, 0, 0    # Nm
]
limits = [
    2, 2, 2,   # m/s
    1, 1, 1    # rad/s
] 

# Calibration
import arm_control.calibration_data as calibration_data
import config

if config.simulation:
    data = calibration_data.simulation
    home_position = [439.398, -133.300, 484.025, 0.000, 180.000, 0.000]   # mm
    home_joints = [0.00, -90.00, -60.00, -120.00, 90.00, 90.00]           # deg
else:
    data = calibration_data.real
    home_position = [-492.7, 132.7, 489.5, 180.52, 0.19, -0.06]           # mm
    home_joints = [180.00, -90.00, -90.00, -90.00, 90.00, 90.00]          # deg

violin_hover_position = data["violin_hover_position"]
violin_hover_joints = data["violin_hover_joints"]

rosin_position = data["rosin_position"]
rosin_joints = data["rosin_joints"]

string_paths = data["string_paths"]
joint_paths = data["joint_paths"]


task_frames = {
    string: poses["frog"]
    for string, poses in string_paths.items()
}

rosin_task_frame = rosin_position["tip"]