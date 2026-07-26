simulation = True    # True = RoboDK, False = Real UR7e
setup = False        # True = Get Coordinates, False = Run Program

arm_ip = "192.168.1.101"
robodk_python_path = r"C:\Users\amand\Documents\RoboDK\Python"
code_path = r"C:\Users\amand\Documents\UROP Summer '26\UR Files\Code"

# Program Parameters
current_string = "E"  # G, D, A, E
start_pos = "middle"    # frog, middle, tip
start_dir = "upbow"    # upbow, downbow
bowing_cycles = 3
rosin_cycles = 1

# General Movement Paramters
speed = 0.1        # m/s
acceleration = 0.1  # m/s^2
joint_speed = 0.5
joint_acceleration = 0.1
lift_height = 3
step_xyz = 5
step_rot = 10

# Bowing Parameters
bow_speed = 0.10        # m/s
bow_acceleration = 0.2  # m/s^2
bow_force = -3.0    # N

# Rosin Paramters
rosin_hover = 50
rosin_selection_vector = [0, 0, 1, 0, 0, 0]
rosin_wrench = [0, 0, -5, 0, 0, 0]
rosin_limits = [2, 2, 2, 1, 1, 1]

# Spiccato Parameters
spiccato_length = 20
spicacto_height = 5
spiccato_frequency = 4
spiccato_force = 3
spiccato_cycles = 10

# Control Parameters
lookahead_time = 0.1
gain = 300
dt = 1.0/500.0

# Force Control
force_type = 2
selection_vector = [1, 0, 1, 0, 0, 0]  # Z-Axis Force Control
wrench = [5, 0, -2.0, 0, 0, 0]  # Desired N of Force
limits = [2, 2, 2, 1, 1, 1]


home_position = [-561.800, -23.698, -50.975, -127.279, 127.279, 0]
home_joints = [0.00, -90.00, -60.00, -120.00, 90.00, -90.00]

# Calibration
import calibration_data

if simulation:
    data = calibration_data.simulation
else:
    data = calibration_data.real

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