from rtde_control import RTDEControlInterface

arm_ip = "192.168.1.101"
rtde_c = RTDEControlInterface("arm_ip")

# Program Parameters
current_string = "E"  # G, D, A, E
start_pos = "frog"    # frog, middle, tip
start_dir = "downbow"    # upbow, downbow
bowing_cycles = 3

# General Movement Paramters
speed = 0.10        # m/s
acceleration = 0.2  # m/s^2
joint_speed = 1.0
joint_acceleration = 1.2
lift_height = 0.05


# Bowing Parameters
bow_speed = 0.10        # m/s
bow_acceleration = 0.2  # m/s^2
bow_force = -3.0    # N

# Control Parameters
lookahead_time = 0.1
gain = 300
dt = 1.0/500.0

# Force Control
force_type = 2
selection_vector = [1, 0, 1, 0, 0, 0]  # Z-Axis Force Control
wrench = [5, 0, -2.0, 0, 0, 0]  # Desired N of Force
limits = [2, 2, 2, 1, 1, 1]


home_position = []
home_joints = []


# Important Positions (Bow Touching String)
string_paths = {
    "G": {
        "frog": [],
        "middle": [],
        "tip": []
    },
    "D": {
        "frog": [],
        "middle": [],
        "tip": []
    },
    "A": {
        "frog": [],
        "middle": [],
        "tip": []
    },
    "E": {
        "frog": [],
        "middle": [],
        "tip": []
    }
}

task_frames = {  
    string: poses["frog"]
    for string, poses in string_paths.items()
}