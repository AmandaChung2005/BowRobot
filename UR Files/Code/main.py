from pynput.keyboard import Key, Listener
import time
import numpy as np
from scipy.spatial.transform import Rotation
from rtde_control import RTDEControlInterface
import config

# Movement Limits
current_path = config.string_paths[config.current_string]

frog_pose = np.array(current_path["frog"], dtype=float)
middle_pose = np.array(current_path["middle"], dtype=float)
tip_pose = np.array(current_path["tip"], dtype=float)

# Bow Paths
if config.start_pos == "frog":
    bow_path = [
        frog_pose,
        tip_pose,
        frog_pose
        ]
elif config.start_pos == "tip":
    bow_path = [
        tip_pose,
        frog_pose,
        tip_pose
        ]
elif config.start_pos == "middle":
    if config.start_dir == "upbow":
        bow_path = [
            middle_pose,
            frog_pose,
            tip_pose,
            middle_pose
            ]
    elif config.start_dir == "downbow":
        bow_path = [
            middle_pose,
            tip_pose,
            frog_pose,
            middle_pose
            ]
    else:
        raise ValueError("start_dir must be 'upbow' or 'downbow'")
    
else:
    raise ValueError("start_pos must be 'frog', 'middle', or 'tip'")

# Movement Function
def bow_segment(start_pose, end_pose):
    for alpha in np.linspace(0.0, 1.0, 500):
        t_start = config.rtde_c.initPeriod()
        pose = (1-alpha)*start_pose+alpha*end_pose

        config.rtde_c.servoL(
            pose.tolist(),
            config.bow_speed,
            config.bow_acceleration,
            config.dt,
            config.lookahead_time,
            config.gain
        )

        config.rtde_c.forceMode(
            config.task_frame,
            config.selection_vector,
            config.wrench,
            config.force_type,
            config.limits
            )
        
        config.rtde_c.waitPeriod(t_start)

# Move to Home
config.rtde_c.moveJ(
    config.home_joints,
    config.joint_speed,
    config.joint_acceleration
)

# Move to Above String
hover_pose = bow_path[0].copy()
hover_pose[2] += config.lift_height

config.rtde_c.moveL(
    hover_pose.tolist(),
    config.speed,
    config.acceleration
)

# Move onto String Position
config.rtde_c.moveL(
    bow_path[0].tolist(),
    config.bow_speed,
    config.bow_acceleration
)           

# Basic Bowing
for _ in range(config.bowing_cycles):
    for i in range(len(bow_path) - 1):
        bow_segment(
            bow_path[i],
            bow_path[i+1]
        )

# Stops Motion
config.rtde_c.servoStop()
config.rtde_c.forceModeStop()

# Move Bow Out of the Way
hover_pose = config.rtde_c.getActualTCPPose()
hover_pose[2] += config.lift_height
config.rtde_c.moveL(
    hover_pose,
    config.speed,
    config.acceleration
)

# Return to Home
config.rtde_c.moveJ(
    config.home_joints,
    config.joint_speedspeed,
    config.joint_acceleration
)
