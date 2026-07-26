import numpy as np
import config
import calibration_data as cal

if config.simulation:
    cal = cal.simulation
else:
    cal = cal.real

# Lift Pose
def lift_pose(pose, height):
    pose = np.array(pose, dtype=float)
    lift = pose.copy()

    lift[2] += height
    return lift.tolist()

# Basic Up and Down Bows
def basic():
    current_path = cal["string_paths"][config.current_string]

    if config.simulation:
        frog_pose = current_path["frog"]
        middle_pose = current_path["middle"]
        tip_pose = current_path["tip"]
    else:
        frog_pose = np.array(current_path["frog"], dtype=float)
        middle_pose = np.array(current_path["middle"], dtype=float)
        tip_pose = np.array(current_path["tip"], dtype=float)   

    if config.start_pos == "frog":

        basic_cartesian_path = [frog_pose, tip_pose, frog_pose]
        joint_names = ["frog", "tip", "frog"]
        
    elif config.start_pos == "tip":

        basic_cartesian_path = [tip_pose, frog_pose, tip_pose]
        joint_names = ["tip", "frog", "tip"]
        
    elif config.start_pos == "middle":

        if config.start_dir == "upbow":

            basic_cartesian_path = [middle_pose, frog_pose, tip_pose, middle_pose]
            joint_names = ["middle", "frog", "tip", "middle"]
            
        elif config.start_dir == "downbow":

            basic_cartesian_path = [middle_pose, tip_pose, frog_pose, middle_pose]
            joint_names = ["middle", "tip", "frog", "middle"]

        else:
            raise ValueError("start_dir must be 'upbow' or 'downbow'")
        
    else:
        raise ValueError("start_pos must be 'frog', 'middle', or 'tip'")
    
    basic_joint_path = [
        np.array(
            cal["joint_paths"][config.current_string][name],
            dtype=float
        )
        for name in joint_names
    ]

    return basic_cartesian_path, basic_joint_path

# Richochet Bowing

# Spiccato Bowing
def spiccato():
    current_path = config.string_paths[config.current_string]

    frog = current_path["frog"]
    middle = current_path["middle"]
    tip = current_path["tip"]

    lift_height = config.spicacto_height
    lift_frog = lift_pose(frog, lift_height)
    lift_middle = lift_pose(middle, lift_height)
    lift_tip = lift_pose(tip, lift_height)

    cartesian_path = [
        frog,
        middle,
        tip,
        lift_tip,
        lift_middle,
        lift_frog
    ]

    return cartesian_path
