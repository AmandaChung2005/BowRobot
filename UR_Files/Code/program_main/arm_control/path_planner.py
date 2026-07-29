import numpy as np
import config
import calibration_data as cal
import robot_interface as robot

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
def basic(current_string, start_pos):
    current_path = cal["string_paths"][current_string]

    if config.simulation:
        frog_pose = current_path["frog"]
        middle_pose = current_path["middle"]
        tip_pose = current_path["tip"]
    else:
        frog_pose = np.array(current_path["frog"], dtype=float)
        middle_pose = np.array(current_path["middle"], dtype=float)
        tip_pose = np.array(current_path["tip"], dtype=float)   

    if start_pos == "frog":

        basic_cartesian_path = [frog_pose, tip_pose, frog_pose]
        joint_names = ["frog", "tip", "frog"]
        
    elif start_pos == "tip":

        basic_cartesian_path = [tip_pose, frog_pose, tip_pose]
        joint_names = ["tip", "frog", "tip"]
        
    elif start_pos == "middle":

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
            cal["joint_paths"][current_string][name],
            dtype=float
        )
        for name in joint_names
    ]

    return basic_cartesian_path, basic_joint_path

# Richochet Bowing

# Spiccato Bowing
def spiccato(current_string):
    current_path = config.data["string_paths"][current_string]

    frog = np.array(current_path["frog"], dtype = float)
    middle = np.array(current_path["middle"], dtype = float)
    tip = np.array(current_path["tip"], dtype = float)

    frog_middle_distance = np.linalg.norm(middle[:3] - frog[:3])

    offset = min(config.spiccato_offset, frog_middle_distance)
    alpha = offset/frog_middle_distance

    start = (1-alpha) * frog + alpha * middle

    stroke = min(config.spiccato_length, frog_middle_distance-offset)
    beta = stroke/frog_middle_distance

    end = start.copy()
    end[:3] += beta * (middle[:3] - frog[:3])


    # Spiccato Around Middle
    # half_length = config.spiccato_length/2

    # start = middle.copy()
    # start[:3] -= direction * half_length

    # end = middle.copy()
    # end[:3] += direction * half_length

    lift_start = np.array(
        lift_pose(start, config.spiccato_height),
        dtype = float
    )

    lift_end = np.array(
        lift_pose(end, config.spiccato_height),
        dtype = float
    )

    spiccato_cartesian_path = [
        lift_start,
        start,
        end,
        lift_end,
        end,
        start,
        lift_start
    ]

    spiccato_joint_path = [
        robot.solveIK(pose)
        for pose in spiccato_cartesian_path
    ]

    return spiccato_cartesian_path, spiccato_joint_path
