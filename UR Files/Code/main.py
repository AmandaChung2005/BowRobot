# RoboDK
import sys
sys.path.append(r"C:\Users\amand\Documents\UROP Summer '26\UR Files\Code")

from pynput.keyboard import Key, Listener
import time
import numpy as np
from scipy.spatial.transform import Rotation
import config
import calibration_data as cal

if config.simulation:
    cal = cal.simulation
else:
    cal = cal.real

import robot_interface as robot
from robodk.robomath import Pose_2_TxyzRxyz, TxyzRxyz_2_Pose

# Keyboard Control
continue_motion = False
waiting = False
halt = False

def pressed(key):
    global waiting, halt

    if key == Key.delete:
        halt = True
        waiting = False
        print ("Program Terminated")
        listener.stop()
        sys.exit()
    if waiting and key == Key.enter:
            waiting = False
            return
    if waiting:
        return

    
def wait_for_enter(message):
    global waiting
    continute_motion = False

    print(message)
    print("Press Enter to continue")
    print("Press DELETE to stop")
    waiting = True

    while waiting:
        if halt:
            robot.stop()
            sys.exit()
        time.sleep(0.01)

listener = Listener(on_press = pressed)
listener.start()

wait_for_enter("Program Started")


# Movement Limits
current_path = cal["string_paths"][config.current_string]

if config.simulation:
    frog_pose = current_path["frog"]
    middle_pose = current_path["middle"]
    tip_pose = current_path["tip"]
else:
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

# Joint Paths
joint_path = [
    np.array(cal["joint_paths"][config.current_string][name], dtype=float)
    for name in (
        ["frog", "tip", "frog"]
        if config.start_pos == "frog"
        else ["tip", "frog", "tip"]
        if config.start_pos == "tip"
        else(
            ["middle", "frog", "tip", "middle"]
            if config.start_dir == "upbow"
            else ["middle", "tip", "frog", "middle"]
        )
    )
]

# Movement Function
def bow_segment(start_pose, end_pose, start_joints, end_joints):
    if config.simulation:
        if halt:
            robot.stop()
            sys.exit()
        end_pose = np.array(end_pose, dtype=float)

        robot.moveL(
            end_pose.tolist(),
            config.bow_speed,
            config.bow_acceleration
        )

    else:
        for alpha in np.linspace(0.0, 1.0, 500):
            if halt:
                robot.stop()
                sys.exit()
            
            t_start = robot.initPeriod()
       
            pose = (1-alpha)*start_pose+alpha*end_pose

            robot.servoL(
                pose.tolist(),
                config.bow_speed,
                config.bow_acceleration,
                config.dt,
                config.lookahead_time,
                config.gain
            )

            robot.forceMode(
                cal["task_frames"],
                config.selection_vector,
                config.wrench,
                config.force_type,
                config.limits
            )
        
            robot.waitPeriod(t_start)

# Move to Home
wait_for_enter("Move to Home")

robot.moveJ(
    config.home_joints,
    config.joint_speed,
    config.joint_acceleration
)

# Move to Above String
wait_for_enter("Move Above String")

robot.moveJ(
    cal["hover_joints"],
    config.speed,
    config.acceleration
)

# Move onto String Position
wait_for_enter("Move Onto String")          

robot.moveJ(
    cal["joint_paths"][config.current_string][config.start_pos],
    config.speed,
    config.acceleration
)

# Basic Bowing
wait_for_enter("Start Bowing")

for _ in range(config.bowing_cycles):
    for i in range(len(bow_path)-1):

        bow_segment(
            bow_path[i],
            bow_path[i+1],
            joint_path[i],
            joint_path[i+1]
        )

# Stops Motion
robot.stop()

# Move Bow Out of the Way
wait_for_enter("Lift Bow")

robot.moveJ(
    cal["hover_joints"],
    config.speed,
    config.acceleration
)

# Return to Home
wait_for_enter("Return Home")

robot.moveJ(
    config.home_joints,
    config.joint_speed,
    config.joint_acceleration
)
