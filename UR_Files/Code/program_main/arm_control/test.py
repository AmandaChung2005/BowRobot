import time
import numpy as np
import sys
from pathlib import Path

project_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(project_path))

import config
import arm_control.robot_interface as robot
import arm_control.arm_config as arm_config

print("RTDE connected:", robot.isConnected())

current = robot.getCurrentJoints()

print("Current joints:")
print(np.degrees(current))

target = np.array(arm_config.home_joints, dtype=float)

print("\nHome target:")
print(target)

print("\nMoving to home...")

robot.moveJ(
    target,
    arm_config.joint_speed,
    arm_config.joint_acceleration
)

print("DONE")