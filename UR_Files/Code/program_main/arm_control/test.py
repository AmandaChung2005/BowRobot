import numpy as np
from pathlib import Path
import sys

project_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(project_path))

import arm_control.robot_interface as robot

input("Press Enter to start the test...")

# Get current joint positions
current = robot.getCurrentJoints()

print("Current joints (rad):", current)
print("Current joints (deg):", np.degrees(current))

# Move joint 1 by +1 degree
target = np.degrees(current.copy())
target[0] += 1.0

print("Target joints (deg):", target)

# Test servoJ at 500 Hz
for _ in range(500):

    t_start = robot.initPeriod()

    robot.servoJ(
        target.tolist(),
        1.0,       # acceleration [rad/s^2]
        0.5,       # velocity [rad/s]
        0.002,     # dt [s]
        0.1,       # lookahead time [s]
        300        # gain
    )

    robot.waitPeriod(t_start)

# Stop servoJ
robot.stop()

print("Test complete")