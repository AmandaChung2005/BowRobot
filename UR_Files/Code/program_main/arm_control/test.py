import sys
from pathlib import Path

project_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(project_path))

import arm_control.robot_interface_new as robot

input("Press Enter to start the test...")

robot.servoJ_trajectory(
    joint=3,
    amplitude_deg=5.0,
    half_duration=3.0,
    dt=0.002,
    lookahead_time=0.1,
    gain=300
)

print("Test complete")