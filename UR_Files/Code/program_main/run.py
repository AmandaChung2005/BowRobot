# Settings
simulation = False   # True = RoboDK, False = Real UR7e
setup = False        # True = Get Coordinates, False = Run Program

import sys
from pathlib import Path

import config


project_path = Path(__file__).resolve().parent
sys.path.insert(0, str(project_path))

print("Python Program Started")

if config.setup:
    print("Starting Calibration...")
    import arm_control.calibration
else:
    print("Connecting to RTDE...")

    import arm_control.robot_interface as robot

    print("RTDE Connected, Watchdog Running")
    print("Press PLAY on Polyscope")
    input("Press ENTER after Polyscope is running: ")

    import arm_control.main



