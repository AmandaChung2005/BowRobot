import sys
from pathlib import Path
import numpy as np

project_path = Path(__file__).resolve().parent
sys.path.insert(0, str(project_path))

import config

print("Python Program Started")

if config.setup:
    print("\nStarting Calibration...")
    import arm_control.calibration
    sys.exit()

else:
    import arm_control.robot_interface_new as robot
    print("RTDE Connected: ", robot.isConnected())

    if not config.simulation:
        print("RTDE Heartbeat Started")
   
        print("\nPress PLAY on Polyscope")
        input("Press ENTER after Polyscope is running: ")

    print("\nStarting Main Program")
    import arm_control.main



