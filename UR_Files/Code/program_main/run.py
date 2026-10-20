import sys
from pathlib import Path

project_path = Path(__file__).resolve().parent
sys.path.insert(0, str(project_path))

import config

# Program Parameters
config.simulation = False   # True = RoboDK, False = Real UR7e
config.setup = False      # True = Get Coordinates, False = Run Program
config.keyboard = False
config.beep = True
config.monitorForce = True
config.avoidObstacles = False

import arm_control.arm_config as arm_config


arm_config.useForce = False
arm_config.current_string = "E"   # G, D, A, E
arm_config.start_pos = "frog"     # frog, middle, tip
arm_config.start_dir = "upbow"    # upbow, downbow

arm_config.bowing_cycles = 3
arm_config.rosin_cycles = 1

arm_config.manual_force_pid = {
    "speed_mm_s": 1.0,
    "tolerance": 0.1,
    "control_period": 0.02,
    "max_adjustment_mm": 3.0,
    "acceleration": 0.05,

    "kp": 0.001,
    "ki": 0.00005,
    "kd": 0.0001,

    "integral_limit": 5.0,
    "derivative_filter_alpha": 0.2,
    "force_direction": -1.0
}

# Run Program
print("Python Program Started")
print("Connecting to Robot...")

if config.keyboard:
    import arm_control.keyboard_control
    sys.exit()

if config.setup:
    print("\nStarting Calibration...")
    import arm_control.calibration
    sys.exit()

else:
    import arm_control.robot_interface_new as robot
    if config.simulation:
        print("Simulation Mode: ", robot.isConnected())
    else:
        print("RTDE Connected: ", robot.isConnected())
    print("\nStarting Main Program")
    import arm_control.main



