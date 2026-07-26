import numpy as np
from pprint import pformat
import sys

import config
import calibration_data as cal

# Read Robot

if config.simulation:
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
    from robodk.robomath import Pose_2_TxyzRxyz

    RDK = Robolink()
    robot = RDK.Item('', ITEM_TYPE_ROBOT)

    joints = robot.Joints().list()
    pose = Pose_2_TxyzRxyz(robot.Pose())

else:
    import rtde_receive

    rtde_r = rtde_receive.RTDEReceiveInterface(config.arm_ip)


while True:
    if config.simulation:
        joints = robot.Joints().list()
        pose = Pose_2_TxyzRxyz(robot.Pose())
    else:
        joints = np.degrees(rtde_r.getActualQ()).tolist()
        pose = rtde_r.getActualTCPPose()

        pose[0] *= 1000
        pose[1] *= 1000
        pose[2] *= 1000
        pose[3:] = np.degrees(pose[3:])
        pose = pose.tolist()

    # Choose Calibration
    if config.simulation:
        data = cal.simulation
        section = "simulation"
    else:
        data = cal.real
        section = "real"

    # Select Calibration Type
    while True:
        calibration_type = input(
            "\nCalibration type (rosin/violin): ").strip().lower()

        if calibration_type in ("rosin", "violin"):
            break

        print("Invalid input, try again")

    # Where to Save
    if calibration_type == "rosin":
        valid_positions_rosin = {"frog", "tip"}

        while True:
            target = input("\nPosition (frog, tip): ").strip().lower()

            if target in valid_positions_rosin:
                break

            print("Invalid posiiton, try again")

        data["rosin_position"][target] = pose
        data["rosin_joints"][target] = joints

    else:
        valid_positions_violin = {"hover", "frog", "middle", "tip"}

        while True:
            target = input("\nPosition (hover, frog, middle, tip): ".strip().lower())

            if target in valid_positions_violin:
                break

            print("Invalid position, try again")

        if target == "hover":
            data["violin_hover_position"] = pose
            data["violin_hover_joints"] = joints

        else:
            valid_strings = {"G", "D", "A", "E"}

            while True:
                string = input("String (G, D, A, E): ").strip().upper()

                if string in valid_strings:
                    break

                print ("Invalid string, try again")

            data["string_paths"][string][target] = pose
            data["joint_paths"][string][target] = joints

    
    # Update Calibration File
    with open("calibration_data.py", "w") as f:

        f.write("# RoboDK Calibration\n")
        f.write("simulation = ")
        f.write(pformat(cal.simulation, sort_dicts=False))

        f.write("\n\n")

        f.write("# Real UR7e Calibration\n")
        f.write("real = ")
        f.write(pformat(cal.real, sort_dicts=False))

    print()

    if calibration_type == "rosin":
        print(f"{section.capitalize()} rosin {target} calibration updated.")

    elif target == "hover":
        print (f"{section.capitalize()} violin hover calibration updated.")

    else:
        print(f"{section.capitalize()} violin calibration updated for {string} {target}.")

    # Repeat?
    while True:
        again = input("\n Take another calibration? (y/n): ").strip().lower()

        if again in ("y", "yes"):
            break
        if again in ("n", "no"):
            print("Calibration Complete")
            sys.exit()
        print("Invalid input, try again")