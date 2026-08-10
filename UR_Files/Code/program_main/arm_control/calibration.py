import numpy as np
from pprint import pformat
import sys
import arm_control.arm_config as arm_config
import config
import user_interface as user
import calibration_data as cal

# Read Robot

if config.simulation:
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
    from robodk.robomath import Pose_2_TxyzRxyz

    RDK = Robolink()
    robot = RDK.Item(config.robotName, ITEM_TYPE_ROBOT)

    joints = robot.Joints().list()
    pose = Pose_2_TxyzRxyz(robot.Pose())

else:
    import rtde_receive

    rtde_r = rtde_receive.RTDEReceiveInterface(config.arm_ip)


while True:
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
                "\nSelect Calibration Type:\n"
                " [R] Rosin\n"
                " [V] Violin\n"
                "> "
            ).strip().lower()

        if calibration_type in {"rosin", "r", "violin", "v"}:
            break

        print("Invalid Input, Try Again")

    # Where to Save
    if calibration_type in {"r", "rosin"}:
        valid_positions_rosin = {"frog", "f", "tip", "t"}

        while True:
            target = input(
                "\nSelect Rosin Position:\n"
                " [F] Frog\n"
                " [T] Tip\n"
                " [B] Back to Calibration Type\n"
                "> "
                ).strip().lower()

            if target in ("back", "b"):
                break

            if target in valid_positions_rosin:
                position_map = {
                    "f": "frog",
                    "t": "tip"
                }

                target = position_map.get(target, target)

                break

            print("Invalid Input, Try Again")

        if target in ("back", "b"):
            continue

    else:
        valid_positions_violin = {
            "hover", "h",
            "frog", "f",
            "tip", "t"
            }

        while True:
            target = input(
                "\nSelect Violin Position:\n"
                " [H] Hover\n"
                " [F] Frog\n"
                " [T] Tip\n"
                " [B] Back to Calibration Type\n"
                "> "
                ).strip().lower()

            if target in ("back", "b"):
                break

            if target not in valid_positions_violin:
                print("Invalid Input, Try Again")
                continue

            position_map = {
                    "h": "hover",
                    "f": "frog",
                    "t": "tip"
                }

            target = position_map.get(target, target)

            if target == "hover":
                break

            valid_strings = {"G", "D", "A", "E"}

            while True:
                string = input(
                    "\nSelect String:\n"
                    " [G] G String\n"
                    " [D] D String\n"
                    " [A] A String\n"
                    " [E] E String\n"
                    " [B] Back to Position Selection\n"
                    "> "
                    ).strip().upper()

                if string.lower() in ("back", "b"):
                    break

                if string not in valid_strings:
                    print ("Invalid string, try again")
                    continue

                break

            if string.lower() in ("back", "b"):
                continue

            break

        if target in ("back", "b"):
            continue

    # Read Robot Position
    if arm_config.simulation:
        joints = robot.Joints().list()
        pose = Pose_2_TxyzRxyz(robot.Pose())
    else:
        joints = np.degrees(rtde_r.getActualQ()).tolist()
        pose = rtde_r.getActualTCPPose()

        pose[0] *= 1000
        pose[1] *= 1000
        pose[2] *= 1000
        pose [3:] = np.degrees(pose[3:])
        pose = pose.tolist()

    # Confirmation
    print("\n")
    print("Calibration to Save")
    print(f"Mode: {section.capitalize()}")

    if calibration_type in {"r", "rosin"}:
        print("Type: Rosin")
        print(f"Position: {target}")
    else:
        print("Type: Violin")

        if target == "hover":
            print("Position: Hover")
        else:
            print(f"String: {string}")
            print(f"Position: {target}")

    print("\nJoints: ")
    print(np.round(joints, 3))

    print("\nPose: ")
    print(np.round(pose, 3))

    while True:
        confirm = input(
            "\nSave This Calibration?\n" 
            " [Y] Yes\n"
            " [N] No, Discard It\n"
            "> "
        ).strip().lower()

        if confirm in ("y", "yes"):
            break
        if confirm in ("n", "no"):
            print("Calibration Discarded")
            break
    if confirm in ("n", "no"):
        continue

    # Save Robot Position
    if calibration_type in {"r", "rosin"}:
        data["rosin_position"][target] = pose
        data["rosin_joints"][target] = joints

    else:
        if target == "hover":
            data["violin_hover_position"] = pose
            data["violin_hover_joints"] = joints
        else:
            data["string_paths"][string][target] = pose
            data["joint_paths"][string][target] = joints
        
    # Update Calibration File
    with open(cal.__file__, "w") as f:

        f.write("# RoboDK Calibration\n")
        f.write("simulation = ")
        f.write(pformat(cal.simulation, sort_dicts=False))

        f.write("\n\n")

        f.write("# Real UR7e Calibration\n")
        f.write("real = ")
        f.write(pformat(cal.real, sort_dicts=False))

    print("\nCalibration Saved Successfully!")

    if calibration_type in {"r", "rosin"}:
        print(f"{section.capitalize()} Rosin {target} calibration updated")

    elif target == "hover":
        print (f"{section.capitalize()} Violin hover calibration updated")

    elif calibration_type in {"v", "violin"}:
        print(f"{section.capitalize()} Violin calibration updated for the {string} string, at the {target}")

    # Repeat?
    while True:
        again = input(
                "\nTake Another Calibration:\n"
                " [Y] Yes\n"
                " [N] No, Exit Program\n"
                "> "
            ).strip().lower()

        if again in ("y", "yes"):
            break
        if again in ("n", "no"):
            print("Calibration Complete")
            sys.exit()
        print("Invalid input, try again")