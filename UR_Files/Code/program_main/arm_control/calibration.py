import numpy as np
from pprint import pformat
import sys

import config
import arm_control.calibration_data as cal
import arm_control.arm_config as arm_config

# Read Robot

if config.simulation:
    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
    from robodk.robomath import Pose_2_TxyzRxyz

    RDK = Robolink()
    robot = RDK.Item(config.robotName, ITEM_TYPE_ROBOT)

    rtde_r = None

else:
    import rtde_receive
    rtde_r = rtde_receive.RTDEReceiveInterface(config.host_ip)

    robot = None

def get_calibration_data():
    if config.simulation:
        return cal.simulation, "simulation"

    return cal.real, "real"

def get_robot_position():
    if config.simulation:
        joints = robot.Joints().list()
        pose = list(Pose_2_TxyzRxyz(robot.Pose()))
        pose[3:] = np.degrees(pose[3:]).tolist()

        return joints, pose

    else:
        joints = np.degrees(rtde_r.getActualQ()).tolist()
        pose = list(rtde_r.getActualTCPPose())
        pose[:3] = [
            value * 1000.0
            for value in pose[:3]
        ]
        pose[3:] = np.degrees(pose[3:]).tolist()
        

        return joints, pose


def get_tcp_position():
    if config.simulation:
        pose = np.array(
            Pose_2_TxyzRxyz(robot.Pose()),
            dtype=float
        )

        return pose[:3]

    pose = np.array(
        rtde_r.getActualTCPPose(),
        dtype=float
    )

    pose[:3] *= 1000.0
    return pose[:3]

def save_calibration():
    with open(
        cal.__file__,
        "w"
    ) as f:
        f.write("# RoboDK Calibration\n")
        f.write("simulation = ")
        f.write(pformat(cal.simulation, sort_dicts=False))
        f.write("\n\n")
        f.write("# Real UR7e Calibration\n")
        f.write("real = ")
        f.write(pformat(cal.real, sort_dicts=False))

def confirm_calibration(
        section,
        calibration_type,
        target,
        joints,
        pose,
        string = None
):
    print("\nCalibration to Save")
    print(f"Mode: {section.capitalize()}")
    print(f"Type: {calibration_type.capitalize()}")

    if calibration_type == "rosin":
        print(f"Position: {target}")
    elif calibration_type == "violin":
        if target == "hover":
            print("Position: Hover")
        else:
            print(f"String: {string}")
            print(f"Position: {target}")

    print("\nJoints:")
    print(np.round(joints, 3))

    print("\nPose:")
    print(np.round(pose, 3))

    while True:
        confirm = input(
            "\nSave This Calibration?\n"
            " [Y] Yes\n"
            " [N] No, Discard It\n"
        ).strip().lower()

        if confirm in {"y", "yes"}:
            return True
        
        if confirm in {"n", "no"}:
            print("Calibration Discarded")
            return False
        
        print("Invalid Input, Try Again")

def repeat_calibration(calibration_type):
    while True:
        again = input(
            f"\nTake Another {calibration_type.capitalize()} Calibration?\n"
            " [Y] Yes\n"
            " [N] No, Return to Calibration Menu\n"
            "> "
        ).strip().lower()

        if again in ("y", "yes"):
            return True
        if again in ("n", "no"):
            return False
        print("Invalid input, try again")

def select_rosin_position():
    positions = {
        "f": "frog",
        "frog": "frog",
        "t": "tip",
        "tip": "tip"
    }

    while True:
        target = input(
            "\nSelect Rosin Position:\n"
            " [F] Frog\n"
            " [T] Tip\n"
            " [B] Back to Calibration Menu\n"
        ).strip().lower()

        if target in {"b", "back"}:
            return None

        if target in positions:
            return positions[target]

        print("Invalid Input, Try Again")

def select_violin_position():
    positions = {
        "h": "hover",
        "hover": "hover",
        "f": "frog",
        "frog": "frog",
        "t": "tip",
        "tip": "tip"
    }

    while True:
        target = input(
            "\nSelect Rosin Position:\n"
            " [H] Hover\n"
            " [F] Frog\n"
            " [T] Tip\n"
            " [B] Back to Calibration Menu\n"
        ).strip().lower()

        if target in {"b", "back"}:
            return None

        if target in positions:
            return positions[target]

        print("Invalid Input, Try Again")

def select_string():
    while True:
        string = input(
            "\nSelect String:\n"
            " [G] G String\n"
            " [D] D String\n"
            " [A] A String\n"
            " [E] E String\n"
            " [B] Back to Calibration Menu\n"
        ).strip().upper()

        if string in {"B", "BCL"}:
            return None

        if string in {"G", "D", "A", "E"}:
            return string

        print("Invalid Input, Try Again")

def calibrate_rosin(data, section):
    while True:
        target = select_rosin_position()

        if target is None:
            return

        joints, pose = get_robot_position()

        if not confirm_calibration(
            section,
            "rosin",
            target,
            joints,
            pose
        ):
            continue

        data["rosin_position"][target] = pose
        data["rosin_joints"][target] = joints

        save_calibration()

        print(
            f"\n{section.capitalize()} "
            f"Rosin {target} Calibration Updated"
        )

        if not repeat_calibration("rosin"):
            return

def calibrate_violin(data, section):
    while True:
        target = select_violin_position()

        if target is None:
            return

        string = None

        if target != "hover":
            string = select_string()

            if string is None:
                continue

        joints, pose = get_robot_position()

        if not confirm_calibration(
            section,
            "violin",
            target,
            joints,
            pose,
            string
        ):
            continue

        if target == "hover":
            data["violin_hover_position"] = pose
            data["violin_hover_joints"] = joints

            print(
                f"\n{section.capitalize()} "
                f"Violin Hover Calibration Updated"
            )
            
        else:
            data["string_paths"][string][target] = pose
            data["joint_paths"][string][target] = joints

            print(
                f"\n{section.capitalize()} "
                f"Violin Calibration Updated for "
                f"the {string} string at {target}"
            )

        save_calibration()

        if not repeat_calibration("violin"):
            return

def measure_corner(prompt):
    while True:
        user_input = input(
            f"\n{prompt}\n"
            "Press ENTER when the TCP is in Position\n"
            " [B] Back\n"
            "> "
        ).strip().lower()

        if user_input in {"b", "back"}:
            return None

        position = get_tcp_position()

        print(
            "\nTCP Position (mm):",
        )

        print(
            np.round(position, 3)
        )

        confirm = input(
            "Use This Point? [Y/N]: "
        ).strip().lower()

        if confirm in {"y", "yes"}:
            return position

        print("Point Discarded. Measure Again")

def measure_obstacle():
    corner_1 = measure_corner(
        "Move TCP to the FIRST Corner"
    )

    if corner_1 is None:
        return None

    corner_2 = measure_corner(
        "Move TCP to the OPPOSITE Corner of the Same Face"
    )

    if corner_2 is None:
        return None

    corner_3 = measure_corner(
        "Move TCP to the Corresponding Corner at the Other Z Level"
    )

    if corner_3 is None:
        return None

    length_x = abs(
        corner_2[0] - corner_1[0]
    )

    length_y = abs(
        corner_2[1] - corner_1[1]
    )

    length_z = abs(
        corner_3[2] - corner_1[2]
    )

    dimensions = np.array(
        [
            length_x,
            length_y,
            length_z
        ],
        dtype=float
    )

    minimum = np.minimum(
        corner_1,
        corner_3
    )

    maximum = minimum + dimensions

    obstacle = {
        "min": np.round(minimum, 3).tolist(),
        "max": np.round(maximum, 3).tolist()
    }

    print("\nMeasured Obstacle Dimensions: ")
    print(f"X = {length_x:.3f} mm")
    print(f"Y = {length_y:.3f} mm")
    print(f"Z = {length_z:.3f} mm")

    print("\nObstacle Box:")
    print("Min:", obstacle["min"])
    print("Max:", obstacle["max"])

    return obstacle

def calibrate_obstacles(data, section):
    while True:
        obstacle = measure_obstacle()
        if obstacle is None:
            return

        while True:
            confirm = input(
                "\nSave This Obstacle?\n"
                " [Y] Yes\n"
                " [N] No, Discard It\n"
                "> "
            ).strip().lower()

            if confirm in {"y", "yes"}:
                break

            if confirm in {"n", "no"}:
                print("Obstacle Discarded")
                break

            print("Invalid Input, Try Again")

        if confirm in {"n", "no"}:
            continue

        if "obstacles" not in data:
            data["obstacles"] = []

        data["obstacles"].append(obstacle)

        save_calibration()

        print("\nObstacle Saved Successfully!")

        print(f"Mode: {section.capitalize()}")

        while True:
            again = input(
                "\nTake Another Obstacle Calibation?\n"
                " [Y] Yes\n"
                " [N] No\n"
                "> "
            ).strip().lower()

            if again in {"y", "yes"}:
                break

            if again in {"n", "no"}:
                print("Obstacle Calibration Complete")
                return

            print("Invalid Input, Try Again")


# Main Program
while True:
    data, section = get_calibration_data()

    while True:
        calibration_type = input(
                "\nSelect Calibration Type:\n"
                " [R] Rosin\n"
                " [V] Violin\n"
                " [O] Obstacles\n"
                " [E] Exit Program\n"
                "> "
            ).strip().lower()

        if calibration_type in {
            "rosin", "r",
            "violin", "v",
            "obstacles", "obstacle", "o",
            "exit", "e"
        }:
            break

        print("Invalid Input, Try Again")

    # Where to Save
    if calibration_type in {"e", "exit"}:
        print("Calibration Complete")
        sys.exit()

    if calibration_type in {"r", "rosin"}:
        calibrate_rosin(data, section)
        continue

    if calibration_type in {"v", "violin"}:
        calibrate_violin(data, section)
        continue

    if calibration_type in {"o", "obstacle", "obstacles"}:
        calibrate_obstacles(data, section)
        continue

    print("Invalid Input, Try Again")