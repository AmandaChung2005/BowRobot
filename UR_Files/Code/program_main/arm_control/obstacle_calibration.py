import sys
import numpy as np
from pprint import pformat
from scipy.spatial.transform import Rotation

import config
import arm_control.calibration_data as cal
import arm_control.arm_config as arm_config

if config.simulation:
    from robodk.robolink import(
        Robolink,
        ITEM_TYPE_ROBOT
    )

    from robodk.robomath import(
        Pose_2_TxyzRxyz
    )

    RDK = Robolink()

    robot = RDK.Item(
        config.robotName,
        ITEM_TYPE_ROBOT
    )

    rtde_r = None

else:
    import rtde_receive

    rtde_r = (
        rtde_receive.RTDEReceiveInterface(
            config.host_ip
        )
    )

    robot = None


# Read TCP Pose
def get_tcp_pose():
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

        position = get_tcp_pose()

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

def save_calibration():
    with open(
        cal.__file__,
        "w"
    ) as f:
        f.write("# RoboDK Calibration\n")
        f.write("simulation = ")
        f.write(
            pformat(
                cal.simulation,
                sort_dicts = False
            )
        )
        f.write(
            "\n\n"
        )
        f.write("# Real UR7e Calibration\n")
        f.write("real = ")
        f.write(pformat(
            cal.real,
            sort_dicts=False
        ))

def calibrate_obstacles():
    if config.simulation:
        data = cal.simulation
        section = "simulation"

    else:
        data = cal.real
        section = "real"

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
