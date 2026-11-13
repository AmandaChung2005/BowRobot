# RoboDK
import sys
import time
import threading

import numpy as np
from scipy.spatial.transform import Rotation

import config
import arm_control.arm_config as arm_config
import arm_control.path_planner as path
import arm_control.robot_interface_new as robot
import arm_control.user_interface as user


if config.simulation:
    from robodk.robomath import Pose_2_TxyzRxyz

cal = arm_config.data

# Start User Interface
user.start_interface()
print("Simple Bowing Program Started")

if not robot.is_remote_control():
    print("\n" + "=" * 60)
    print("Error: Polyscope Isn't in Remote Control Mode")
    print("=" * 60)
    print("\nPlease Switch Polyscope to Remote Control Mode")
    print("Then Restart the Program")
    print()
    sys.exit()

# Move to Home
user.beep()
user.wait_for_enter("Move to Home")

robot.moveJ_safe(
    arm_config.home_joints,
    arm_config.joint_speed,
    arm_config.joint_acceleration,
    arm_config.home_position
)

time.sleep(0.02)

at_violin_hover = False

while True:
    if not at_violin_hover:
        user.beep()
        user.wait_for_enter("Move Above String")

        moved_to_hover = robot.moveJ_safe(
            cal["violin_hover_joints"],
            arm_config.joint_speed,
            arm_config.joint_acceleration,  
            cal["violin_hover_position"]
        )

        if moved_to_hover is False:
            print("Could not safely reach violin hover position")
            continue

        at_violin_hover = True

    current_string = user.select_string()
    if current_string is None:
        continue

    while True:
        start_pos = user.select_start_position()
        if start_pos is None:
            break

        start_dir = None

        if start_pos == "middle":
            start_dir = user.select_start_direction()
            if start_dir is None:
                continue
            break

    if start_pos is None:
        continue

    frog_values = cal["string_paths"][current_string]["frog"]
    frog_pose = np.asarray(
        frog_values,
        dtype=float
    ).flatten()

    if frog_pose.shape != (6,) or not np.all(np.isfinite(frog_pose)):
        print(
            f"The {current_string} string needs a valid frog pose "
            "calibration."
        )
        continue

    bow_poses = robot.make_simple_bow_poses(
        frog_pose=frog_pose,
        stroke_length_mm=arm_config.stroke_length_mm,
        stroke_y_direction=arm_config.stroke_y_direction,
        x_offset_mm=arm_config.tcp_x_offset_mm,
        z_offset_mm=arm_config.tcp_z_offset_mm,
        pitch_deg=arm_config.stroke_pitch_deg
    )

    bow_path = robot.make_simple_bow_path(
        bow_poses=bow_poses,
        start_pos=start_pos,
        start_dir=start_dir
    )

    print("\n=== SIMPLE BOW PATH ===")
    print(f"String: {current_string}")
    print(f"Start position: {start_pos}")
    if start_dir is not None:
        print(f"Start direction: {start_dir}")
    print(f"Cycles: {arm_config.bowing_cycles}")
    print(f"Stroke length: {arm_config.stroke_length_mm:.2f} mm")
    print(f"TCP-local Y direction: {arm_config.stroke_y_direction:+.0f}")
    print(f"TCP-local X offset: {arm_config.tcp_x_offset_mm:.2f} mm")
    print(f"Tip TCP-local Z correction: {arm_config.tcp_z_offset_mm:.2f} mm")
    print(f"Additional path pitch: {arm_config.stroke_pitch_deg:.2f} deg")

    print("\nPath poses (mm / degrees):")
    for index, pose in enumerate(bow_path):
        print(
            f"Point {index}: "
            f"X={pose[0]:.2f}, "
            f"Y={pose[1]:.2f}, "
            f"Z={pose[2]:.2f}"
        )

    user.beep()
    user.wait_for_enter(
        "Move to selected bow start position"
    )

    approach_pose = robot.move_to_bow_start(
        hover_pose=cal["violin_hover_position"],
        start_pose=bow_path[0],
        approach_height_mm=arm_config.approach_height_mm,
        speed=arm_config.simple_approach_speed,
        acceleration=arm_config.simple_approach_acceleration
    )

    user.beep()
    user.wait_for_enter("Move Onto String")

    robot.moveL(
        bow_path[0],
        arm_config.approach_speed,
        arm_config.approach_acceleration
    )

    user.beep()
    user.wait_for_enter("Start Bowing")

    for cycle in range(arm_config.bowing_cycles):
        user.check_halt()

        print(
            f"\n=== BOWING CYCLE "
            f"{cycle + 1}/{arm_config.bowing_cycles} ==="
        )

        for index in range(1, len(bow_path)):
            user.check_halt()

            robot.moveL(
                bow_path[index],
                arm_config.simple_bow_speed,
                arm_config.simple_bow_acceleration
            )

            print(f"Segment {index} complete")

    user.beep()
    user.wait_for_enter("Lift Bow")

    robot.moveL(
        approach_pose.tolist(),
        arm_config.speed,
        arm_config.acceleration
    )

    robot.moveJ_safe(
        cal["violin_hover_joints"],
        arm_config.joint_speed,
        arm_config.joint_acceleration,
        cal["violin_hover_position"]
    )

    if not user.yes_no("\nAnother Task (y/n)? "):
        user.beep()
        user.wait_for_enter("Return Home")

        robot.moveJ(
            arm_config.home_joints,
            arm_config.joint_speed,
            arm_config.joint_acceleration
        )

        if arm_config.bow_loaded:
            if user.yes_no("\nUnload Bow (y/n)?"):
                robot.moveJ(
                    arm_config.load_joints,
                    arm_config.joint_speed,
                    arm_config.joint_acceleration
                )
                arm_config.save_bow_loaded(False)

        print("\nSimple Bowing Program Complete")
        robot.stop()
        sys.exit()

    at_violin_hover = True

        