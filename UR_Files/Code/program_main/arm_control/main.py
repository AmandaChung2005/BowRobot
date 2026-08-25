# RoboDK
import sys
import time
import numpy as np

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
print("Program Started")

# # Move to Home
# user.beep()
# user.wait_for_enter("Move to Home")

# robot.moveJ_safe(
#     arm_config.home_joints,
#     arm_config.joint_speed,
#     arm_config.joint_acceleration,
#     arm_config.home_position
# )

# time.sleep(0.02)

at_violin_hover = False

while True:
    # Select Bowing Type
    user.beep()
    bow = user.select_bowing_type()


    if bow == "rosin":
        user.beep()
        user.wait_for_enter("Move Above Rosin")

        hover_pose = path.lift_pose(
            cal["rosin_position"]["tip"],
            arm_config.rosin_hover
        )

        # hover_joints = robot.solveIK(
        #     hover_pose,
        #     reference = cal["rosin_joints"]["tip"]
        # )

        robot.moveL_safe(
            hover_pose,
            arm_config.speed,
            arm_config.acceleration
        )

        force_offset = robot.zero_force_sensor()
        robot.set_force_offset(force_offset)
            
        while True:
            user.beep()
            user.wait_for_enter("Move Onto Rosin")

            robot.moveL(
                cal["rosin_position"]["tip"],
                arm_config.joint_speed,
                arm_config.joint_acceleration
            )

            task_frame = robot.generate_task_frame()

            user.beep()
            user.wait_for_enter("Rosin Bow")

            for _ in range(arm_config.rosin_cycles):
                robot.bowing_segment(
                    cal["rosin_position"]["tip"],
                    cal["rosin_position"]["frog"],
                    cal["rosin_joints"]["tip"],
                    cal["rosin_joints"]["frog"],
                    task_frame = task_frame,
                    rosin = True
                )

                robot.bowing_segment(
                    cal["rosin_position"]["frog"],
                    cal["rosin_position"]["tip"],
                    cal["rosin_joints"]["frog"],
                    cal["rosin_joints"]["tip"],
                    task_frame = task_frame,
                    rosin = True
                )

            user.beep()
            user.wait_for_enter("Lift Bow")

            robot.moveL(
                hover_pose,
                arm_config.speed,
                arm_config.acceleration
            )

            if user.yes_no("\nEnough Rosin(y/n)? "):
                break

    elif bow in ("basic", "spiccato"):
        # Move to Above String
        if not at_violin_hover:
            user.beep()
            user.wait_for_enter("Move Above String")

            robot.moveJ_safe(
                cal["violin_hover_joints"],
                arm_config.speed,
                arm_config.acceleration,  
                cal["violin_hover_position"]
            )

        force_offset = robot.zero_force_sensor()
        robot.set_force_offset(force_offset)


        if bow == "basic":
            current_string = user.select_string()
            start_pos = user.select_start_position()

            # Move onto String Position
            user.beep()
            user.wait_for_enter("Move Onto String")      

            if start_pos == "middle":
                middle_pose = robot.get_middle_pose(current_string)
                middle_joints = robot.get_middle_joints(current_string)

                print("\n=== MIDDLE MOVE DEBUG ===")

                print(
                    "Expected Middle Pose (mm/deg):"
                )
                print(
                    np.round(
                        middle_pose,
                        3
                    )
                )

                print(
                    "\nCalculated Middle Joints (deg):"
                )
                print(
                    np.round(
                        middle_joints,
                        3
                    )
                )

                robot.moveJ(
                    middle_joints,
                    arm_config.speed,
                    arm_config.acceleration
                )

                actual_middle_pose = np.asarray(
                    robot.getActualTCPPose(),
                    dtype=float
                ).copy()

                print(
                    "\nActual TCP After Middle Move:"
                )
                print(
                    np.round(
                        actual_middle_pose,
                        3
                    )
                )

                print(
                    "\nMiddle Position Error (mm):"
                )
                print(
                    np.linalg.norm(
                        actual_middle_pose[:3]
                        - middle_pose[:3]
                    )
                )

            else:
                robot.moveJ(
                    cal["joint_paths"][current_string][start_pos],
                    arm_config.speed,
                    arm_config.acceleration
                )

            task_frame = robot.generate_task_frame()

            frog_pose = cal["string_paths"][current_string]["frog"]
            tip_pose = cal["string_paths"][current_string]["tip"]

            force_monitor = None

            if not config.simulation and arm_config.useForce:
                if config.monitorForce:
                    force_monitor = robot.start_force_monitor(
                        task_frame,
                        frog_pose,
                        tip_pose,
                        interval = 0.05
                    )

                force_preparation_complete = not arm_config.useForce

                while not force_preparation_complete:
                    prepared = robot.prepare_force(
                        task_frame,
                        arm_config.selection_vector,
                        arm_config.wrench,
                        arm_config.limits,
                        arm_config.force_constant,
                        arm_config.bow_force,
                        frog_pose,
                        tip_pose
                    )

                    if prepared:
                        print("Force Prepared")
                        force_preparation_complete = True
                        break

                    if force_monitor is not None:
                        robot.pause_force_monitor(force_monitor)

                    print("\nForce Preparation Failed")

                    while True:
                        choice = input(
                            "\nForce Preparation Failed:\n"
                            " [M] Move On Anyway\n"
                            " [T] Try Again\n"
                            " [E] Exit Program\n"
                        ).strip().lower()

                        if choice in {"m", "move"}:
                            print("Moving On Without Successful Force Preparation")
                            force_preparation_complete = True
                            break

                        if choice in {"t", "try"}:
                            print("Trying Force Preparation Again")
                            if force_monitor is not None:
                                robot.resume_force_monitor(force_monitor)
                            break

                        if choice in {"e", "exit"}:
                            print("Exiting Program")

                            if force_monitor is not None:
                                robot.stop_force_monitor(force_monitor)

                            robot.forceModeStop()
                            robot.stop()
                            sys.exit()
                        else:
                            print("Invalid Input, Try Again")
            
                if force_monitor is not None:
                    robot.pause_force_monitor(force_monitor)

            # Basic Bowing
            user.beep()
            user.wait_for_enter("Start Bowing")

            if force_monitor is not None:
                robot.resume_force_monitor(force_monitor)


            basic_cartesian_path, basic_joint_path = path.basic(
                current_string,
                start_pos
            )

            print("\n=== BASIC PATH DEBUG ===")
            print(f"Start Position: {start_pos}")
            print(f"Number of Cartesian Points: {len(basic_cartesian_path)}")
            print(f"Number of Bow Segments: {len(basic_cartesian_path) - 1}")

            print("\n=== BASIC PATH Z DEBUG ===")

            for i, pose in enumerate(basic_cartesian_path):
                print(
                    f"Point {i}: "
                    f"X={pose[0]:.2f}, "
                    f"Y={pose[1]:.2f}, "
                    f"Z={pose[2]:.2f}"
                )

            for cycle in range(arm_config.bowing_cycles):

                print(
                    f"\n=== CYCLE "
                    f"{cycle + 1}/{arm_config.bowing_cycles} ==="
                )

                for i in range(len(basic_cartesian_path) - 1):

                    robot.bowing_segment(
                        basic_cartesian_path[i],
                        basic_cartesian_path[i+1],
                        basic_joint_path[i],
                        basic_joint_path[i+1],
                        task_frame = task_frame
                    )

                    print(
                        f"Segment "
                        f"{i + 1} complete"
                    )

                print(
                    f"=== CYCLE "
                    f"{cycle + 1} COMPLETE ==="
                )

            if force_monitor is not None:
                robot.stop_force_monitor(force_monitor)
                force_monitor = None

            if arm_config.useForce:
                robot.forceModeStop()

        elif bow == "spiccato":
            current_string = user.select_string()

            # Move onto String Position
            user.beep()
            user.wait_for_enter("Move Onto String")          

            spiccato_cartesian_path, spiccato_joint_path = path.spiccato(current_string)

            robot.moveJ(
                spiccato_joint_path[0],
                arm_config.speed,
                arm_config.acceleration
            )

            # print("\nMoving...")
            # robot.moveJ(
            #     cal["joint_paths"][arm_config.current_string]["frog"],
            #     arm_config.speed,
            #     arm_config.acceleration
            # )

            task_frame = robot.generate_task_frame()

            # Spiccato Bowing
            user.beep()
            user.wait_for_enter("Start Bowing")

            for _ in range(arm_config.spiccato_cycles):
                for i in range(len(spiccato_cartesian_path) -1):
                    robot.bowing_segment(
                        spiccato_cartesian_path[i],
                        spiccato_cartesian_path[i+1],
                        spiccato_joint_path[i],
                        spiccato_joint_path[i+1],
                        task_frame = task_frame
                    )

        # Stops Motion
        robot.stop()

        # Move Bow Out of the Way
        user.beep()
        user.wait_for_enter("Lift Bow")

        robot.moveJ(
            cal["violin_hover_joints"],
            arm_config.speed,
            arm_config.acceleration
        )

        at_violin_hover = True

    if not user.yes_no("\nAnother Task (y/n)? "):
        user.beep()
        user.wait_for_enter("Return Home")

        robot.moveJ(
            arm_config.home_joints,
            arm_config.joint_speed,
            arm_config.joint_acceleration
        )

        print("\nProgram Complete")

        robot.stop()
        sys.exit()


