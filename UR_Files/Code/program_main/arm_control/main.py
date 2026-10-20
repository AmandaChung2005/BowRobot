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
print("Program Started")

if not robot.is_remote_control():
    print("\n" + "=" * 60)
    print("Error: Polyscope Isn't in Remote Control Mode")
    print("=" * 60)
    print("\nPlease Switch Polyscope to Remote Control Mode")
    print("Then Restart the Program")
    print()
    sys.exit()

# Load Bow?
if not arm_config.bow_loaded and not config.simulation:
    if user.yes_no("Load Bow (y/n)?"):
        user.wait_for_enter("Moving to Bow Loading Position")
        robot.moveJ_safe(
            arm_config.load_joints,
            arm_config.joint_speed,
            arm_config.joint_acceleration,
            arm_config.load_position
        )

        user.wait_for_enter("Press ENTER When the Bow Has Been Secured")

        arm_config.save_bow_loaded(True)

        print("\nBow Loaded Successfully, Moving to Home")

        time.sleep(1.0)

        robot.moveJ(
            arm_config.home_joints,
            arm_config.joint_speed,
            arm_config.joint_acceleration
        )

        print("\nReturned Home")


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

    if bow is None:
        continue

    if bow == "unload":
        user.beep()
        user.wait_for_enter("Move to Unload Position")

        robot.moveJ(
            arm_config.load_joints,
            arm_config.joint_speed,
            arm_config.joint_acceleration
        )

        arm_config.save_bow_loaded(False)

        print("\nBow Unloaded")
        continue

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

        if not config.simulation and arm_config.useForce:
            force_offset = robot.zero_force_sensor()
            robot.set_force_offset(force_offset)
            
        while True:
            user.beep()
            user.wait_for_enter("Move Onto Rosin")

            robot.moveL(
                cal["rosin_position"]["tip"],
                arm_config.speed,
                arm_config.acceleration
            )

            commanded_force = abs(float(arm_config.rosin_wrench[2]))

            preparation_monitor = None
            force_prepared = None

            if not config.simulation and arm_config.useForce:
                if config.monitorForce:
                    preparation_monitor = robot.start_force_monitor(
                        task_frame = robot.generate_task_frame(),
                        frog_pose = cal["rosin_position"]["frog"],
                        tip_pose = cal["rosin_position"]["tip"],
                        interval = 0.05,
                        show_graph = False
                    )
                try:
                    while True:
                        prepared = robot.prepare_force_manual(
                            commanded_force=abs(float(arm_config.rosin_wrench[2])),
                            timeout=arm_config.force_prepare_timeout
                        )

                        if prepared:
                            print("Force Preparation Successful, Moving On")
                            break

                        choice = robot.force_preparation_menu()
                        
                        if choice == "move":
                            print("Continuing without confirmed target force")
                            break

                        if choice == "try":
                            print("Retrying force preparation")
                            continue

                        if choice == "exit":
                            robot.stop()
                            sys.exit()
                finally:
                    if preparation_monitor is not None:
                        robot.stop_force_monitor(preparation_monitor)

            

            #     force_preparation_complete = not arm_config.useForce

            #     while not force_preparation_complete:
            #         prepared = robot.prepare_force(
            #             task_frame,
            #             arm_config.selection_vector,
            #             arm_config.wrench,
            #             arm_config.limits,
            #             arm_config.force_constant,
            #             arm_config.bow_force,
            #             frog_pose,
            #             tip_pose
            #         )

            #         if prepared:
            #             print("Force Prepared")
            #             force_preparation_complete = True
            #             break

            #         if force_monitor is not None:
            #             robot.pause_force_monitor(force_monitor)

            #         choice = robot.force_preparation_menu()

            #         if choice == "move":
            #             force_preparation_complete = True

            #         elif choice == "try":
            #             continue

            #         elif choice == "exit":
            #             if force_monitor is not None:
            #                 robot.stop_force_monitor(force_monitor)
            #             robot.forceModeStop()
            #             robot.stop()
            #             sys.exit()
            
            #     if force_monitor is not None:
            #         robot.pause_force_monitor(force_monitor)

            user.beep()
            user.wait_for_enter("Rosin Bow")

            # if force_monitor is not None:
            #     robot.resume_force_monitor(force_monitor)

            # for _ in range(arm_config.rosin_cycles):
            #     robot.bowing_segment(
            #         cal["rosin_position"]["tip"],
            #         cal["rosin_position"]["frog"],
            #         cal["rosin_joints"]["tip"],
            #         cal["rosin_joints"]["frog"],
            #         task_frame = task_frame,
            #         rosin = True
            #     )

            #     robot.bowing_segment(
            #         cal["rosin_position"]["frog"],
            #         cal["rosin_position"]["tip"],
            #         cal["rosin_joints"]["frog"],
            #         cal["rosin_joints"]["tip"],
            #         task_frame = task_frame,
            #         rosin = True
            #     )

            # if force_monitor is not None:
            #     robot.stop_force_monitor(force_monitor)
            #     force_monitor = None

            bowing_monitor = None

            if not config.simulation and arm_config.useForce:
                bowing_monitor = robot.start_force_monitor(
                    task_frame = robot.generate_task_frame(),
                    frog_pose = cal["rosin_position"]["frog"],
                    tip_pose = cal["rosin_position"]["tip"],
                    interval = 0.05,
                    show_graph = True
                )

            try:
                robot.execute_bowing_path(
                    cartesian_path = [
                        cal["rosin_position"]["tip"],
                        cal["rosin_position"]["frog"],
                        cal["rosin_position"]["tip"],
                    ],
                    joint_path = [
                        cal["rosin_joints"]["tip"],
                        cal["rosin_joints"]["frog"],
                        cal["rosin_joints"]["tip"],
                    ],
                    cycles = arm_config.rosin_cycles,
                    rosin = True,
                    commanded_force = commanded_force
                )
            finally:
                if bowing_monitor is not None:
                    robot.stop_force_monitor(bowing_monitor)

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


        if bow == "basic":
            while True:
                current_string = user.select_string()
                if current_string is None:
                    break

                start_pos = user.select_start_position()
                if start_pos is None:
                    continue

                if start_pos == "middle":
                    start_dir = user.select_start_direction()
                    if start_dir is None:
                        continue
                    break
                break
            if current_string is None:
                continue

            force_offset = robot.zero_force_sensor()
            robot.set_force_offset(force_offset)

            # Move onto String Position
            user.beep()
            user.wait_for_enter("Move Onto String")   

            if start_pos == "middle":
                target_pose = robot.get_middle_pose(current_string)
                target_joints = robot.get_middle_joints(current_string)

            else:
                target_pose = np.asarray(
                    cal["string_paths"][current_string][start_pos],
                    dtype = float
                )
                target_joints = np.asarray(
                    cal["joint_paths"][current_string][start_pos],
                    dtype = float
                )

            intermediate_pose = np.asarray(
                cal["violin_hover_position"],
                dtype = float
            ).copy()
            intermediate_pose[3:6] = target_pose[3:6]

            intermediate_joints = robot.solveIK(
                intermediate_pose
            )

            print("\n=== INTERMEDIATE POSE ===")
            print(np.round(intermediate_pose, 3))

            print("\n=== INTERMEDIATE JOINTS ===")
            print(np.round(intermediate_joints, 3))

            print("\n=== TARGET JOINTS ===")
            print(np.round(target_joints, 3))

            robot.moveJ_blended(
                intermediate_joints,
                target_joints,
                arm_config.joint_speed,
                arm_config.joint_acceleration,
                blend_radius = 0.0
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

                    choice = robot.force_preparation_menu()

                    if choice == "move":
                        force_preparation_complete = True
                    elif choice == "try":
                        continue
                    elif choice == "exit":
                        if force_monitor is not None:
                            robot.stop_force_monitor(force_monitor)
                        robot.forceModeStop()
                        robot.stop()
                        sys.exit()
            
                if force_monitor is not None:
                    robot.pause_force_monitor(force_monitor)

            # Basic Bowing
            user.beep()
            user.wait_for_enter("Start Bowing")

            if force_monitor is not None:
                robot.resume_force_monitor(force_monitor)

            if start_pos == "middle":
                basic_cartesian_path, basic_joint_path = path.basic(
                    current_string,
                    start_pos,
                    start_dir
                )
            else:
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

            if current_string is None:
                continue

            force_offset = robot.zero_force_sensor()
            robot.set_force_offset(force_offset)

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

        if arm_config.bow_loaded:
            if user.yes_no("\nUnload Bow (y/n)?"):
                robot.moveJ(
                    arm_config.load_joints,
                    arm_config.joint_speed,
                    arm_config.joint_acceleration
                )

                arm_config.save_bow_loaded(False)

        print("\nProgram Complete")

        robot.stop()
        sys.exit()


