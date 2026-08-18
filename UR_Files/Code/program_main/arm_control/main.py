# RoboDK
import sys
import time

import arm_control.arm_config as arm_config
import arm_control.path_planner as path
import arm_control.robot_interface_new as robot
import arm_control.user_interface as user

cal = arm_config.data

# Start User Interface
user.start_interface()

user.wait_for_enter("Program Started")


# Move to Home
user.wait_for_enter("Move to Home")


robot.moveJ(
    arm_config.home_joints,
    arm_config.joint_speed,
    arm_config.joint_acceleration
)


time.sleep(0.02)

at_violin_hover = False

while True:
    # Select Bowing Type
    bow = user.select_bowing_type()


    if bow == "rosin":
        user.wait_for_enter("Move Above Rosin")

        hover_tip = path.lift_pose(
            cal["rosin_position"]["tip"],
            arm_config.rosin_hover
        )

        robot.moveL(
            hover_tip,
            arm_config.speed,
            arm_config.acceleration
        )

        while True:
            user.wait_for_enter("Move Onto Rosin")

            robot.moveJ(
                cal["rosin_joints"]["tip"],
                arm_config.joint_speed,
                arm_config.joint_acceleration
            )

            user.wait_for_enter("Rosin Bow")

            for _ in range(arm_config.rosin_cycles):
                robot.bowing_segment(
                    cal["rosin_position"]["tip"],
                    cal["rosin_position"]["frog"],
                    cal["rosin_joints"]["tip"],
                    cal["rosin_joints"]["frog"],
                    rosin = True
                )

                robot.bowing_segment(
                    cal["rosin_position"]["frog"],
                    cal["rosin_position"]["tip"],
                    cal["rosin_joints"]["frog"],
                    cal["rosin_joints"]["tip"],
                    rosin = True
                )


            user.wait_for_enter("Lift Bow")

            robot.moveL(
                hover_tip,
                arm_config.speed,
                arm_config.acceleration
            )

            if user.yes_no("\nEnough Rosin(y/n)? "):
                break

    elif bow in ("basic", "spiccato"):
        # Move to Above String
        if not at_violin_hover:
            user.wait_for_enter("Move Above String")

            robot.moveJ(
                cal["violin_hover_joints"],
                arm_config.speed,
                arm_config.acceleration
            )


        if bow == "basic":
            current_string = user.select_string()
            start_pos = user.select_start_position()

            # Move onto String Position
            user.wait_for_enter("Move Onto String")      

            if start_pos == "middle":
                robot.moveJ(
                    robot.get_middle_joints(current_string),
                    arm_config.speed,
                    arm_config.acceleration
                )
            else:
                robot.moveJ(
                    cal["joint_paths"][current_string][start_pos],
                    arm_config.speed,
                    arm_config.acceleration
                )

            # Basic Bowing
            user.wait_for_enter("Start Bowing")

            basic_cartesian_path, basic_joint_path = path.basic(
                current_string,
                start_pos
            )

            for _ in range(arm_config.bowing_cycles):
                for i in range(len(basic_cartesian_path)-1):

                    robot.bowing_segment(
                        basic_cartesian_path[i],
                        basic_cartesian_path[i+1],
                        basic_joint_path[i],
                        basic_joint_path[i+1]
                    )

        elif bow == "spiccato":
            current_string = user.select_string()

            # Move onto String Position
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

            # Spiccato Bowing
            user.wait_for_enter("Start Bowing")

            for _ in range(arm_config.spiccato_cycles):
                for i in range(len(spiccato_cartesian_path) -1):
                    robot.bowing_segment(
                        spiccato_cartesian_path[i],
                        spiccato_cartesian_path[i+1],
                        spiccato_joint_path[i],
                        spiccato_joint_path[i+1]
                    )

        # Stops Motion
        robot.stop()

        # Move Bow Out of the Way
        user.wait_for_enter("Lift Bow")

        robot.moveJ(
            cal["violin_hover_joints"],
            arm_config.speed,
            arm_config.acceleration
        )

        at_violin_hover = True

    if not user.yes_no("\nAnother Task (y/n)? "):
        user.wait_for_enter("Return Home")

        robot.moveJ(
            arm_config.home_joints,
            arm_config.joint_speed,
            arm_config.joint_acceleration
        )

        print("\nProgram Complete")

        robot.stop()
        sys.exit()


