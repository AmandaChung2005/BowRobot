# RoboDK
import sys


from pynput.keyboard import Key, Listener
import time

import config
sys.path.append(config.code_path)
import path_planner as path
import robot_interface as robot

cal = config.data

# Keyboard Control
halt = False
selection = None
using_input = False

def pressed(key):
    global halt, selection

    if key == Key.delete:
        halt = True
        print ("\nProgram Terminated")
        return False

    if using_input:
        return

    if key == Key.enter:
        selection = "enter"

listener = Listener(on_press = pressed)
listener.start()

def check_halt():
    if halt:
        robot.stop()
        listener.stop()
        sys.exit()
   
def wait_for_enter(message):
    global selection

    selection = None

    print()
    print(message)
    print("Press Enter to continue")
    print("Press Delete to stop")

    while selection != "enter":
        check_halt()
        time.sleep(0.01)

    selection = None

def safe_input(prompt):
    global using_input

    using_input = True
    try:
        answer = input(prompt)
    finally:
        using_input = False

    check_halt()
    return answer


wait_for_enter("Program Started")


# Move to Home
wait_for_enter("Move to Home")

robot.moveJ(
    config.home_joints,
    config.joint_speed,
    config.joint_acceleration
)

time.sleep(0.02)

while True:
    # Option Selection
    while True:
        bow = safe_input("\nBowing Type (Rosin/Basic): ").strip().lower()

        if bow in ("b", "basic"):
            bow = "basic"
            break
        elif bow in ("r", "rosin"):
            bow = "rosin"
            break
        else:
            print("Invalid input, try again")
            continue

    if bow == "rosin":
        while True:
            wait_for_enter("Move Above Rosin")
            hover_tip = path.lift_pose(
                config.rosin_position["tip"],
                config.rosin_hover
            )

            robot.moveL(
                hover_tip,
                config.speed,
                config.acceleration
            )

            wait_for_enter("Move Onto Rosin")
            robot.moveJ(
                cal["rosin_joints"]["tip"],
                config.speed,
                config.acceleration
            )

            robot.forceMode(
                config.rosin_task_frame,
                config.rosin_selection_vector,
                config.rosin_wrench,
                config.rosin_limits
            )

            wait_for_enter("Rosin Bow")

            for _ in range(config.rosin_cycles):
                    robot.bowing_segment(
                        config.rosin_position["tip"],
                        config.rosin_position["frog"],
                        config.rosin_joints["tip"],
                        config.rosin_joints["frog"]
                    )

                    robot.bowing_segment(
                        config.rosin_position["frog"],
                        config.rosin_position["tip"],
                        config.rosin_joints["frog"],
                        config.rosin_joints["tip"]
                    )

            robot.stop()

            wait_for_enter("Lift Bow")

            robot.moveL(
                hover_tip,
                config.speed,
                config.acceleration
            )

            while True:
                again = safe_input("\nEnough rosin applied? ").strip().lower()

                if again in ("y", "yes"):
                    break
                if again in ("n", "no"):
                    break
                print("Invalid input, try again") 

            if again in ("y", "yes"):
                break

            continue

    if bow == "basic":
        # Move to Above String
        wait_for_enter("Move Above String")

        robot.moveJ(
            cal["violin_hover_joints"],
            config.speed,
            config.acceleration
        )

        # Move onto String Position
        wait_for_enter("Move Onto String")          

        robot.moveJ(
            cal["joint_paths"][config.current_string][config.start_pos],
            config.speed,
            config.acceleration
        )

        # Basic Bowing
        wait_for_enter("Start Bowing")

        if bow == "basic":
            basic_cartesian_path, basic_joint_path = path.basic()

            for _ in range(config.bowing_cycles):
                for i in range(len(basic_cartesian_path)-1):

                    robot.bowing_segment(
                        basic_cartesian_path[i],
                        basic_cartesian_path[i+1],
                        basic_joint_path[i],
                        basic_joint_path[i+1]
                    )

        # Stops Motion
        robot.stop()

        # Move Bow Out of the Way
        wait_for_enter("Lift Bow")

        robot.moveJ(
            cal["violin_hover_joints"],
            config.speed,
            config.acceleration
        )

    # Other Task?
    while True:
        again = safe_input("\nDo another task? ").strip().lower()

        if again in ("y", "yes"):
            break

        if again in ("n", "no"):
            # Return to Home
            wait_for_enter("Return Home")

            robot.moveJ(
                config.home_joints,
                config.joint_speed,
                config.joint_acceleration
            )

            print("\nProgram Complete")
            robot.stop()
            sys.exit()

        print("Invalid input, try again")

    continue


