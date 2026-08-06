# https://github.com/IanBallinger/Robotics_final_spring2026_thursPM/blob/main/UR5/demo_4_16/keyboard_proto.py

from pynput.keyboard import Key, Listener
import time
import numpy as np
from scipy.spatial.transform import Rotation
import config
import robot_interface as robot

# RoboDK
import sys
sys.path.append(config.code_path)

commands = {
    "start": False, # Starts Program
    "pause": False, # Pauses Program
    "halt": False   # Ends Program
}

directions = {
    "w": False,  # +Y
    "a": False,  # +X
    "s": False,  # -Y
    "d": False,  # -X
    "q": False,  # +Z
    "e": False,  # -Z
    "i": False,  # +Roll
    "j": False,  # +Pitch
    "k": False,  # -Roll
    "l": False,  # -Pitch
    "u": False,  # +Yaw
    "o": False   # -Yaw
}

def pressed(key):
    global gripperstate
    if key == Key.enter:   # Starts listener
        commands["start"] = True
        return

    if key == Key.space:
        commands["pause"] = True
        return

    if key == Key.delete:   # Stops lisetener
        commands["halt"] = True
        return
        
    try:
        if key.char in directions.keys() and not (directions[key.char]):  # Detects key pressed
            directions[key.char] = True
            # print(f"{key.char} Pressed")
    except AttributeError:
        pass

def released(key):
    try:
        if key.char in directions.keys():
            directions[key.char] = False
            # print(f"{key.char} Released")
    except AttributeError:
        pass


with Listener(on_press = pressed, on_release = released) as listener:

    if config.simulation:
        print("Connected to RoboDK Simulation")
        print("Press ENTER to start")
        print("Press DELETE to stop")
    else:
        connection_tries = 0

        if not robot.rtde_c.isConnected():
            while connection_tries < 3:
                robot.rtde_c.reconnect()
                time.sleep(0.1)
                if robot.rtde_c.isConnected():
                  break
                connection_tries += 1

        if robot.rtde_c.isConnected():
            print("Robot Connection Successful!")
            print ("Press ENTER to start")
            print ("Press DELETE to stop")
        else:
            print ("Robot Connection Not Working, Try Again")
            robot.rtde_c.stopScript()


    # Force Mode Parameters
    limits = [2, 2, 2, 1, 1, 1]

    force_commands = {
        "a": (0, 10),   # +X
        "d": (0, -10),  # -X
        "w": (1, 10),   # +Y
        "s": (1, -10),  # -Y
        "q": (2, 10),   # +Z
        "e": (2, -10),  # -Z
        "i": (3, 2),   # +Roll
        "k": (3, -2),  # -Roll
        "j": (4, 2),   # +Pitch
        "l": (4, -2),  # -Pitch
        "u": (5, 2),   # +Yaw
        "o": (5, -2)   # -Yaw
    }

    # Force Control
    while True:
        commands ["start"] = False

        while not commands["start"]:
            time.sleep(0.01)
        print("Program Started")
        
        while commands["start"]:
            t_start = robot.initPeriod()

            if commands["pause"]:
                print("Program Paused")
                print ("Press ENTER to resume")
                print ("Press DELETE to stop")

                robot.stop()

                commands["pause"] = False
                commands["start"] = False

                for k in directions:
                    directions[k] = False

            if commands["halt"]:
                print("Program Terminated")
                robot.stop()
                listener.stop()
                sys.exit()

            if not any(directions.values()):
                robot.stop()

            else:
                selection_vector = [0, 0, 0, 0, 0, 0]
                wrench = [0, 0, 0, 0, 0, 0]

                for key, (axis, force) in force_commands.items():
                    if directions[key]:
                        selection_vector[axis] = 1
                        wrench[axis] += force
                if config.simulation:
                    robot.jogCartesian(selection_vector, wrench)
                else:
                    # robot.forceMode(
                    #     config.task_frames[config.current_string],
                    #     selection_vector,
                    #     wrench,
                    #     limits
                    # )

                    pose = robot.getActualTCPPose()

                    step = 0.010  # 10 mm

                    if directions["w"]:
                        pose[1] += step
                    if directions["s"]:
                        pose[1] -= step
                    if directions["a"]:
                        pose[0] += step
                    if directions["d"]:
                        pose[0] -= step
                    if directions["q"]:
                        pose[2] += step
                    if directions["e"]:
                        pose[2] -= step

                    robot.moveL(
                        pose,
                        0.05,
                        0.1
                    )

            robot.waitPeriod(t_start)

