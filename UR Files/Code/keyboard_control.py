# https://github.com/IanBallinger/Robotics_final_spring2026_thursPM/blob/main/UR5/demo_4_16/keyboard_proto.py

from pynput.keyboard import Key, Listener
import time
import numpy as np
from scipy.spatial.transform import Rotation
from rtde_control import RTDEControlInterface
import config

commands = {
    "start": False, # Starts Program
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
        print("Program starting...")
        return

    if key == Key.delete:   # Stops lisetener
        commands["halt"] = True
        return False
    
    try:
        if key.char in directions.keys() and not (directions[key.char]):  # Detects key pressed
            directions[key.char] = True
            print(f"{key.char} Pressed")
    except AttributeError:
        pass

def released(key):
    try:
        if key.char in directions.keys():
            directions[key.char] = False
            print(f"{key.char} Released")
    except AttributeError:
        pass

with Listener(on_press = pressed, on_release = released) as listener:

    connection_tries = 0
    if not config.rtde_c.isConnected():
        while connection_tries < 3:
            config.rtde_c.reconnect()
            time.sleep(0.1)
            if config.rtde_c.isConnected():
                break
            connection_tries += 1

    if config.rtde_c.isConnected():
        print("Robot Connection Successful!")
        print ("Press ENTER to start")
        print ("Press DELETE to stop")
    else:
        print ("Robot Connection Not Working, Try Again")
        config.rtde_c.stopScript()


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
while not commands["start"]:
    if commands["halt"]:
        break
    time.sleep(0.01)

while not commands["halt"]:
    t_start = config.rtde_c.initPeriod()

    if not any(directions.values()):
         config.rtde_c.forceModeStop()

    else:
        selection_vector = [0, 0, 0, 0, 0, 0]
        wrench = [0, 0, 0, 0, 0, 0]

        for key, (axis, force) in force_commands.items():
            if directions[key]:
                selection_vector[axis] = 1
                wrench[axis] += force

        config.rtde_c.forceMode(
            config.task_frames[config.current_string],
            selection_vector,
            wrench,
            config.force_type,
            limits
        )

    config.rtde_c.waitPeriod(t_start)

config.rtde_c.forceModeStop()
