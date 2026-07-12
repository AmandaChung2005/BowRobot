# https://github.com/IanBallinger/Robotics_final_spring2026_thursPM/blob/main/UR5/demo_4_16/keyboard_proto.py

from pynput.keyboard import Key, Listener
import time
import numpy as np
from scipy.spatial.transform import Rotation
from rtde_control import RTDEControlInterface
import config

directions = {
    "w": False,     # Forward
    "a": False,     # Left
    "s": False,     # Down
    "d": False,     # Right
    "q": False,     # Up
    "e": False,     # Down
    "halt": False,  # Ends Program
    "start": False  # Starts Program
}

def pressed(key):
    global gripperstate
    if key == Key.enter:   # Stops lisetener
        directions["start"] = True
        print("Program starting...")
        return

    if key == Key.delete:   # Stops lisetener
        directions["halt"] = True
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
    else:
        print ("Robot Connection Not Working")
        config.rtde_c.stopScript()

print ("Press ENTER to start")
print ("Press DELETE to stop")

while not directions["start"]:
    if directions["halt"]:
        break
    time.sleep(0.01)

while not directions["halt"]:
    if directions["start"]:
        
    
