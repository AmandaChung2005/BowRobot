from pynput.keyboard import Key, Listener
import time
import numpy as np
from scipy.spatial.transform import Rotation
from rtde_control import RTDEControlInterface
import config



# Movement Limits

# Force Control
task_frame = [0, 0, 0, 0, 0, 0]

selection_vector = [0, 0, 1, 0, 0, 0]  # Z-Axis Force Control

force_type = 2
wrench = [0, 0, -1.0, 0, 0, 0]  # Desired N of Force

limits = [2, 2, 4, 1, 1, 1]

while True:
    t_start = config.rtde_c.initPeriod()
    config.rtde_c.waitPeriod(t_start)

while bowing:
    rtde_c.forceMode(task_frame,
                     selection_vector,
                     wrench,
                     force_type,
                     limits)