import sys
from pathlib import Path

sys.path.append(str(Path(__file__).resolve().parent.parent))

from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import config
import numpy as np
import math


# =========================
# Configuration
# =========================

ROBOT_IP = config.arm_ip

SPEED = 0.1       # rad/s
ACCELERATION = 0.1  # rad/s^2
MOVE_DEGREES = 5.0


# =========================
# Connect to robot
# =========================

print(f"Robot IP: {ROBOT_IP}")
print("Connecting...")

rtde_c = RTDEControlInterface(ROBOT_IP)
rtde_r = RTDEReceiveInterface(ROBOT_IP)

print("RTDE connection successful!")


# =========================
# Read current position
# =========================

current_joints = rtde_r.getActualQ()

print("\nCurrent joint positions:")
print(current_joints)

print("\nCurrent joint 1:")
print(f"{math.degrees(current_joints[0]):.2f} degrees")


# =========================
# Create test position
# =========================

test_joints = current_joints.copy()

# Convert 5 degrees to radians
move_radians = math.radians(MOVE_DEGREES)

# Move joint 1 by +5 degrees
test_joints[0] += move_radians

print("\nTest position:")
print(test_joints)

print(
    f"\nJoint 1 will move "
    f"{MOVE_DEGREES} degrees "
    f"from {math.degrees(current_joints[0]):.2f} "
    f"to {math.degrees(test_joints[0]):.2f} degrees."
)


# =========================
# Confirm before moving
# =========================

input("\nPress ENTER to move the robot...")


# =========================
# Move to test position
# =========================

print("\nMoving to test position...")

rtde_c.moveJ(
    test_joints,
    SPEED,
    ACCELERATION
)

print("Reached test position.")


# =========================
# Confirm before returning
# =========================

input("\nPress ENTER to return to the original position...")


# =========================
# Return to original
# =========================

print("\nReturning to original position...")

rtde_c.moveJ(
    current_joints,
    SPEED,
    ACCELERATION
)

print("Returned to original position.")


# =========================
# Disconnect
# =========================

rtde_c.stopScript()
rtde_c.disconnect()

print("\nRTDE test complete.")

