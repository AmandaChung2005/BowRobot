from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface
import time

ROBOT_IP = "192.168.56.101"

print("Connecting to robot...")

rtde_c = RTDEControlInterface(ROBOT_IP)
rtde_r = RTDEReceiveInterface(ROBOT_IP)

print("Control connected:", rtde_c.isConnected())

# Read current joint position
q = rtde_r.getActualQ()

print("Current joints:")
print(q)

# Move joint 1 by only 0.05 radians (~2.9 degrees)
target = q.copy()
target[0] += 0.05

print("\nTarget joints:")
print(target)

print("\nMoving...")
result = rtde_c.moveJ(
    target,
    0.1,   # velocity
    0.1    # acceleration
)

print("moveJ returned:", result)

time.sleep(1)

print("\nFinal joints:")
print(rtde_r.getActualQ())

rtde_c.stopScript()
rtde_c.disconnect()
rtde_r.disconnect()

print("Done.")