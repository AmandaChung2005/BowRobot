
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface

IP = "192.168.56.101"

rtde_c = RTDEControlInterface(
    IP,
    500.0,
    flags=int(RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
)

rtde_r = RTDEReceiveInterface(IP)

print("Connected:", rtde_c.isConnected())

q = rtde_r.getActualQ()
q[0] += 0.2

print("moveJ returned:", rtde_c.moveJ(q, 0.5, 0.5))

rtde_c.stopScript()
