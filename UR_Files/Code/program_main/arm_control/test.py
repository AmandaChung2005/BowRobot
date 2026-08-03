from robodk.robolink import *
from robodk.robomath import *


RDK = Robolink()

robot = RDK.Item('', ITEM_TYPE_ROBOT)

print(robot.Pose())
print(robot.SolveIK(robot.Pose()))