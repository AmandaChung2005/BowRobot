from robodk.robolink import Robolink, ITEM_TYPE_ROBOT

RDK = Robolink()
robot = RDK.Item('', ITEM_TYPE_ROBOT)

print("Robot:", robot.Name())
print("Current joints:", robot.Joints().list())

pose = robot.Pose()

print("SolveIK:")
ik = robot.SolveIK(pose)
print(type(ik))
print(ik)

print("\nSolveIK_All:")
ika = robot.SolveIK_All(pose)
print(type(ika))
print(ika)

print("\nRows:", ika.Rows())
print("Cols:", ika.Cols())