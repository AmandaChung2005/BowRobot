import config

# Initialization (Simulation vs Reality)
if config.simulation:
    import sys

    sys.path.append(config.robodk_python_path)

    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
    from robodk.robomath import transl, rotx, roty, rotz, Pose_2_TxyzRxyz, TxyzRxyz_2_Pose, Mat
    import numpy as np

    RDK = Robolink()
    robot = RDK.Item('', ITEM_TYPE_ROBOT)
    
else:
    from rtde_control import RTDEControlInterface
    rtde_c = RTDEControlInterface(config.arm_ip)

# Motion control
def moveJ(joints, speed, acceleration):
    if config.simulation:
        robot.setSpeed(
            speed_linear = config.speed*1000,
            accel_linear = config.acceleration*1000,
            speed_joints = np.degrees(speed),
            accel_joints = np.degrees(acceleration)
        )
        
        robot.MoveJ(joints, blocking=False)
    else:
        rtde_c.moveJ(
            joints,
            speed,
            acceleration
        )

def moveL(pose, speed, acceleration):
    if config.simulation:
        if isinstance(pose, list) and len(pose) == 16:
            rdk_pose = Mat([
                pose[0:4],
                pose[4:8],
                pose[8:12],
                pose[12:16]
            ])

        else:
            if hasattr(pose, "tolist"):
                pose = pose.tolist()
            rdk_pose = TxyzRxyz_2_Pose(pose)
        robot.setSpeed(speed_linear=speed*1000, accel_linear=acceleration*1000)
        robot.MoveL(rdk_pose)

    else:
        if hasattr(pose, "tolist"):
            pose = pose.tolist()
        rtde_c.moveL(
            pose,
            speed,
            acceleration
        )

def servoL(pose, speed, acceleration, dt, lookahead_time, gain):
    if config.simulation:
        target_pose = TxyzRxyz_2_Pose(pose)
        solutions = robot.SolveIK_All(target_pose)
        cols = len(solutions.Cols())

        best = None
        best_error = float ("inf")

        current = np.array(robot.Joints().list(), dtype=float)

        for i in range(cols):
            q = np.array([
                solutions[0, i],
                solutions[1, i],
                solutions[2, i],
                solutions[3, i],
                solutions[4, i],
                solutions[5, i]
            ], dtype=float)

            error = np.linalg.norm(q - current)
            
            if error < best_error:
                best_error = error
                best = q
            
        if best is None:
            print("IK Failed")
            return
        
        robot.setJoints(best.tolist())

    else:
        if hasattr(pose, "tolist"):
            pose = pose.tolist()

        rtde_c.servoL(
            pose,
            config.bow_speed,
            config.bow_acceleration,
            config.dt,
            config.lookahead_time,
            config.gain
        )

# Cartesian Jogging
last_motion = None

def jogCartesian(selection_vector, wrench):
    global last_motion
    if config.simulation:
        pose = robot.Pose()

        dx = dy = dz = 0.0
        rx = ry = rz = 0.0

        motion = ""

        if selection_vector[0]:
            dx = np.sign(wrench[0]) * config.step_xyz
            motion += "+X " if dx >0 else "-X "
        if selection_vector[1]:
            dy = np.sign(wrench[1]) * config.step_xyz
            motion += "+Y " if dy >0 else "-Y "
        if selection_vector[2]:
            dz =  np.sign(wrench[2]) * config.step_xyz
            motion += "+Z " if dz >0 else "-Z "
        if selection_vector[3]:
            rx = np.sign(wrench[3]) * config.step_rot
            motion += "+Roll " if rx >0 else "-Roll "
        if selection_vector[4]:
            ry = np.sign(wrench[4]) * config.step_rot
            motion += "+Pitch " if ry >0 else "-Pitch "
        if selection_vector[5]:
            rz = np.sign(wrench[5]) * config.step_rot
            motion += "+Yaw " if rz >0 else "-Yaw "
        
        motion = motion.strip()

        if motion != last_motion:
            print(motion)
            last_motion = motion

        pose = pose * transl(dx, dy, dz)
        pose = pose * rotx(rx)
        pose = pose * roty(ry)
        pose = pose * rotz(rz)

        robot.MoveL(pose)
        return
    return

# Force Control
def forceMode(task_frame, selection_vector, wrench, limits):
    if config.simulation:
        return
    rtde_c.forceMode(
        task_frame,
        selection_vector,
        wrench,
        config.force_type,
        limits
    )

# Connection Utilities
def isConnected():
    if config.simulation:
        return True
    return rtde_c.isConnected()

def reconnect():
    if config.simulation:
        return True
    rtde_c.reconnect()
    return rtde_c.isConnected()

# Timing Utilities
def initPeriod():
    if config.simulation:
        return None
    return rtde_c.initPeriod()

def waitPeriod(t_start):
    if config.simulation:
        return
    rtde_c.waitPeriod(t_start)

# Robot State
def getActualTCPPose():
    if config.simulation:
        return Pose_2_TxyzRxyz(robot.Pose())
    return rtde_c.getActualTCPPose()

def getPose():
    if config.simulation:
        return robot.Pose()
    return None

def getJoints():
    if config.simulation:
        return robot.Joints()
    return None

# Robot Shutdown
def stop():
    if config.simulation:
        return
    rtde_c.servoStop()
    rtde_c.forceModeStop()

# Motion Routines
def bowing_segment(start_pose, end_pose, start_joints, end_joints, halt=False):
    if config.simulation:
        if halt:
            stop()
            sys.exit()
        end_pose = np.array(end_pose, dtype=float)

        moveL(
            end_pose.tolist(),
            config.bow_speed,
            config.bow_acceleration
        )

    else:
        for alpha in np.linspace(0.0, 1.0, 500):
            if halt:
                stop()
                sys.exit()
            
            t_start = initPeriod()
       
            pose = (1-alpha)*start_pose+alpha*end_pose

            servoL(
                pose.tolist(),
                config.bow_speed,
                config.bow_acceleration,
                config.dt,
                config.lookahead_time,
                config.gain
            )

            forceMode(
                config.task_frames[config.current_string],
                config.selection_vector,
                config.wrench,
                config.limits
            )
        
            waitPeriod(t_start)