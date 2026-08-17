import sys
import numpy as np
from scipy.spatial.transform import Rotation

import config
import arm_control.arm_config as arm_config
import arm_control.calibration_data as cal
import RTDE.code.control_loop as control_loop


# Initialization (Simulation vs Reality)
if config.simulation:
    sys.path.append(config.robodk_python_path)

    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
    from robodk.robomath import *

    RDK = Robolink()
    robot = RDK.Item(config.robotName, ITEM_TYPE_ROBOT)
    
else:
    rtde = control_loop.RTDEInterface()
    rtde.connect()

# Motion control
def moveJ(joints, velocity, acceleration):    
    if hasattr(joints, "tolist"):
        joints = joints.tolist()

    target_deg = np.asarray(joints, dtype = float)
        
    if config.simulation:
        robot.setSpeed(
            speed_linear = arm_config.speed*1000,
            accel_linear = arm_config.acceleration*1000,
            speed_joints = np.degrees(velocity),
            accel_joints = np.degrees(acceleration)
        )

        current = np.array(robot.Joints().list(), dtype = float)

        if np.linalg.norm(current - target_deg) < 0.1:
            print("Already at Target Joint Position")
            return
    
        robot.MoveJ(target_deg.tolist(), blocking=True)

    else:
        current = np.array(rtde.getActualQ(), dtype = float)

        difference = np.degrees(current) - target_deg
        error = np.linalg.norm(current - np.deg2rad(target_deg))

        if error < np.deg2rad(2.0):
            print("Already at Target Joint Position")
            return

        print("Sending moveJ...")

        rtde.moveJ(
            target_deg.tolist(),
            velocity,
            acceleration
        )

        rtde.wait_for_move()
        print("moveJ Complete")

def moveL(pose, velocity, acceleration):
    if hasattr(pose, "tolist"):
        pose = pose.tolist()

    if config.simulation:
        if isinstance(pose, list) and len(pose) == 16:
            rdk_pose = Mat([
                pose[0:4],
                pose[4:8],
                pose[8:12],
                pose[12:16]
            ])
        else:
            rdk_pose = TxyzRxyz_2_Pose(pose)

        robot.setSpeed(speed_linear=velocity*1000, accel_linear=acceleration*1000)
        robot.MoveL(rdk_pose)

    else:
        pose_array = np.asarray(pose, dtype=float).copy()

        if pose_array.shape != (6,):
            raise ValueError(
                f"moveL Expected 6 Pose Values, Got {pose_array}: "
                f"shape = {pose_array.shape}"
            )
        pose_array[:3] /= 1000.0
        pose_array[3:6] = np.deg2rad(pose_array[3:6])

        rtde.moveL(
            pose_array.tolist(),
            velocity,
            acceleration
        )

        rtde.wait_for_move()

def moveL_safe(
        pose,
        velocity,
        acceleration
):
    pose = np.asarray(
        pose,
        dtype = float
    )

    current_pose = np.asarray(
        getActualTCPPose(),
        dtype = float
    )

    start = current_pose[:3]
    end = pose[:3]

    obstacle = find_collision_obstacle(
        start,
        end
    )

    if obstacle is not None:
        print(
            "TCP Collision Risk Detected Near Obstacle: ",
            obstacle
        )

        waypoint_position = generate_avoidance_waypoint(
            start,
            end,
            obstacle
        )

        waypoint_pose = pose_with_position(
            pose,
            waypoint_position
        )

        print(
            "Routing Through Waypoint: ",
            waypoint_position
        )

        moveL(
            waypoint_pose,
            velocity,
            acceleration
        )

        moveL(
            pose,
            velocity,
            acceleration
        )

        return

    bow_direction = get_bow_direction()

    for alpha in np.linspace(0.0, 1.0, 20):
        tcp_position = (
            (1.0 - alpha) * start + alpha * end
        )

        obstacle = bow_collision_obstacle(
            tcp_position,
            bow_direction
        )

        if obstacle is not None:
            print(
                "Bow Collsion Risk Detecetd Near Obstacle: ",
                obstacle
            )

            waypoint_position = generate_avoidance_waypoint(
                start,
                end,
                obstacle
            )

            waypoint_pose = pose_with_position(
                pose,
                waypoint_position
            )

            print(
                "Routing Bow Through Waypoint: ",
                waypoint_position
            )

            moveL(
                waypoint_pose,
                velocity,
                acceleration
            )

            moveL(
                pose,
                velocity,
                acceleration
            )

            return

        print("Path Clear")

        moveL(
            pose,
            velocity,
            acceleration
        )


def servoJ(joints,acceleration, velocity, dt, lookahead_time, gain):
    print("\nMoving...")

    if hasattr(joints, "tolist"):
        joints = joints.tolist()

    if config.simulation:
        target_pose = TxyzRxyz_2_Pose(joints)
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
        rtde.servoJ(
            joints,
            velocity,
            acceleration,
            dt,
            lookahead_time,
            gain
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
            dx = np.sign(wrench[0]) * arm_config.step_xyz
            motion += "+X " if dx >0 else "-X "
        if selection_vector[1]:
            dy = np.sign(wrench[1]) * arm_config.step_xyz
            motion += "+Y " if dy >0 else "-Y "
        if selection_vector[2]:
            dz =  np.sign(wrench[2]) * arm_config.step_xyz
            motion += "+Z " if dz >0 else "-Z "
        if selection_vector[3]:
            rx = np.sign(wrench[3]) * arm_config.step_rot
            motion += "+Roll " if rx >0 else "-Roll "
        if selection_vector[4]:
            ry = np.sign(wrench[4]) * arm_config.step_rot
            motion += "+Pitch " if ry >0 else "-Pitch "
        if selection_vector[5]:
            rz = np.sign(wrench[5]) * arm_config.step_rot
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
def forceMode(
    task_frame,
    selection_vector,
    wrench,
    limits,
    force_constant,
    desired_force,
    rosin = False,
    string = None
):
    if config.simulation:
        return

    if rosin:
        distance = distance_from_frog(
            rosin = True
        )
    else:
        distance = distance_from_frog(
            rosin = False,
            string = string
        )

    force_z = scale_force_with_distance(
        desired_force = desired_force,
        distance_from_frog = distance,
        force_constant = force_constant
    )

    force_wrench = list(wrench)
    force_wrench[0] = 0.0
    force_wrench[1] = 0.0
    force_wrench[2] = force_z
    force_wrench[3] = 0.0
    force_wrench[4] = 0.0
    force_wrench[5] = 0.0

    rtde.set_force_parameters(
        task_frame,
        selection_vector,
        force_wrench,
        arm_config.force_type,
        limits
    )

    rtde.set_force_mode(True)

def forceModeStop():
    if config.simulation:
        return

    rtde.set_force_mode(False)

# Connection Utilities
def isConnected():
    if config.simulation:
        return True
    return rtde.isConnected()

def reconnect():
    if config.simulation:
        return True
    return rtde.reconnect()

def disconnect():
    if config.simulation:
        return
    rtde.disconnect()

# Timing Utilities
def initPeriod():
    if config.simulation:
        return None
    return rtde.initPeriod()

def waitPeriod(t_start):
    if config.simulation:
        return
    rtde.waitPeriod(t_start)

# Robot State
def getActualTCPPose():
    if config.simulation:
        return Pose_2_TxyzRxyz(robot.Pose())
    return rtde.getActualTCPPose()

def getPose():
    if config.simulation:
        return robot.Pose()
    return rtde.getActualTCPPose()

def getJoints():
    if config.simulation:
        return robot.Joints()
    return rtde.getActualQ()

def getCurrentJoints():
    if config.simulation:
        return np.array(robot.Joints().list(), dtype = float)
    else:
        return np.array(rtde.getActualQ(), dtype = float)

# Robot Shutdown
def stop():
    if config.simulation:
        return
    rtde.stop()

# Calculations
def solveIK(pose, reference = None):
    if config.simulation:
        if isinstance(pose, Mat):
            target_pose = pose
        else:
            if hasattr(pose, "tolist"):
                pose = pose.tolist()

            tool = robot.PoseTool()
            target_pose = TxyzRxyz_2_Pose(pose) * tool.inv()

        solutions = robot.SolveIK_All(target_pose)    


        if reference is not None:
            robot.setJoints(reference.tolist())

        q = robot.SolveIK(target_pose)

        if q is None:
            raise RuntimeError(f"IK Failed for Pose: \n{pose}")

        # Verify solution
        robot.setJoints(q)

        actual = np.array(Pose_2_TxyzRxyz(robot.Pose()), dtype=float)
        target = np.array(pose, dtype=float)

        return np.array(q.list(), dtype=float)    

    return None

def get_middle_pose(string):
    frog = np.array(arm_config.data["string_paths"][string]["frog"])
    tip = np.array(arm_config.data["string_paths"][string]["tip"])
    return 0.5 * (frog + tip)

def get_middle_joints(string):
    middle_pose = get_middle_pose(string)

    frog_joints = np.array(
        arm_config.data["joint_paths"][string]["frog"],
        dtype = float
    )

    return solveIK(middle_pose, reference = frog_joints)

def distance_from_frog(
    string = None,
    rosin = True
):
    if rosin:
        frog_pose = np.asarray(
            arm_config.rosin_task_frame,
            dtype = float
        )
    else:
        frog_pose = np.asarray(
            arm_config.task_frames[string],
            dtype = float
        )

    tcp_pose = np.asarray(
        getActualTCPPose(),
        dtype = float
    )

    delta_base = tcp_pose[:3] - frog_pose[:3]

    R_task = Rotation.from_rotvec(
        frog_pose[3:6]
    ).as_matrix()

    delta_task = R_task.T @ delta_base

    distance = delta_task[0]

    return distance

def scale_force_with_distance(
    desired_force,
    distance_from_frog,
    force_constant
):
    force = desired_force + force_constant * distance_from_frog
    # + damping_constant * velocity

    return force


# Collision Avoidance
def point_line_distance(point, start, end):
    point = np.asarray(point, dtype = float)
    start = np.asarray(start, dtype = float)
    end = np.asarray(end, dtype = float)

    line = end - start
    line_length_sq = np.dot(line, line)

    if line_length_sq < 1e-12:
        return np.linalg.norm(point - start)

    t = np.dot(point - start, line) / line_length_sq
    t = np.clip(t, 0.0, 1.0)

    closest = start + t * line

    return np.linalg.norm(point - closest)

def get_bow_direction():
    tcp_pose = np.asarray(
        getActualTCPPose(),
        dtype = float
    )

    rotation_vector = tcp_pose[3:6]

    R_tcp = Rotation.from_rotvec(
        rotation_vector
    ).as_matrix()

    local_direction = np.array([
        -1.0,
        0.0,
        0.0
    ])

    bow_direction = R_tcp @ local_direction

    return bow_direction / np.linalg.norm(bow_direction)

def bow_line(
    tcp_position,
    bow_direction
):
    tcp_position = np.asarray(tcp_position, dtype = float)
    bow_direction = np.asarray(bow_direction, dtype = float)

    norm = np.linalg.norm(bow_direction)

    if norm < 1e-12:
        raise ValueError("Bow Direction Can't Be Zero")

    bow_direction = bow_direction / norm
    bow_tip = tcp_position + bow_direction * arm_config.bow_length

    return tcp_position, bow_tip


def find_collision_obstacle(
        start,
        end,
        safety_distance = None,
        num_samples = 50
):
    start = np.asarray(start, dtype = float)
    end = np.asarray(end, dtype = float)

    if safety_distance is None:
        safety_distance = arm_config.collision_radius

    bow_direction = get_bow_direction()

    for alpha in np.linspace(0.0, 1.0, num_samples):
        tcp_position = ((1.0-alpha)* start + alpha * end)

    for obstacle in arm_config.obstacles:
        obstacle = np.asarray(obstacle, dtype = float)
        tcp_distance = np.linalg.norm(obstacle - tcp_position)

        if tcp_distance < safety_distance:
            print(
                "TCP Collision Detected: ",
                obstacle,
                "distance: ",
                tcp_distance
            )

            return obstacle

    
    bow_start, bow_end = bow_line(
        tcp_position,
        bow_direction
    )

    for obstacle in arm_config.obstacles:
        obstacle = np.asarray(
            obstacle,
            dtype = float
        )

        bow_distance = point_line_distance(
            obstacle,
            bow_start,
            bow_end
        )

        if bow_distance < safety_distance:
            print(
                "Bow Collision Detected: ",
                obstacle,
                "Distance: ",
                bow_distance
            )

            return obstacle

    return None

def generate_avoidance_waypoint(start, end, obstacle):
    start = np.asarray(start, dtype = float)
    end = np.asarray(end, dtype = float)
    obstacle = np.asarray(obstacle, dtype = float)

    direction = end - start
    direction_norm = np.linalg.norm(direction)

    if direction_norm < 1e-9:
        return start.copy()

    direction = direction / direction_norm
    obstacle_relative = obstacle - start

    projection = np.dot(
        obstacle_relative,
        direction
    )

    closest = start + projection * direction
    away = closest - obstacle
    away_norm = np.linalg.norm(away)

    if away_norm < 1e-9:
        axes = [
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 1.0, 0.0]),
            np.array([0.0, 0.0, 1.0])
        ]

        reference = min(
            axes,
            key = lambda axis: abs(np.dot(axis, direction))
        )

        away = np.cross(direction, reference)
        away_norm = np.linalg.norm(away)

    away = away / away_norm

    waypoint = closest + away * (
        arm_config.collision_radius + arm_config.waypoint_offset
    )

    return waypoint

def pose_with_position(pose, position):
    new_pose = np.asarray(
        pose,
        dtype = float
    ).copy()

    new_pose[:3] = position

    return new_pose

# Motion Routines
def bowing_segment(
        start_pose,
        end_pose,
        start_joints,
        end_joints,
        halt=False,
        rosin = False
    ):

    start_pose = np.asarray(start_pose, dtype = float)
    end_pose = np.asarray(end_pose, dtype = float)

    start_joints = np.asarray(start_joints, dtype = float)
    end_joints = np.asarray(end_joints, dtype = float)

    if start_pose.shape != (6,):
        raise ValueError("Starting Pose Must Containg 6 Values")

    if end_pose.shape != (6,):
        raise ValueError("Ending Pose Must Containg 6 Values")

    if start_joints.shape != (6,):
        raise ValueError("Starting Joints Must Containg 6 Values")

    if end_joints.shape != (6,):
        raise ValueError("Ending Joints Must Containg 6 Values")

    if config.simulation:
        if halt:
            stop()
            sys.exit()

        moveL(
            end_pose.tolist(),
            arm_config.bow_speed,
            arm_config.bow_acceleration
        )

    else:
        if rosin:
            task_frame = arm_config.rosin_task_frame
            selection_vector = arm_config.rosin_selection_vector
            wrench = arm_config.rosin_wrench
            limits = arm_config.rosin_limits
        else:
            task_frame = arm_config.task_frames[
                arm_config.current_string
            ]
            selection_vector = arm_config.selection_vector
            wrench = arm_config.wrench
            limits = arm_config.limits

        # forceMode(
        #     task_frame,
        #     selection_vector,
        #     wrench,
        #     limits
        # )

        for alpha in np.linspace(0.0, 1.0, 2000):
            if halt:
                stop()
                sys.exit()
            
            t_start = initPeriod()
       
            pose = (1 - alpha) * start_pose + alpha * end_pose

            joints = (
                (1 - alpha) * start_joints 
                + alpha * end_joints
            )

            servoJ(
                joints.tolist(),
                arm_config.bow_acceleration,
                arm_config.bow_velocity,
                arm_config.dt,
                arm_config.lookahead_time,
                arm_config.gain
            )
        
            waitPeriod(t_start)

        # forceModeStop()

def getRobot():
    if config.simulation:
        return robot
    return None