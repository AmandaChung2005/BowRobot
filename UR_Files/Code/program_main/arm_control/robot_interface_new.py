import sys
import socket
import time
import numpy as np
from scipy.spatial.transform import Rotation

import config
import arm_control.arm_config as arm_config
import arm_control.calibration_data as cal


# Initialization (Simulation vs Reality)
if config.simulation:
    sys.path.append(config.robodk_python_path)

    from robodk.robolink import Robolink, ITEM_TYPE_ROBOT
    from robodk.robomath import *

    RDK = Robolink()
    robot = RDK.Item(config.robotName, ITEM_TYPE_ROBOT)

    rtde_r = None
    
else:
    from rtde_receive import RTDEReceiveInterface

    rtde_r = RTDEReceiveInterface(config.host_ip)
    robot = None

def as_list(values):
    if hasattr(values, "tolist"):
        values = values.tolist()
    return [float(v) for v in values]

def format_vector(values):
    values = as_list(values)
    return "[" + ", ".join(f"{float(v):.8f}" for v in values) + "]"

def format_pose(values):
    return "p" + format_vector(as_list(values))

# URScript Communication
script_counter = 0

def send_urscript(script, wait = 0.05):
    global script_counter

    lines = [
        line.strip()
        for line in script.strip().splitlines()
        if line.strip()
    ]

    if not lines:
        return

    if len(lines) > 1 and not lines[0].lstrip().startswith("def "):
        script_counter += 1

        body = "\n".join(
            " " + line
            for line in lines
        )

        function_name = f"urcmd_{script_counter}"

        script = (
            f"def {function_name}():\n"
            f"{body}\n"
            "end\n"
        )

    payload = script

    if not payload.endswith("\n"):
        payload += "\n"

    with socket.create_connection(
        (config.host_ip, config.script_port),
        timeout = 5.0
    ) as sock:
        sock.sendall(payload.encode("utf-8"))

    time.sleep(wait)

def wait_joints(target, tol = 0.01, timeout = 30.0):
    target = np.array(as_list(target), dtype = float)
    t0 = time.time()

    while time.time() - t0 < timeout:
        current = np.array(getCurrentJoints(), dtype = float)

        if np.linalg.norm(current - target) < tol:
            return True
        
        time.sleep(0.05)
    return False

def wait_pose(target, tol = 0.002, timeout = 30.0):
    target = np.array(as_list(target), dtype = float)
    t0 = time.time()

    while time.time() - t0 < timeout:
        current = np.array(getActualTCPPose(), dtype = float)

        if np.linalg.norm(current[:3] - target[:3]) < tol:
            return True
        
        time.sleep(0.05)
    return False


# Motion control
def moveJ(joints, speed, acceleration):    
    print("\nMoving...")

    joints = as_list(joints)
    joints_rad = np.deg2rad(joints)
        
    if config.simulation:
        robot.setSpeed(
            speed_linear = arm_config.speed*1000,
            accel_linear = arm_config.acceleration*1000,
            speed_joints = np.degrees(speed),
            accel_joints = np.degrees(acceleration)
        )

        current = np.array(robot.Joints().list(), dtype = float)

        if np.linalg.norm(current - joints_rad) < np.deg2rad(0.1):
            print("Already at Target Joint Position")
            return
    
        robot.MoveJ(joints_rad.tolist(), blocking=True)

        return


    current = np.asarray(getCurrentJoints(), dtype = float)

    error = np.linalg.norm(current - joints_rad)

    if error < np.deg2rad(2.0):
        print("Already at Target Joint Position")
        return

    print("Sending moveJ...")

    send_urscript(
        f"movej("
        f"{format_vector(joints_rad)}, "
        f"a = {acceleration}, "
        f"v = {speed}"
        f")"
    )

    wait_joints(joints_rad)
    print("moveJ Complete")

def moveJ_safe(
        joints,
        speed,
        acceleration,
        target_pose,
        safety_distance=None
    ):
    print("\nChecking MoveJ Path...")

    joints = np.asarray(
        as_list(joints),
        dtype=float
    )

    target_pose = np.asarray(
        as_list(target_pose),
        dtype=float
    )

    if joints.shape != (6,):
        raise ValueError(
            f"moveJ_safe Expected 6 Joint Values, Got {joints}"
        )

    if target_pose.shape != (6,):
        raise ValueError(
            f"moveJ_safe Expected 6 Pose Values, Got {target_pose}"
        )

    if safety_distance is None:
        safety_distance = (
            arm_config.collision_radius
        )

    current_pose = np.asarray(
        getActualTCPPose(),
        dtype=float
    )

    current_pose_mm = current_pose.copy()

    current_pose_mm[:3] *= 1000.0

    start_pose = current_pose_mm
    end_pose = target_pose.copy()
    end_pose[3:6] = np.deg2rad(end_pose[3:6])

    start = start_pose[:3]
    end = end_pose[:3]

    print(
        "Current TCP:",
        np.round(start, 2)
    )

    print(
        "Target TCP:",
        np.round(end, 2)
    )

    obstacle = find_collision_obstacle(
        start,
        end,
        safety_distance = safety_distance
    )

    if obstacle is not None:

        print(
            "\nMoveJ TCP path blocked."
        )

        waypoint_position = (
            generate_avoidance_waypoint(
                start,
                end,
                obstacle
            )
        )

        waypoint_pose = (
            pose_with_position(
                target_pose,
                waypoint_position
            )
        )

        print(
            "Moving to avoidance waypoint:",
            np.round(
                waypoint_position,
                2
            )
        )

        moveL(
            waypoint_pose.tolist(),
            speed,
            acceleration
        )

        remaining_obstacle = find_collision_obstacle(
            waypoint_position,
            end,
            safety_distance = safety_distance
        )

        if remaining_obstacle is not None:
            print(
                "Waypoint path is still blocked."
            )

            return False

        print(
            "Waypoint clear. Executing MoveJ."
        )

        moveJ(
            joints.tolist(),
            speed,
            acceleration
        )

        return True


    obstacle = find_bow_collision(
        start_pose,
        end_pose,
        safety_distance = safety_distance
    )

    if obstacle is not None:

        print(
            "\nMoveJ bow path blocked."
        )

        waypoint_position = (
            generate_avoidance_waypoint(
                start,
                end,
                obstacle
            )
        )

        waypoint_pose = (
            pose_with_position(
                target_pose,
                waypoint_position
            )
        )

        print(
            "Moving to bow-avoidance waypoint:",
            np.round(
                waypoint_position,
                2
            )
        )

        moveL(
            waypoint_pose.tolist(),
            speed,
            acceleration
        )

        waypoint_start_pose = np.asarray(
            getActualTCPPose(),
            dtype=float
        ).copy()

        waypoint_start_pose[:3] *= 1000.0

        remaining_obstacle = find_bow_collision(
            waypoint_start_pose,
            end_pose,
            safety_distance = safety_distance
        )

        if remaining_obstacle is not None:

            print(
                "Waypoint bow path is still blocked."
            )

            return False

        print(
            "Waypoint clear. Executing MoveJ."
        )

        moveJ(
            joints.tolist(),
            speed,
            acceleration
        )

        return True

    print(
        "MoveJ Path Clear."
    )

    moveJ(
        joints.tolist(),
        speed,
        acceleration
    )

    return True



def moveL(pose, speed, acceleration):
    print("\nMoving...")
    pose = as_list(pose)

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

        robot.setSpeed(speed_linear=speed*1000, accel_linear=acceleration*1000)
        robot.MoveL(rdk_pose)
        return

    target_pose = np.array(
        pose,
        dtype = float
    )

    if target_pose.shape != (6,):
        raise ValueError(
            f"moveL Expected 6 Pose Values, Got {target_pose}: "
            f" shape = {target_pose.shape}"
        )

    current= np.asarray(
        getActualTCPPose(),
        dtype = float
    )

    position_error = np.linalg.norm(
        current[:3] - target_pose[:3] / 1000.0
    )

    rotation_error = np.linalg.norm(
        current[3:6] - np.deg2rad(target_pose[3:6])
    )

    if (
        position_error < 0.001
        and rotation_error < np.deg2rad(1.0)
    ):
        print("Already at Target Cartesian Position")
        return

    target_pose[:3] /= 1000.0
    target_pose[3:6] = np.deg2rad(
        target_pose[3:6]
    )

    send_urscript(
        f"movel("
        f"{format_pose(target_pose)}, "
        f"a = {acceleration}, "
        f"v = {speed}"
        f")"
    )

    wait_pose(target_pose)

    print("moveL Complete")

def moveL_safe(
        pose,
        speed,
        acceleration
):
    pose = np.asarray(as_list(pose), dtype=float)

    if pose.shape != (6,):
        raise ValueError("moveL_safe Expected 6 Pose Values")

    current_pose = np.asarray(getActualTCPPose(), dtype=float)

    current_pose_collision = current_pose.copy()
    current_pose_collision[:3] *= 1000.0

    target_pose_collision = pose.copy()
    target_pose_collision[3:6] = np.deg2rad(target_pose_collision[3:6])

    start = current_pose_collision[:3]
    end = target_pose_collision[:3]

    obstacle = find_collision_obstacle(start, end)

    if obstacle is not None:
        print("\nRouting TCP Around Obstacle")

        waypoint_position = (
            generate_avoidance_waypoint(start, end, obstacle)
        )

        waypoint_pose = (
            pose_with_position(pose, waypoint_position)
        )

        print("TCP Avoidance Waypoint:", np.round(waypoint_position, 2))

        moveL(
            waypoint_pose.tolist(),
            speed,
            acceleration
        )

        moveL(
            pose.tolist(),
            speed,
            acceleration
        )

        return

    obstacle = find_bow_collision(current_pose_collision, target_pose_collision)

    if obstacle is not None:
        print("\nRouting Bow Around Obstacle")
        waypoint_position = (
            generate_avoidance_waypoint(start, end, obstacle)
        )

        waypoint_pose = (
            pose_with_position(pose, waypoint_position)
        )

        print("Routing Bow Around Obstacle")

        moveL(
            waypoint_pose.tolist(),
            speed,
            acceleration
        )

        moveL(
            pose.tolist(),
            speed,
            acceleration
        )

        return

    print("Path Clear")

    moveL(
        pose.tolist(),
        speed,
        acceleration
    )


def servoJ(
        joints,
        acceleration,
        speed,
        dt,
        lookahead_time,
        gain
    ):
    print("\nMoving...")

    joints = np.asarray(as_list(joints), dtype=float)

    if joints.shape != (6,):
        raise ValueError(
            f"servoJ Expected 6 Joint Values, Got {joints}"
        )

    if config.simulation:
        robot.setJoints(joints.tolist())
        return

    send_urscript(
        f"servoj("
        f"{format_vector(joints)}, "
        f"a = {acceleration}, "
        f"v = {speed}, "
        f"t = {dt}, "
        f"lookahead_time = {lookahead_time}, "
        f"gain = {gain}"
        f")",
        wait = 0.0
    )

def servoJ_trajectory(
    joint,
    amplitude_deg,
    half_duration,
    dt = 0.002,
    lookahead_time = 0.1,
    gain = 300
):
    if config.simulation:
        raise RuntimeError(
            "servoJ_trajectory is Only Implemented on the Real Robot"
        )

    if joint not in range(6):
        raise ValueError(
            "Joint Must Be Between 0 and 5"
        )

    if amplitude_deg == 0:
        raise ValueError(
            "amplitude_deg Can't Be 0"
        )

    if half_duration <= 0:
        raise ValueError(
            "half_duration Must Be Positive"
        )

    amplitude = np.deg2rad(amplitude_deg)
    half_steps = max(
        1,
        round(half_duration / dt)
    )

    def target_expression(offset):
        values = [
            f"q_start[{i}]"
            for i in range(6)
        ]

        values[joint] = (
            f"q_start[{joint}] + ({offset})"
        )

        return("[" + ", ".join(values) + "]")

    outbound = target_expression(
        f"{amplitude:.10f} * blend"
    )

    inbound = target_expression(
        f"{amplitude:.10f} * (1.0 - blend)"
    )

    start = target_expression("0.0")
    script = f"""
        def servoj_trajectory():
            q_start = get_actual_joint_positions()
            
            i = 0
            while i <= {half_steps}:
                u = i / {float(half_steps):.1f}
                blend = 10.0*u*u*u - 15.0*u*u*u*u + 6.0*u*u*u*u*u

                servoj(
                    {outbound},
                    t = {dt:.4f},
                    lookahead_time = {lookahead_time:.3f},
                    gain = {gain}
                )

                i = i + 1
            end

            i = 0
            while i <= {half_steps}:
                u = i / {float(half_steps):.1f}
                blend = 10.0*u*u*u - 15.0*u*u*u*u + 6.0*u*u*u*u*u

                servoj(
                    {inbound},
                    t = {dt:.4f},
                    lookahead_time = {lookahead_time:.3f},
                    gain = {gain}
                )

                i = i + 1
            end

            i = 0
            while i < 125:
                servoj(
                    {start},
                    t = {dt:.4f},
                    lookahead_time = {lookahead_time:.3f},
                    gain = {gain}
                )

                i = i + 1
            end

            servoj_trajectory()
    """

    send_urscript(
        script,
        wait = 0.0
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
    
    pose = np.asarray(getActualTCPPose(), dtype = float).copy()

    vx = vy = vz = 0.0
    rx = ry = rz = 0.0

    motion = ""

    speed_xyz = arm_config.speed
    speed_rot = arm_config.joint_speed

    if selection_vector[0]:
        vx = np.sign(wrench[0]) * speed_xyz
        motion += "+X" if vx > 0 else "-X"
    if selection_vector[1]:
        vy = np.sign(wrench[1]) * speed_xyz
        motion += "+Y" if vy > 0 else "-Y"
    if selection_vector[2]:
        vz = np.sign(wrench[2]) * speed_xyz
        motion += "+Z" if vz > 0 else "-Z"
    if selection_vector[3]:
        rx = np.sign(wrench[3]) * speed_rot
        motion += "+Roll" if rx > 0 else "-Roll"
    if selection_vector[4]:
        ry = np.sign(wrench[4]) * speed_rot
        motion += "+Pitch" if ry > 0 else "-Pitch"
    if selection_vector[5]:
        rz = np.sign(wrench[5]) * speed_rot
        motion += "+Yaw" if rz > 0 else "-Yaw"

    motion = motion.strip()

    if motion != last_motion:
        print(f"Moving in {motion}")
        last_motion = motion

    send_urscript(
        f"speedl("
        f"[{vx}, {vy}, {vz}, {rx}, {ry}, {rz}], "
        f"{arm_config.acceleration}, "
        f"0.1"
        f")",
        wait = 0.0
    )

    
    return

# Force Control
def forceMode(
        task_frame,
        selection_vector,
        wrench,
        limits
):
    if config.simulation or not arm_config.useForce:
        return

    task_frame = np.asarray(task_frame, dtype = float).copy()
    task_frame[:3] /= 1000.0
    task_frame[3:6] = np.deg2rad(task_frame[3:6])

    script = (
        "force_mode("
        f"{format_pose(task_frame)}, "
        f"{format_vector(selection_vector)}, "
        f"{format_vector(wrench)}, "
        f"{arm_config.force_type}, "
        f"{format_vector(limits)}"
        ")"
    )

    send_urscript(script)

def forceMode_scaled(
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

    force_wrench = list(as_list(wrench))
    force_wrench[0] = 0.0
    force_wrench[1] = 0.0
    force_wrench[2] = force_z
    force_wrench[3] = 0.0
    force_wrench[4] = 0.0
    force_wrench[5] = 0.0

    force_task_frame = np.asarray(task_frame, dtype=float).copy()
    force_task_frame[:3] /= 1000.0
    force_task_frame[3:6] = np.deg2rad(force_task_frame[3:6])

    script = (
        "force_mode("
        f"{format_pose(force_task_frame)}, "
        f"{format_vector(selection_vector)}, "
        f"{format_vector(force_wrench)}, "
        f"{arm_config.force_type}, "
        f"{format_vector(limits)}"
        ")"
    )

    print(
        f"Scaled Force: {force_z:.3f} N "
        f"(Distance From Frog: {distance:.3f} m)"
    )

    send_urscript(
        script,
        wait = 0.0
    )

def forceModeStop():
    if config.simulation:
        return

    rtde_r.set_force_mode(False)

# Connection Utilities
def isConnected():
    if config.simulation:
        return True
    return rtde_r.isConnected()

def reconnect():
    if config.simulation:
        return True
    return rtde_r.reconnect()

def disconnect():
    if config.simulation:
        return
    rtde_r.disconnect()

# Timing Utilities
def initPeriod():
    if config.simulation:
        return None
    return rtde_r.initPeriod()

def waitPeriod(t_start):
    if config.simulation:
        return
    rtde_r.waitPeriod(t_start)

# Robot State
def getActualTCPPose():
    if config.simulation:
        return Pose_2_TxyzRxyz(robot.Pose())
    return rtde_r.getActualTCPPose()

def getPose():
    if config.simulation:
        return robot.Pose()
    return rtde_r.getActualTCPPose()

def getJoints():
    if config.simulation:
        return robot.Joints()
    return rtde_r.getActualQ()

def getCurrentJoints():
    if config.simulation:
        return np.array(robot.Joints().list(), dtype = float)
    else:
        return np.array(rtde_r.getActualQ(), dtype = float)

# Robot Shutdown
def stop():
    if config.simulation:
        return

    if arm_config.useForce:
        send_urscript(
            "stopl(1.2)\n"
            "end_force_mode()"
        )
    else:
        send_urscript(
            "stopl(1.2)"
        )

# Calculations
def solveIK(pose, reference = None):
    if config.simulation:
        if isinstance(pose, Mat):
            target_pose = pose
        else:
            pose = as_list(pose)
            tool = robot.PoseTool()
            target_pose = TxyzRxyz_2_Pose(pose) * tool.inv() 

        if reference is not None:
            robot.setJoints(reference.tolist())

        q = robot.SolveIK(target_pose)

        if q is None:
            raise RuntimeError(f"IK Failed for Pose: \n{pose}")

        # Verify solution
        robot.setJoints(q)

        return np.array(q.list(), dtype=float)    

    if reference is not None:
        return np.asarray(as_list(reference), dtype = float)

    return getCurrentJoints()

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
    force = desired_force - force_constant * distance_from_frog
    # + damping_constant * speed

    return force


# Collision Avoidance
def point_box_distance(point, obstacle):
    point = np.asarray(point, dtype=float)
    minimum = np.asarray(obstacle["min"], dtype=float)
    maximum = np.asarray(obstacle["max"], dtype=float)
    closest = np.maximum(minimum, np.minimum(point, maximum))

    return np.linalg.norm(point-closest)

def point_line_distance(point, start, end):
    point = np.asarray(point, dtype=float)
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)

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

def get_obstacles():
    if config.simulation:
        return cal.simulation.get(
            "obstacles",
            []
        )
    return cal.real.get("obstacles", [])
def find_collision_obstacle(
        start,
        end,
        safety_distance = None,
        sample_spacing = None
):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)

    if safety_distance is None:
        safety_distance = arm_config.collision_radius

    if sample_spacing is None:
        sample_spacing = arm_config.sample_spacing

    path_length = np.linalg.norm(end - start)

    if path_length < 1e-9:
        num_samples = 1
    else: num_samples = max(
        2,
        int(np.ceil(path_length / sample_spacing)) + 1
    )

    for alpha in np.linspace(
        0.0,
        1.0,
        num_samples
    ):
        tcp_position = ((1.0 - alpha) * start + alpha * end)

        for obstacle in get_obstacles():
            if not isinstance(
                obstacle,
                dict
            ):
                continue

            distance = point_box_distance(
                tcp_position,
                obstacle
            )

            if distance < safety_distance:
                print("\nTCP Collision Detected")
                print("Obstacle:", obstacle)
                print("Distance:", distance)

                return obstacle
    return None

def find_bow_collision(
        start_pose,
        end_pose,
        safety_distance = None,
        sample_spacing = None
):
    start_pose = np.asarray(start_pose, dtype = float)
    end_pose = np.asarray(end_pose, dtype=float)

    if start_pose.shape != (6,):
        raise ValueError("find_bow_collision start_pose Must Contain 6 Values")

    if end_pose.shape != (6,):
            raise ValueError("find_bow_collision end_pose Must Contain 6 Values")

    if safety_distance is None:
        safety_distance = (arm_config.collision_radius)

    if sample_spacing is None:
        sample_spacing =(arm_config.sample_spacing)

    start_position = start_pose[:3]
    end_position = end_pose[:3]

    path_length = np.linalg.norm(end_position - start_position)

    if path_length < 1e-9:
        num_samples = 1
    else:
        num_samples = max(
            2,
            int(np.ceil(path_length/sample_spacing) + 1)
        )

    start_rotvec = start_pose[3:6]
    end_rotvec = end_pose[3:6]

    for alpha in np.linspace(
        0.0,
        1.0,
        num_samples
    ):
        tcp_position = (
            (1.0 - alpha) * start_position + alpha * end_position
        )

        rotvec = (
            (1.0 - alpha) * start_rotvec + alpha * end_rotvec
        )

        R_tcp = Rotation.from_rotvec(rotvec).as_matrix()

        bow_direction = (
            R_tcp @ np.array([
                -1.0,
                0.0,
                0.0
            ])
        )

        bow_direction /= np.linalg.norm(bow_direction)

        bow_start = tcp_position.copy()

        bow_end = (bow_start + bow_direction * arm_config.bow_length)

        bow_length = np.linalg.norm(bow_end - bow_start)

        bow_samples = max(
            2,
            int(np.ceil(bow_length / sample_spacing)) + 1
        )

        for beta in np.linspace(
            0.0,
            1.0,
            bow_samples
        ):
            bow_point = (
                (1.0 - beta) * bow_start + beta * bow_end
            )

            for obstacle in get_obstacles():
                if not isinstance(obstacle, dict):
                    continue

                minimum = np.asarray(obstacle["min"], dtype=float)
                maximum = np.asarray(obstacle["max"], dtype=float)
                closest = np.maximum(minimum, np.minimum(bow_point, maximum))

                distance = np.linalg.norm(bow_point - closest)

                if distance < safety_distance:
                    print("\nBow Collision Detected")
                    print("Bow Point:", np.round(bow_point, 2))
                    print("Obstacle:", obstacle)
                    print("Distance:", distance)
                    return obstacle

    return None

def generate_avoidance_waypoint(start, end, obstacle):
    start = np.asarray(start, dtype=float)
    end = np.asarray(end, dtype=float)
    minimum = np.asarray(obstacle["min"], dtype=float)
    maximum = np.asarray(obstacle["max"], dtype=float)

    direction = end - start
    direction_norm = np.linalg.norm(direction)

    if direction_norm < 1e-9:
        return start.copy()

    direction = direction / direction_norm
    center = (minimum + maximum) / 2.0

    projection = np.dot(
        center - start,
        direction
    )

    closest = start + projection * direction

    distances = np.array([
        abs(closest[0] - minimum[0]),
        abs(maximum[0] - closest[0]),
        abs(closest[1] - minimum[1]),
        abs(maximum[1] - closest[1]),
        abs(closest[2] - minimum[2]),
        abs(maximum[2] - closest[2]),
    ])

    face = int(np.argmin(distances))
    waypoint = closest.copy()
    margin = (
        arm_config.collision_radius + arm_config.waypoint_offset
    )

    if face == 0:
        waypoint[0] = minimum[0] - margin

    elif face == 1:
        waypoint[0] = maximum[0] + margin

    elif face == 2:
        waypoint[1] = minimum[1] - margin

    elif face == 3:
        waypoint[1] = maximum[1] + margin

    elif face == 4:
        waypoint[2] = minimum[2] - margin

    elif face == 5:
        waypoint[2] = maximum[2] + margin

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
    if halt:
        stop()
        sys.exit()

    if config.simulation:
        end_pose = np.array(end_pose, dtype = float)

        moveL(
            end_pose.tolist(),
            arm_config.bow_speed,
            arm_config.bow_acceleration
        )

        return

    if rosin:
        task_frame = np.asarray(
            arm_config.rosin_task_frame,
            dtype = float
        )
        selection_vector = arm_config.rosin_selection_vector
        wrench = arm_config.rosin_wrench
        limits = arm_config.rosin_limits
    else:
        current_string = arm_config.current_string
        task_frame = np.asarray(
            arm_config.task_frames[current_string],
            dtype = float
        )
        selection_vector = arm_config.selection_vector
        wrench = arm_config.wrench
        limits = arm_config.limits

    start_pose = np.asarray(as_list(start_pose), dtype=float)
    end_pose = np.asarray(as_list(end_pose), dtype=float)

    start_joints = np.asarray(start_joints, dtype=float)
    end_joints = np.asarray(end_joints, dtype=float)

    if start_pose.shape != (6,):
        raise ValueError("Starting Pose Must Containg 6 Values")

    if end_pose.shape != (6,):
        raise ValueError("Ending Pose Must Containg 6 Values")

    if start_joints.shape != (6,):
        raise ValueError("Starting Joints Must Containg 6 Values")

    if end_joints.shape != (6,):
        raise ValueError("Ending Joints Must Containg 6 Values")

    obstacle = find_bow_collision(
        start_pose,
        end_pose
    )

    if obstacle is not None:
        print("\nBow Stroke Blocked by Obstacle")

        waypoint_position = (
            generate_avoidance_waypoint(
                start_pose[:3],
                end_pose[:3],
                obstacle
            )
        )

        waypoint_pose = pose_with_position(
            end_pose,
            waypoint_position
        )

        print(
            "Routing Bow Through Waypoint:",
            waypoint_position
        )

        moveL(
            waypoint_pose.tolist(),
            arm_config.speed,
            arm_config.acceleration
        )

        moveL(
            end_pose.tolist(),
            arm_config.speed,
            arm_config.acceleration
        )

        return

    if arm_config.useForce:
        if rosin:
            forceMode_scaled(
                task_frame,
                selection_vector,
                wrench,
                limits,
                arm_config.force_constant,
                arm_config.bow_force,
                rosin = True
            )
        else:
            forceMode_scaled(
                task_frame,
                selection_vector,
                wrench,
                limits,
                arm_config.force_constant,
                arm_config.bow_force,
                rosin = False,
                string = current_string
            )

    target_pose = end_pose.copy()
    target_pose[:3] /= 1000.0
    target_pose[3:6] = np.deg2rad(target_pose[3:6])

    force_task_frame = task_frame.copy()
    force_task_frame[:3] /= 1000.0
    force_task_frame[3:6] = np.deg2rad(force_task_frame[3:6])

    stroke = (
        f"movel({format_pose(target_pose)}, "
        f"a = {arm_config.bow_acceleration}, "
        f"v = {arm_config.bow_speed})"
    )

    send_urscript(stroke, wait = 0.01)
    wait_pose(target_pose)

    if arm_config.useForce:
        send_urscript(
            "end_force_mode()",
            wait = 0.0
        )




        # for alpha in np.linspace(0.0, 1.0, 2000):
        #     if halt:
        #         stop()
        #         sys.exit()
            
        #     t_start = initPeriod()
       
        #     pose = (1 - alpha) * start_pose + alpha * end_pose

        #     joints = (
        #         (1 - alpha) * start_joints 
        #         + alpha * end_joints
        #     )

        #     servoJ(
        #         joints.tolist(),
        #         arm_config.bow_acceleration,
        #         arm_config.bow_speed,
        #         arm_config.dt,
        #         arm_config.lookahead_time,
        #         arm_config.gain
        #     )
        
        #     waitPeriod(t_start)

        # forceModeStop()

def getRobot():
    if config.simulation:
        return robot
    return None