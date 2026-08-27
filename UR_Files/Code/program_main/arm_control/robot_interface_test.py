import sys
import socket
import time
import numpy as np
import threading
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
    from rtde_control import RTDEControlInterface

    rtde_r = RTDEReceiveInterface(config.host_ip)
    rtde_c = RTDEControlInterface(config.host_ip)
    robot = None
    force_offset = np.zeros(6)

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

    script = script.strip()

    if not script:
        return

    lines = [
        line.strip()
        for line in script.strip().splitlines()
        if line.strip()
    ]
    
    starts_top_level = lines[0].strip().startswith(("def ", "thread "))

    if len(lines) > 1 and not starts_top_level:
        script_counter += 1

        function_name = f"urcmd_{script_counter}"

        body = "\n".join(
            " " + line
            for line in lines
        )

        script = (
            f"def {function_name}():\n"
            f"{body}\n"
            f"end\n"
            f"{function_name}()\n"
        )


    if not script.endswith("\n"):
        script += "\n"

    with socket.create_connection(
        (config.host_ip, config.script_port),
        timeout = 5.0
    ) as sock:
        sock.sendall(script.encode("utf-8"))

    time.sleep(wait)

def send_motion_script(
    script,
    register = 12,
    timeout = 30.0
):
    if config.simulation:
        send_urscript(script, wait = 0.0)
        return

    send_urscript(
        f"write_output_integer_register({register}, 0)",
        wait = 0.0
    )

    send_urscript(script, wait = 0.0)

    if not wait_script_complete(register, timeout):
        raise RuntimeError("Robot Motion Didn't Report Completion")

def wait_joints(target, tol = 0.01, timeout = 30.0):
    target = np.array(as_list(target), dtype = float)
    t0 = time.time()

    while time.time() - t0 < timeout:
        current = np.array(getCurrentJoints(), dtype = float)

        if np.linalg.norm(current - target) < tol:
            return True
        
        time.sleep(0.05)
    return False

def wait_pose(
        target,
        tol = 0.002,
        timeout = 30.0,
        task_frame = None,
        selection_vector = None
    ):
    target = np.array(as_list(target), dtype = float)
    t0 = time.time()

    if task_frame is not None and selection_vector is not None:
        R_task = Rotation.from_rotvec(
            np.asarray(task_frame[3:6],
            dtype = float)
        ).as_matrix()

        free_axes = np.array([bool(s) for s in selection_vector[:3]])

    else:
        R_task = np.eye(3)
        free_axes = np.zeros(3, dtype = bool)

    while time.time() - t0 < timeout:
        current = np.array(getActualTCPPose(), dtype = float)

        delta_base = current[:3] - target[:3]
        delta_task = R_task.T @ delta_base
        delta_task[free_axes] = 0.0

        if np.linalg.norm(delta_task) < tol:
            return True

        time.sleep(0.05)
    return False

def start_force_monitor(
    task_frame,
    frog_pose,
    tip_pose,
    interval = 0.05
):
    if config.simulation:
        return None

    stop_event = threading.Event()
    pause_event = threading.Event()

    task_frame = np.asarray(
        task_frame,
        dtype = float
    )

    def monitor():
        while not stop_event.is_set():
            if pause_event.is_set():
                stop_event.wait(interval)
                continue

            try:
                distance = distance_from_frog(frog_pose, tip_pose)

                force_magnitude = (
                    arm_config.bow_force + arm_config.force_constant * distance
                )

                force_magnitude = np.clip(
                    force_magnitude,
                    0.0,
                    arm_config.max_force
                )

                commanded_force = -force_magnitude

                measured_wrench = np.asarray(
                    rtde_r.getActualTCPForce(),
                    dtype = float
                ) - force_offset

                R_task = Rotation.from_rotvec(
                    np.deg2rad(task_frame[3:6])
                ).as_matrix()

                measured_force_base = measured_wrench[:3]
                measured_force_task = R_task.T @ measured_force_base
                measured_force = measured_force_task[2]

                print(
                    f"Commanded: {commanded_force:7.3f} N | "
                    f"Measured: {measured_force:7.3f} N | "
                    f"Distance: {distance:7.4f} m",
                    flush = True
                )

            except Exception as e:
                if not pause_event.is_set():
                    print(
                        f"Force Monitor Error: {e}",
                        flush = True
                    )

            stop_event.wait(interval)
    thread = threading.Thread(target = monitor, daemon = True)
    thread.start()
    return {
        "thread": thread,
        "stop_event": stop_event,
        "pause_event": pause_event
    }

def pause_force_monitor(force_monitor):
    if force_monitor is not None:
        force_monitor["pause_event"].set()

def resume_force_monitor(force_monitor):
    if force_monitor is not None:
        force_monitor["pause_event"].clear()

def stop_force_monitor(force_monitor):
    if force_monitor is None:
        return
    force_monitor["stop_event"].set()
    force_monitor["thread"].join(timeout = 1.0)
    print()
    
def zero_force_sensor(
    samples = 20,
    delay = 0.01
):
    if config.simulation:
        return np.zeros(6)
    print("\nZeroing Force Sensor...")
    readings = []

    for _ in range(samples):
        readings.append(np.asarray(
            rtde_r.getActualTCPForce(),
            dtype = float
        ))

        time.sleep(delay)

    offset = np.mean(
        np.asarray(readings),
        axis = 0
    )

    return offset

def set_force_offset(offset):
    global force_offset

    force_offset = np.asarray(offset, dtype = float)

def get_force_offset():
    return force_offset.copy()

# Motion control
def moveJ(joints, speed, acceleration):    
    print("\nMoving...")

    joints = as_list(joints)
    joints_rad = np.deg2rad(joints)
        
    if config.simulation:
        target_joints = np.asarray(
            joints,
            dtype=float
        )

        print("\nTarget Joints (deg):")
        print(
            np.round(
                target_joints,
                3
            )
        )

        # Convert degrees to radians for RoboDK FK
        target_joints_rad = np.deg2rad(
            target_joints
        )

        # RoboDK SolveFK expects a RoboDK Mat
        target_joints_mat = Mat(
            target_joints_rad.tolist()
        )

        tool_pose = robot.PoseTool()

        print("\nActive RoboDK Tool Pose:")
        print(tool_pose)

        # FK including the TCP/tool
        fk_pose = robot.SolveFK(
            target_joints_mat,
            tool_pose
        )

        fk_txyzrxyz = Pose_2_TxyzRxyz(
            fk_pose
        )

        print("\nFK TCP Pose From Target Joints:")
        print(
            np.round(
                fk_txyzrxyz,
                3
            )
        )

        print("\nStored Rosin Tip Calibration Pose:")
        print(
            np.round(
                arm_config.rosin_position["tip"],
                3
            )
        )

        print("\nMoving To Target Joints...")

        

        robot.setSpeed(
            speed_linear = arm_config.speed*1000,
            accel_linear = arm_config.acceleration*1000,
            speed_joints = np.degrees(speed),
            accel_joints = np.degrees(acceleration)
        )

        current = np.array(robot.Joints().list(), dtype = float)

        if np.linalg.norm(current - np.array(joints, dtype = float)) < 0.1:
            print("Already at Target Joint Position")
            return

        robot.MoveJ(target_joints.tolist(), blocking = True)

        actual_pose = Pose_2_TxyzRxyz(
            robot.Pose()
        )

        print("\nActual RoboDK Pose After MoveJ:")
        print(
            np.round(
                actual_pose,
                3
            )
        )

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
        safety_distance = None
    ):
    print("\nChecking MoveJ Path...")

    joints = np.asarray(as_list(joints), dtype = float)

    target_pose = np.asarray(as_list(target_pose), dtype = float)

    if joints.shape != (6,):
        raise ValueError(
            f"moveJ_safe Expected 6 Joint Values, Got {joints}"
        )

    if target_pose.shape != (6,):
        raise ValueError(
            f"moveJ_safe Expected 6 Pose Values, Got {target_pose}"
        )

    start_pose = current_pose_mm()
    end_pose = collision_pose(target_pose)

    if not resolve_bow_path(
        start_pose,
        end_pose,
        target_pose,
        speed,
        acceleration,
        safety_distance = safety_distance
    ):
        return False

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
            sim_pose = list(pose)
            sim_pose[3:6] = np.deg2rad(sim_pose[3:6]).tolist()
            rdk_pose = TxyzRxyz_2_Pose(sim_pose)


        robot.setSpeed(
            speed_linear = speed * 1000,
            accel_linear = acceleration * 1000
        )
        robot.MoveL(rdk_pose)
        return

    target_pose = np.array(pose, dtype = float)

    if target_pose.shape != (6,):
        raise ValueError(
            f"moveL Expected 6 Pose Values, Got {target_pose}: "
            f" shape = {target_pose.shape}"
        )

    current= np.asarray(getActualTCPPose(), dtype = float)

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
        acceleration,
        safety_distance = None
):
    pose = np.asarray(as_list(pose), dtype = float)

    if pose.shape != (6,):
        raise ValueError("moveL_safe Expected 6 Pose Values")

    start_pose = current_pose_mm()
    end_pose = collision_pose(pose)

    if not resolve_bow_path(
        start_pose,
        end_pose,
        pose,
        speed,
        acceleration,
        safety_distance = safety_distance
    ):
        print("\nUnable to Navigate Around Obstacle - Not Moving")
        return False

    print("Path Clear")

    moveL(
        pose.tolist(),
        speed,
        acceleration
    )

    return True

def servoJ(
        joints,
        acceleration,
        speed,
        dt,
        lookahead_time,
        gain
    ):
    print("\nMoving...")

    joints = np.asarray(as_list(joints), dtype = float)

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
        raise ValueError("Joint Must Be Between 0 and 5")

    if amplitude_deg == 0:
        raise ValueError("amplitude_deg Can't Be 0")

    if half_duration <= 0:
        raise ValueError("half_duration Must Be Positive")

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

    time.sleep(0.05)

def forceMode_scaled(
    task_frame,
    selection_vector,
    wrench,
    limits,
    force_constant,
    desired_force,
    frog_pose,
    tip_pose,
    rosin = False
):
    if config.simulation or not arm_config.useForce:
        return

    distance = distance_from_frog(frog_pose, tip_pose)

    force_z = scale_force_with_distance(
        desired_force = arm_config.bow_force,
        distance_from_frog = distance,
        force_constant = force_constant
    )

    force_z = np.clip(
        force_z,
        -arm_config.max_force,
        arm_config.max_force
    )

    force_wrench = list(as_list(wrench))
    force_wrench[0] = 0.0
    force_wrench[1] = 0.0
    force_wrench[2] = force_z
    force_wrench[3] = 0.0
    force_wrench[4] = 0.0
    force_wrench[5] = 0.0

    forceMode(
        task_frame,
        selection_vector,
        force_wrench,
        limits
    )

    print(
        f"Scaled Force: {force_z:.3f} N"
        f"(Distance: {distance:.3f} m)"
    )

def prepare_force(
    task_frame,
    selection_vector,
    wrench,
    limits,
    force_constant,
    desired_force,
    frog_pose,
    tip_pose
):
    if config.simulation or not arm_config.useForce:
        return True

    distance = distance_from_frog(frog_pose, tip_pose)
    force_z = scale_force_with_distance(
        desired_force = desired_force,
        distance_from_frog = distance,
        force_constant = force_constant
    )
    force_z = np.clip(
        force_z,
        -arm_config.max_force,
        arm_config.max_force
    )

    force_wrench = list(as_list(wrench))
    force_wrench[0] = 0.0
    force_wrench[1] = 0.0
    force_wrench[2] = force_z
    force_wrench[3] = 0.0
    force_wrench[4] = 0.0
    force_wrench[5] = 0.0

    ur_task_frame = np.asarray(task_frame, dtype = float).copy()
    ur_task_frame[:3] /= 1000.0
    ur_task_frame[3:6] = np.deg2rad(ur_task_frame[3:6])

    tolerance = arm_config.force_tolerance
    timeout = arm_config.force_prepare_timeout

    R_task = Rotation.from_rotvec(
        np.deg2rad(task_frame[3:6])
    ).as_matrix()

    task_z = R_task[:, 2]


    print("\nPreparing Force Mode")

    send_urscript(
        "write_output_integer_register(12, 0)",
        wait = 0.0
    )

    script = f"""
    def prepare_force():
    
        force_mode(
            {format_pose(ur_task_frame)},
            {format_vector(selection_vector)},
            {format_vector(force_wrench)},
            {arm_config.force_type},
            {format_vector(limits)}
        )

    start_time = get_steptime()
    force_ready = False

    while True:
        tcp_force = get_tcp_force()
        task_force = (
            tcp_force[0] * {task_z[0]:.10f} +
            tcp_force[1] * {task_z[1]:.10f} +
            tcp_force[2] * {task_z[2]:.10f}
        )

        if abs(task_force - {force_z:.8f}) <= {tolerance:.8f}:
            force_ready = True
            break
        end

        if get_steptime() - start_time > {timeout:.3f}:
            break
        end
        sync()
    end

    if force_ready:
        write_output_integer_register(12,1)
    else:
        end_force_mode()

        write_output_integer_register(12, 2)
    end

    prepare_force()
    """

    send_urscript(
        script,
        wait = 0.0
    )

    start_time = time.time()

    while(time.time() - start_time < timeout + 2.0):
        value = rtde_r.getOutputIntRegister(12)

        if value == 1:
            print(f"Force Prepared: {force_z:.3f} N")
            return True

        if value == 2:
            print("Force Preparation Failed")
            return False
        
        time.sleep(0.01)

    print("Force Preparation Timed Out")
    return False
   
def forceModeStop():
    if config.simulation:
        return

    print("\nStopping Force Mode")

    send_urscript("end_force_mode()")

    time.sleep(0.02)

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

def wait_script_complete(register = 12, timeout = 30.0):
    if config.simulation:
        return True

    start_time = time.time()

    while time.time() - start_time < timeout:
        value = rtde_r.getOutputIntRegister(register)

        if value == 1:
            return True

        time.sleep(0.01)

    return False

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

def generate_task_frame():
    tcp_pose = np.asarray(getActualTCPPose(), dtype = float).copy()
    tcp_pose[:3] *= 1000.0
    tcp_pose[3:6] = np.degrees(tcp_pose[3:6])
    return tcp_pose.tolist()

def get_tcp_task_frame(position):
    position = np.asarray(position, dtype = float)

    if position.shape != (3,):
        raise ValueError(
            f"Expected 3 Position Values, Got {position}"
        )

    tcp_pose = np.asarray(
        getActualTCPPose(),
        dtype = float
    )

    return np.array([
        position[0],
        position[1],
        position[2],
        tcp_pose[3],
        tcp_pose[4],
        tcp_pose[5],
    ], dtype = float)

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
            target_tcp_pose = pose
        else:
            pose = as_list(pose)
            sim_pose = list(pose)
            sim_pose[3:6] = np.deg2rad(sim_pose[3:6]).tolist()
            target_tcp_pose = TxyzRxyz_2_Pose(sim_pose) 

        tool_pose = robot.PoseTool()
        target_flange_pose = target_tcp_pose * tool_pose.inv()
        q = robot.SolveIK(target_flange_pose)

        if q is None:
            raise RuntimeError(f"IK Failed for Pose: \n{pose}")

        return np.asarray(q.list(), dtype = float)

    pose = np.asarray(as_list(pose), dtype = float)

    if pose.shape != (6,):
        raise ValueError(
            f"solveIK Expected 6 Pose Values, "
            f"Got {pose}"
        )

    target_pose = pose.copy()
    target_pose[:3] /= 1000.0
    target_pose[3:6] = np.deg2rad(target_pose[3:6])

    if reference is None:
        qnear = np.asarray(rtde_r.getActualQ(), dtype = float)
    else:
        reference = np.asarray(as_list(reference), dtype = float)

        if reference.shape !=(6,):
            raise ValueError(
                "solveIK Reference Must Contain 6 Joint Values"
            )

        qnear = np.deg2rad(reference)

    print("\n=== REAL UR7e IK ===")

    print("\nOriginal Pose (mm / deg):")
    print(np.round(pose, 6))

    print("\nIK Pose (m / rad):")
    print(np.round(target_pose, 6))

    print("\nqnear (deg):")
    print(np.round(np.degrees(qnear), 3))

    

    q = solveIK_urscript(target_pose, qnear)

    if q is None:
        raise RuntimeError(
            f"IUR7e Controller Reports No IK Solutions for Pose {pose}"
        )

    print("\nRaw IK Result:")
    print(q)

    return np.degrees(q)

ik_status_register = 19
ik_result_register = range(13, 19)

def solveIK_urscript(
    target_pose,
    qnear,
    max_position_error = 1e-5,
    max_orientation_error = 1e-3,
    timeout = 5.0
):
    if config.simulation: 
        raise RuntimeError("solveIK_urscript Isn't Valid in Simulation Mode")

    pose_arg = format_pose(target_pose)
    qnear_arg = format_vector(qnear)

    send_urscript(
        f"write_output_integer_register({ik_status_register}, 0)",
        wait = 0.0
    )

    script = (
        f"if get_inverse_kin_has_solution({pose_arg}, {qnear_arg}, "
        f"{max_position_error}, {max_orientation_error}):\n"
        f"  local q = get_inverse_kin({pose_arg}, {qnear_arg}, "
        f"{max_position_error}, {max_orientation_error})\n"
        f"  write_output_float_register(13, q[0])\n"
        f"  write_output_float_register(14, q[1])\n"
        f"  write_output_float_register(15, q[2])\n"
        f"  write_output_float_register(16, q[3])\n"
        f"  write_output_float_register(17, q[4])\n"
        f"  write_output_float_register(18, q[5])\n"
        f"  write_output_integer_register({ik_status_register}, 1)\n"
        f"else:\n"
        f"  write_output_integer_register({ik_status_register}, 2)\n"
        f"end"
    )

    send_urscript(script, wait = 0.0)

    status = 0
    start_time = time.time()

    while time.time() - start_time < timeout:
        status = rtde_r.getOutputIntRegister(ik_status_register)

        if status != 0:
            break
        time.sleep(0.01)

    if status == 0:
        raise RuntimeError(
            "solveIK_urscript Timed OUt Waiting for Controller Response"
        )

    if status == 2:
        return None

    return np.array(
        [rtde_r.getOutputDoubleRegister(i) for i in ik_result_register],
        dtype = float
    )

def get_middle_pose(string):
    frog = np.array(arm_config.data["string_paths"][string]["frog"])
    tip = np.array(arm_config.data["string_paths"][string]["tip"])
    middle = frog.copy()

    middle[:3] = (frog[:3] + tip[:3])/2
    middle[3:6] = frog[3:6]

    return middle

def get_middle_joints(string):
    middle_pose = get_middle_pose(string)

    frog_joints = np.asarray(
        arm_config.data["joint_paths"][string]["frog"],
        dtype = float
    )

    tip_joints = np.asarray(
        arm_config.data["joint_paths"][string]["tip"],
        dtype = float
    )

    qnear_candidates = [
        frog_joints,
        tip_joints,
        (frog_joints + tip_joints) / 2.0
    ]

    print("\n=== MIDDLE IK SEARCH ===")
    print("Middle Pose:")
    print(np.round(middle_pose, 3))

    for index, qnear in enumerate(qnear_candidates):
        print(f"\nTrying qnear {index + 1}:")
        print(np.round(qnear, 3))

        try:
            q = solveIK(middle_pose, reference=qnear)

            if q is not None and len(q) == 6:
                print("Middle IK Found:")
                print(np.round(q, 3))
                return q

        except Exception as e:
            print(f"IK seed {index + 1} failed: {e}")

    raise RuntimeError(
        "Could not find a valid real-robot IK solution "
        "for calculated middle pose."
    )

def distance_from_task_frame(task_frame):
    task_frame = np.asarray(task_frame, dtype = float)

    if task_frame.shape != (6,):
        raise ValueError(
            f"Expected 6 Task-Frae Values, Got {task_frame}"
        )

    tcp_pose = np.asarray(
        getActualTCPPose(),
        dtype = float
    ).copy()

    tcp_pose[:3] *= 1000.0
    tcp_pose[3:6] = np.degrees(tcp_pose[3:6])

    delta_base = tcp_pose[:3] - task_frame[:3]

    R_task = Rotation.from_rotvec(
        np.deg2rad(task_frame[3:6])
    ).as_matrix()

    delta_task = R_task.T @ delta_base
    distance_m = (delta_task[0] / 1000.0)

    return distance_m

def distance_from_frog(
    frog_pose,
    tip_pose
):
    frog_pose = np.asarray(frog_pose, dtype = float)
    tip_pose = np.asarray(tip_pose, dtype = float)
    tcp_pose = np.asarray(getActualTCPPose(), dtype = float).copy()
    tcp_position = tcp_pose[:3] * 1000.0

    bow_vector = tip_pose[:3] - frog_pose[:3]
    bow_length = np.linalg.norm(bow_vector)

    if bow_length < 1e-9:
        raise ValueError("Frog and Tip Positions are Identical")

    bow_direction = bow_vector / bow_length

    tcp_from_frog = tcp_position - frog_pose[:3]

    distance_mm = np.dot(
        tcp_from_frog,
        bow_direction
    )

    distance_mm = np.clip(
        distance_mm,
        0.0,
        bow_length
    )

    return distance_mm / 1000.0

def scale_force_with_distance(
    desired_force,
    distance_from_frog,
    force_constant
):
    force = desired_force + force_constant * distance_from_frog
    # + damping_constant * speed

    return force


# Collision Avoidance
def point_box_distance(point, obstacle):
    point = np.asarray(point, dtype = float)
    minimum = np.asarray(obstacle["min"], dtype = float)
    maximum = np.asarray(obstacle["max"], dtype = float)
    closest = np.maximum(minimum, np.minimum(point, maximum))

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

def current_pose_mm():
    pose = np.asarray(as_list(getActualTCPPose()), dtype = float).copy()
    pose[:3] *= 1000.0
    return pose

def collision_pose(raw_pose):
    pose = np.asarray(as_list(raw_pose), dtype = float).copy()
    pose[3:6] = np.deg2rad(pose[3:6])
    return pose

def find_bow_collision(
        start_pose,
        end_pose,
        safety_distance = None,
        sample_spacing = None
):
    start_pose = np.asarray(start_pose, dtype = float)
    end_pose = np.asarray(end_pose, dtype = float)

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

                minimum = np.asarray(obstacle["min"], dtype = float)
                maximum = np.asarray(obstacle["max"], dtype = float)
                closest = np.maximum(minimum, np.minimum(bow_point, maximum))

                distance = np.linalg.norm(bow_point - closest)

                if distance < safety_distance:
                    print("\nBow Collision Detected")
                    print("Bow Point:", np.round(bow_point, 2))
                    print("Obstacle:", obstacle)
                    print("Distance:", distance)
                    return obstacle

    return None

def generate_avoidance_waypoint(
    start,
    end,
    obstacle,
    excluded_faces = None
):
    start = np.asarray(start, dtype = float)
    end = np.asarray(end, dtype = float)
    minimum = np.asarray(obstacle["min"], dtype = float)
    maximum = np.asarray(obstacle["max"], dtype = float)

    direction = end - start
    direction_norm = np.linalg.norm(direction)

    if direction_norm < 1e-9:
        return start.copy(), None

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

    if excluded_faces is not None:
        for face_index in excluded_faces:
            distances[face_index] = np.inf

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

    return waypoint, face

def pose_with_position(pose, position):
    new_pose = np.asarray(
        pose,
        dtype = float
    ).copy()

    new_pose[:3] = position

    return new_pose

def move_around_obstacle(
        start_position,
        end_position,
        obstacle,
        raw_target_pose,
        speed,
        acceleration,
        excluded_faces = None
):
    waypoint_position, face_used = generate_avoidance_waypoint(
        start_position,
        end_position,
        obstacle,
        excluded_faces = excluded_faces
    )

    waypoint_pose = pose_with_position(
        raw_target_pose,
        waypoint_position
    )

    print(
        "Routing Through Avoidance Waypoint:",
        np.round(waypoint_position, 2)
    )

    moveL(
        waypoint_pose.tolist(),
        speed,
        acceleration
    )

    return waypoint_position, face_used

def resolve_bow_path(
        start_pose,
        end_pose,
        raw_target_pose,
        speed,
        acceleration,
        safety_distance = None,
        max_attempts = 5
):
    current_start = np.asarray(start_pose, dtype = float).copy()
    excluded_faces_by_obstacle = {}

    for attempt in range(1, max_attempts + 1):
        obstacle = find_bow_collision(
            current_start,
            end_pose,
            safety_distance = safety_distance
        )

        if obstacle is None:
            print(
                "Path Clear After Avoidance"
                if attempt > 1 else
                "Path Clear"
            )

            return True

        print(
            f"\nPath Blocked by Obstacle"
            f"(Avoidance Attempt: {attempt}/{max_attempts})"
        )

        obstacle_key = (
            tuple(obstacle["min"]),
            tuple(obstacle["max"])
        )

        excluded_faces = excluded_faces_by_obstacle.setdefault(
            obstacle_key,
            set()
        )

        _, face_used = move_around_obstacle(
            current_start[:3],
            end_pose[:3],
            obstacle,
            raw_target_pose,
            speed,
            acceleration,
            excluded_faces = excluded_faces
        )

        if face_used is not None:
            excluded_faces.add(face_used)

        current_start = current_pose_mm()

        print(f"Path Still Blocked After {max_attempts} Avoidance Attempts")
        return False

# Motion Routines
def bowing_segment(
        start_pose,
        end_pose,
        start_joints,
        end_joints,
        halt=False,
        rosin = False,
        task_frame = None
    ):
    if halt:
        stop()
        sys.exit()

    start_pose = np.asarray(as_list(start_pose), dtype = float)
    end_pose = np.asarray(as_list(end_pose), dtype = float)

    if start_pose.shape != (6,):
        raise ValueError("Starting Pose Must Contain 6 Values")
    if end_pose.shape != (6,):
        raise ValueError("Ending Pose Must Contain 6 Values")

    if config.simulation:
        current_rdk_pose = robot.Pose()

        target_rdk_pose = Mat(current_rdk_pose)
        target_rdk_pose.setPos([
            end_pose[0],
            end_pose[1],
            end_pose[2]
        ])

        robot.setSpeed(
            speed_linear = arm_config.bow_speed * 1000,
            accel_linear = arm_config.bow_acceleration * 1000
        )

        robot.MoveL(target_rdk_pose)

        return

    ur_target_pose = end_pose.copy()
    ur_target_pose[:3] /= 1000.0
    ur_target_pose[3:6] = np.deg2rad(ur_target_pose[3:6])

    if not arm_config.useForce:
        print("\nForce Mode OFF")
        print("Bowing Without Force Control")
        
        script = (
            f"movel("
            f" {format_pose(ur_target_pose)},"
            f"a = {arm_config.bow_acceleration}, "
            f"v = {arm_config.bow_speed}"
            f")"
        )

        send_urscript(script, wait = 0.001)
        wait_pose(
            ur_target_pose
        )

        return

    if task_frame is None:
        raise ValueError(
            "Task Frame Must Be Generated Before Bowing"
        )

    if rosin:
        selection_vector = arm_config.rosin_selection_vector
        wrench = arm_config.rosin_wrench
        limits = arm_config.rosin_limits
        frog_pose = np.asarray(
            arm_config.rosin_position["frog"],
            dtype = float
        )
        tip_pose = np.asarray(
            arm_config.rosin_position["tip"],
            dtype = float
        )

    else:
        selection_vector = arm_config.selection_vector
        wrench = arm_config.wrench
        limits = arm_config.limits
        frog_pose = np.asarray(
            arm_config.data["string_paths"]
            [arm_config.current_string]["frog"],
            dtype=float
        )

        tip_pose = np.asarray(
            arm_config.data["string_paths"]
            [arm_config.current_string]["tip"],
            dtype=float
        )

    distance = distance_from_frog(frog_pose, tip_pose)

    force_z = scale_force_with_distance(
        desired_force = arm_config.bow_force,
        distance_from_frog = distance,
        force_constant = arm_config.force_constant
    )

    force_z = np.clip(
        force_z,
        -arm_config.max_force,
        arm_config.max_force
    )

    force_wrench = list(as_list(wrench))
    force_wrench[0] = 0.0
    force_wrench[1] = 0.0
    force_wrench[2] = force_z
    force_wrench[3] = 0.0
    force_wrench[4] = 0.0
    force_wrench[5] = 0.0

    ur_task_frame = np.asarray(task_frame, dtype = float).copy()
    ur_task_frame[:3] /= 1000.0
    ur_task_frame[3:6] = np.deg2rad(ur_task_frame[3:6])

    bow_vector = tip_pose[:3] - frog_pose[:3]
    bow_length = np.linalg.norm(bow_vector)

    if bow_length < 1e-9:
        raise ValueError(
            "Frog and Tip Positions are Identical"
        )

    bow_direction = bow_vector / bow_length

    frog_x = frog_pose[0] / 1000.0
    frog_y = frog_pose[1] / 1000.0
    frog_z = frog_pose[2] / 1000.0

    bow_dir_x = bow_direction[0]
    bow_dir_y = bow_direction[1]
    bow_dir_z = bow_direction[2]

    bow_length_m = bow_length / 1000.0

    limits = list(as_list(limits))
    limits[2] = arm_config.force_z_speed_limit
  
    print("\nBowing")

    script = f"""
def bow_segment():
    thread force_controller():

        while True:
            actual = get_actual_tcp_pose()

            frog_dx = actual[0] - {frog_x:.10f}
            frog_dy = actual[1] - {frog_y:.10f}
            frog_dz = actual[2] - {frog_z:.10f}

            distance = (
                frog_dx * {bow_dir_x:.10f}
                + frog_dy * {bow_dir_y:.10f}
                + frog_dz * {bow_dir_z:.10f}
            )

            if distance < 0.0:
                distance = 0.0
            end

            if distance > {bow_length_m:.10f}:
                distance = {bow_length_m:.10f}
            end

            force_magnitude = (
                {arm_config.bow_force:.10f}
                + {arm_config.force_constant:.10f} * distance
            )

            if force_magnitude < 0.0:
                force_magnitude = 0.0
            end


            if force_magnitude > {arm_config.max_force:.10f}:
                force_magnitude = {arm_config.max_force:.10f}
            end

            commanded_force = -force_magnitude

            force_mode(
                {format_pose(ur_task_frame)},
                {format_vector(selection_vector)},
                [0, 0, commanded_force, 0, 0, 0],
                {arm_config.force_type},
                {format_vector(limits)}
            )

            sync()

        end
    end

    global force_handler = run force_controller()
    sleep(0.05)

    movel(
        {format_pose(ur_target_pose)},
        a={arm_config.bow_acceleration},
        v={arm_config.bow_speed}
    )

    kill force_handler
    end_force_mode()

end

bow_segment()
"""


    # script = f"""
    # force_mode(
    #     {format_pose(ur_task_frame)},
    #     {format_vector(selection_vector)},
    #     {format_vector(force_wrench)},
    #     {arm_config.force_type},
    #     {format_vector(limits)}
    # )

    # movel(
    #     {format_pose(ur_target_pose)},
    #     a = {arm_config.bow_acceleration},
    #     v = {arm_config.bow_speed}
    # )

    # end_force_mode()
    # """

    send_urscript(
        script,
        wait = 0.0
    )

    if not wait_pose(
        ur_target_pose,
        tol = 0.01,
        timeout = 30.0, 
        task_frame = ur_task_frame,
        selection_vector = selection_vector
    ):
        final_pose = np.asarray(getActualTCPPose(), dtype = float)
        final_error = np.linalg.norm(final_pose[:3] - ur_target_pose[:3])

        raise RuntimeError(
            f"Bow Motion Didn't Reach Target "
            f"Final Error: {final_error * 1000.0:.2f} mm"
        )

    print("Segment Complete")
    
def getRobot():
    if config.simulation:
        return robot
    return None