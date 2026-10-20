import socket
import time
import numpy as np
from scipy.spatial.transform import Rotation
from pathlib import Path
import sys

project_path = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(project_path))

import config

from rtde_receive import RTDEReceiveInterface

COMMAND_FORCE = 1.0       # N
TEST_TIME = 5.0            # seconds

FORCE_GAIN_SCALING = 1.6
FORCE_DAMPING = 0.7

FORCE_LIMITS = [
    0.05,
    0.05,
    0.002,      # 2 mm/s
    1.0,
    1.0,
    1.0
]

FORCE_TYPE = 2
SELECTION_VECTOR = [0, 0, 1, 0, 0, 0]

script_counter = 0

def send_urscript(script, wait=0.05):

    global script_counter

    script = script.strip()

    if not script:
        return

    lines = [
        line.strip()
        for line in script.splitlines()
        if line.strip()
    ]

    starts_top_level = lines[0].startswith(("def ", "thread "))

    if len(lines) > 1 and not starts_top_level:

        script_counter += 1

        function_name = f"stationary_test_{script_counter}"

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
        timeout=5.0
    ) as sock:

        sock.sendall(
            script.encode("utf-8")
        )

    time.sleep(wait)

rtde_r = RTDEReceiveInterface(
    config.host_ip
)

def main():

    if not rtde_r.isConnected():
        raise RuntimeError(
            f"Could not connect to robot at {config.host_ip}"
        )

    print()
    print("UR7e STATIONARY FORCE TEST")
    print()
    print(f"Robot IP:           {config.host_ip}")
    print(f"Commanded Task Fz:  {COMMAND_FORCE:+.3f} N")
    print(
        f"Z Speed Limit:      "
        f"{FORCE_LIMITS[2]:.4f} m/s"
    )
    print(
        f"Gain Scaling:       "
        f"{FORCE_GAIN_SCALING:.3f}"
    )
    print(
        f"Damping:            "
        f"{FORCE_DAMPING:.3f}"
    )
    print()

    input(
        "Move robot to the desired stationary position, "
        "then press ENTER..."
    )

    tcp_pose = np.asarray(
        rtde_r.getActualTCPPose(),
        dtype=float
    ).flatten()

    R_task = Rotation.from_rotvec(
        tcp_pose[3:6]
    ).as_matrix()

    task_z = R_task[:, 2]

    print()
    print("TASK FRAME")
    
    print(
        f"Task Z base = "
        f"[{task_z[0]:+.6f}, "
        f"{task_z[1]:+.6f}, "
        f"{task_z[2]:+.6f}]"
    )

    print()


    task_frame = (
        "p["
        f"{tcp_pose[0]}, "
        f"{tcp_pose[1]}, "
        f"{tcp_pose[2]}, "
        f"{tcp_pose[3]}, "
        f"{tcp_pose[4]}, "
        f"{tcp_pose[5]}"
        "]"
    )


    print("Entering force mode...")

    script = f"""
def stationary_force_test():

    force_mode_set_gain_scaling(
        {FORCE_GAIN_SCALING}
    )

    force_mode_set_damping(
        {FORCE_DAMPING}
    )

    force_mode(
        {task_frame},
        {SELECTION_VECTOR},
        [0, 0, {COMMAND_FORCE}, 0, 0, 0],
        {FORCE_TYPE},
        {FORCE_LIMITS}
    )

end

stationary_force_test()
"""

    send_urscript(
        script,
        wait=0.05
    )

    print("Force mode active.")
    print()

    print("LIVE DATA")

    start_time = time.time()

    final_raw_task_fz = 0.0
    final_vz_task = 0.0

    try:

        while time.time() - start_time < TEST_TIME:
            wrench = np.asarray(
                rtde_r.getActualTCPForce(),
                dtype=float
            ).flatten()

            force_base = wrench[:3]

            force_task = (
                R_task.T @ force_base
            )

            task_fz = float(
                force_task[2]
            )

            speed = np.asarray(
                rtde_r.getActualTCPSpeed(),
                dtype=float
            ).flatten()

            velocity_base = speed[:3]

            velocity_task = (
                R_task.T @ velocity_base
            )

            vz_task = float(
                velocity_task[2]
            )

            error = (
                task_fz - COMMAND_FORCE
            )

            current_pose = np.asarray(
                rtde_r.getActualTCPPose(),
                dtype=float
            ).flatten()

            z = float(
                current_pose[2]
            )

            final_raw_task_fz = task_fz
            final_vz_task = vz_task

            print(
                f"Cmd Fz: {COMMAND_FORCE:+7.3f} N | "
                f"Measured Fz: {task_fz:+7.3f} N | "
                f"Error: {error:+7.3f} N | "
                f"Z: {z:+.5f} m | "
                f"Vz_task: {vz_task:+.5f} m/s",
                flush=True
            )

            time.sleep(0.05)

    except KeyboardInterrupt:

        print()
        print("Test interrupted by user.")

    finally:

        print()
        print("Stopping force mode...")

        send_urscript(
            "end_force_mode()",
            wait=0.05
        )

        time.sleep(0.1)

        print()
        print("STATIONARY FORCE TEST COMPLETE")
        print()

        print(
            f"Commanded Task Fz: "
            f"{COMMAND_FORCE:+.3f} N"
        )

        print(
            f"Final Measured Task Fz: "
            f"{final_raw_task_fz:+.3f} N"
        )

        print(
            f"Final Task Vz: "
            f"{final_vz_task:+.5f} m/s"
        )

        print()


if __name__ == "__main__":
    main()