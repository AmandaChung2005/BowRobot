import socket
import time

ROBOT_IP = "192.168.56.101"   # CHANGE THIS
SCRIPT_PORT = 30002

force = 2.0

script = f"""
def force_test():

    zero_ftsensor()
    sleep(0.2)

    # Current TCP pose is used as the task frame
    task_frame = get_actual_tcp_pose()

    # Z-axis compliant
    selection_vector = [0, 0, 1, 0, 0, 0]

    # Apply +2 N in task-frame Z
    wrench = [0, 0, {force}, 0, 0, 0]

    # Type 2 = frame-relative
    limits = [0.2, 0.2, 0.2, 0.5, 0.5, 0.5]

    force_mode(
        task_frame,
        selection_vector,
        wrench,
        2,
        limits
    )

    sleep(5.0)

    end_force_mode()

end
force_test()
"""

print("Connecting to robot...")

with socket.create_connection(
    (ROBOT_IP, SCRIPT_PORT),
    timeout=5
) as sock:

    sock.sendall(
        script.encode("utf-8")
    )

print("Force-mode test sent.")