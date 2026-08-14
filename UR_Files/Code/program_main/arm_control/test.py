import socket

ROBOT_IP = "192.168.56.101"
ROBOT_PORT = 30004

print(f"Connecting to {ROBOT_IP}:{ROBOT_PORT}...")

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.settimeout(10)

try:
    sock.connect((ROBOT_IP, ROBOT_PORT))
    print("TCP CONNECTION SUCCESSFUL")
except Exception as e:
    print(f"TCP CONNECTION FAILED: {type(e).__name__}: {e}")
finally:
    sock.close()