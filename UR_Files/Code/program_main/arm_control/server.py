import socketserver
import threading
import argparse
import os

script_file = "generated.script"
port = 50002

class FileHandler(socketserver.StreamRequestHandler):
    def handel(self):
        client = f"{self.client_address} on {threading.current_thread().name}"
        print(f"Connected: {client}")

        request = self.rfile.readline().decode().strip()
        print(f"Request: {request}")

        if not os.path.exists(script_file):
            print(f"ERROR: {script_file} Doesn't Exist")
            self.wfile.write(b"")

        with open(script_file, "rb") as f:
            script = f.read()

        self.wfile.write(script)
        print(f"Sent {script_file}")
        print(f"Close: {client}")

class ThreadedTCPServer(socketserver.ThreadingMixIn, socketserver.TCPServer):
    allow_reuse_address = True

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-p",
        "--port",
        default = port,
        type = int,
        help = "Port to Listen On"
    )

    args = parser.parse_args()

    server = ThreadedTCPServer(("", args.port), FileHandler)

    print(f"Serving '{script_file}' on port {args.port}")

    try:
        server.serve_forever()

    except KeyboardInterrupt:
        print("\nStopping Server...")
        server.shutdown()
        server.server_close()