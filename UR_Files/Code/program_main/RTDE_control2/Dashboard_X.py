#!/usr/bin/env python
"""
UR PolyScope X - Robot REST API CLI

Usage:
* use for live robot
  python Dashboard_X.py --host 192.168.0.12 --port 8080 --scheme http --cfg endpoints.yaml
* use for URSim (PolyScope X) in Docker
  python Dashboard_X.py --host 192.168.0.12 --port 8080 --scheme http --cfg endpoints.yaml

Then at the prompt:
  > help
  > show base
  > headers add Authorization "Bearer <token-if-any>"
  > get /api/.../status
  > post /api/.../power/on
  > post /api/.../programs/load {"name":"my_program"}
  > cmd power_on
  > cmd load_program name=my_program
  > exit

IMPORTANT:
- Switch the robot to *Remote* mode in PolyScope X before sending commands.
- Prefer testing on URSim (PolyScope X) first.
"""

import argparse
import json
import shlex
from typing import Any, Dict, Optional
import requests
import yaml
from rich import print
from rich.prompt import Prompt
from rich.console import Console
from rich.table import Table
from rich.panel import Panel

console = Console()

DEFAULT_TIMEOUT = 10.0


# --- Built-in command templates (minimal defaults; prefer endpoints.yaml) ---
# These are overridden by endpoints.yaml if provided via --cfg flag
COMMANDS_TEMPLATE = {
    # Each item defines: method, path, fixedBody, and optional args
    # For actual robot control, use: python Dashboard_X.py --host 192.168.0.10cd --cfg endpoints.yaml
    "power_on": {
        "method": "PUT",
        "path": "/universal-robots/robot-api/robotstate/v1/state/",
        "fixedBody": {"action": "POWER_ON"}
    },
    "power_off": {
        "method": "PUT",
        "path": "/universal-robots/robot-api/robotstate/v1/state/",
        "fixedBody": {"action": "POWER_OFF"}
    },
    "brake_release": {
        "method": "PUT",
        "path": "/universal-robots/robot-api/robotstate/v1/state/",
        "fixedBody": {"action": "BRAKE_RELEASE"}
    },
    "unlock_protective_stop": {
        "method": "PUT",
        "path": "/universal-robots/robot-api/robotstate/v1/state/",
        "fixedBody": {"action": "UNLOCK_PROTECTIVE_STOP"}
    },
    "restart_safety": {
        "method": "PUT",
        "path": "/universal-robots/robot-api/robotstate/v1/state/",
        "fixedBody": {"action": "RESTART_SAFETY"}
    },
    "load_program": {
        "method": "PUT",
        "path": "/universal-robots/robot-api/program/v1/load",
        "args": ["programName"]
    },
    "play": {
        "method": "PUT",
        "path": "/universal-robots/robot-api/program/v1/state",
        "fixedBody": {"action": "play"}
    },
    "pause": {
        "method": "PUT",
        "path": "/universal-robots/robot-api/program/v1/state",
        "fixedBody": {"action": "pause"}
    },
    "stop": {
        "method": "PUT",
        "path": "/universal-robots/robot-api/program/v1/state",
        "fixedBody": {"action": "stop"}
    },
    "resume": {
        "method": "PUT",
        "path": "/universal-robots/robot-api/program/v1/state",
        "fixedBody": {"action": "resume"}
    },
    "get_program_state": {
        "method": "GET",
        "path": "/universal-robots/robot-api/program/v1/state"
    },
    "get_system_time": {
        "method": "GET",
        "path": "/universal-robots/robot-api/system/v1/system-time/"
    }
}


def load_commands_from_yaml(path: Optional[str]) -> Dict[str, Dict[str, Any]]:
    if not path:
        return COMMANDS_TEMPLATE
    with open(path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    # Merge: YAML overrides template
    merged = dict(COMMANDS_TEMPLATE)
    merged.update(data)
    return merged


class RobotClient:
    def __init__(self, base_url: str, timeout: float = DEFAULT_TIMEOUT):
        self.base_url = base_url.rstrip("/")
        self.timeout = timeout
        self.session = requests.Session()
        self.headers: Dict[str, str] = {
            "Accept": "application/json",
            "Content-Type": "application/json"
        }

    def set_header(self, key: str, value: str):
        if not value:
            self.headers.pop(key, None)
        else:
            self.headers[key] = value


    def request(self, method: str, path: str, data: Optional[Any] = None) -> tuple[int, Any]:
        url = f"{self.base_url}/{path.lstrip('/')}"
        try:
            resp = self.session.request(method, url, json=data, headers=self.headers, timeout=self.timeout)
            resp.raise_for_status()
            try:
                return resp.status_code, resp.json()
            except json.JSONDecodeError:
                return resp.status_code, resp.text
        except requests.RequestException as e:
            raise e


def pretty_print_response(status: int, data: Any):
    if status is None:
        status = 0
    status_color = "green" if 200 <= status < 300 else "red"
    from rich.panel import Panel
    console.print(Panel.fit(f"[{status_color}]HTTP {status}[/]"))
    if data is None:
        print("[yellow]No response body (None)[/]")
        return
    if isinstance(data, (dict, list)):
        console.print_json(data=data)
    else:
        print(data)


def print_help():
    table = Table(title="URX Robot API CLI - Commands")
    table.add_column("Command")
    table.add_column("Description / Syntax")
    table.add_row("help", "Show this help")
    table.add_row("show base", "Show base URL and headers")
    table.add_row("headers add <Key> <Value>", "Add/replace an HTTP header")
    table.add_row("headers del <Key>", "Remove an HTTP header")
    table.add_row("get <path>", "HTTP GET, e.g., get /api/.../state")
    table.add_row("post <path> [json]", "HTTP POST with optional JSON body")
    table.add_row("put <path> [json]", "HTTP PUT with optional JSON body")
    table.add_row("patch <path> [json]", "HTTP PATCH with optional JSON body")
    table.add_row("delete <path>", "HTTP DELETE")
    table.add_row("cmd <name> [k=v ...]", "Run a named command (from YAML or template)")
    table.add_row("exit / quit", "Exit the CLI")
    console.print(table)


def parse_kv_args(tokens):
    args = {}
    for tok in tokens:
        if "=" not in tok:
            raise ValueError(f"Expected k=v argument, got '{tok}'")
        k, v = tok.split("=", 1)
        args[k] = v
    return args


def main():
    print("Running:", __file__)
    ap = argparse.ArgumentParser(description="UR PolyScope X Robot API CLI")
    ap.add_argument("--host", required=True, help="Robot hostname/IP (or URSim)")
    ap.add_argument("--port", type=int, default=80, help="HTTP port (default 80)")
    ap.add_argument("--scheme", choices=["http", "https"], default="http", help="HTTP scheme")
    ap.add_argument("--cfg", help="YAML file with endpoint mappings (optional)")
    ap.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT, help="Request timeout (s)")
    args = ap.parse_args()

    base_url = f"{args.scheme}://{args.host}:{args.port}"
    client = RobotClient(base_url, timeout=args.timeout)
    cmd_map = load_commands_from_yaml(args.cfg)

    console.print(Panel.fit(
        f"[bold]UR PolyScope X - Robot API CLI[/]\n"
        f"Base: [cyan]{base_url}[/]\n"
        f"Tip: Switch robot to [bold]Remote[/bold] mode and test on URSim first.",
        title="Welcome"
    ))
    print_help()

    while True:
        try:
            line = Prompt.ask("[bold cyan]>[/]")
        except (EOFError, KeyboardInterrupt):
            print("\nExiting.")
            break
        if not line.strip():
            continue

        try:
            tokens = shlex.split(line)
        except ValueError as e:
            print(f"[red]Parse error:[/] {e}")
            continue

        if not tokens:
            continue
        cmd, *rest = tokens

        try:
            if cmd in ("exit", "quit"):
                break
            elif cmd == "help":
                print_help()
            elif cmd == "show" and rest and rest[0] == "base":
                print({"base_url": client.base_url, "headers": client.headers})
            elif cmd == "headers" and rest:
                sub = rest[0]
                if sub == "add" and len(rest) >= 3:
                    key = rest[1]
                    value = " ".join(rest[2:])
                    client.set_header(key, value)
                    print(f"[green]Header set:[/] {key}={value}")
                elif sub == "del" and len(rest) == 2:
                    client.set_header(rest[1], "")
                    print(f"[yellow]Header removed:[/] {rest[1]}")
                else:
                    print("[red]Usage:[/] headers add <Key> <Value> | headers del <Key>")
            elif cmd in ("get", "post", "put", "patch", "delete"):
                if not rest:
                    print(f"[red]Usage:[/] {cmd} <path> [json]")
                    continue
                path = rest[0]
                body = None
                if len(rest) > 1:
                    try:
                        body = json.loads(" ".join(rest[1:]))
                    except json.JSONDecodeError as e:
                        print(f"[red]Invalid JSON:[/] {e}")
                        continue
                status, data = client.request(cmd, path, body)
                pretty_print_response(status, data)
            elif cmd == "cmd":
                if not rest:
                    print("[red]Usage:[/] cmd <name> [k=v ...]")
                    continue

                name, *kv = rest
                if name not in cmd_map:
                    print(f"[red]Unknown command:[/] {name}")
                    continue

                m = cmd_map[name]
                method = m.get("method", "POST").upper()
                path = m.get("path")
                if not path:
                    print(f"[red]Command '{name}' is missing 'path' in YAML[/]")
                    continue

                expected_args = m.get("args", []) or []
                fixed_body = m.get("fixedBody", None)

                # Build the JSON body:
                data = {}

                # 1) Bring in fixedBody first (constant JSON from YAML)
                if isinstance(fixed_body, dict):
                    data.update(fixed_body)

                # 2) Overlay any CLI k=v args (so users can override if needed)
                if expected_args:
                    try:
                        provided = parse_kv_args(kv)
                    except ValueError as e:
                        print(f"[red]Arg parsing error:[/] {e}")
                        continue
                    # If you want to restrict to only listed args, uncomment next line:
                    # provided = {k: v for k, v in provided.items() if k in expected_args}
                    data.update(provided)

                # If neither fixedBody nor args provided anything, use None (no body)
                if data == {}:
                    data = None

                # --- DEBUG LOGGING: show exactly what we will send ---
                console.print(Panel.fit(
                    f"[bold]REQUEST[/]\n"
                    f"Method: [cyan]{method}[/]\n"
                    f"Path:   [cyan]{path}[/]\n"
                    f"Body:   [cyan]{json.dumps(data) if data is not None else 'None'}[/]\n"
                    f"Headers:{client.headers}",
                    title="Robot API"
                ))

                status, resp = client.request(method, path, data)
                pretty_print_response(status, resp)
            else:
                print("[yellow]Unknown command. Type 'help' for options.[/]")
        except requests.RequestException as e:
            print(f"[red]HTTP error:[/] {e}")
        except Exception as e:
            print(f"[red]Error:[/] {e}")


if __name__ == "__main__":
    main()