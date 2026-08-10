# RTDE_control_Amanda

Minimal package to run:
- `control_loop_connector.py` (RTDE setpoint control loop)
- `Dashboard_X.py` (PolyScope X Robot REST API CLI)

## Contents
- `control_loop_connector.py` - RTDE control loop client
- `connector.py` - RTDE helper used by the control loop
- `Dashboard_X.py` - PolyScope X REST API CLI
- `control_loop_configuration.xml` - RTDE input/output recipes
- `RTDE_Inputs.csv` / `RTDE_Outputs.csv` - RTDE field catalogs
- `endpoints.yaml` - named REST commands for Dashboard_X
- `rtde/` - Universal Robots RTDE Python library
- `requirements.txt` - Python dependencies for Dashboard_X

## Setup
```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Run RTDE control loop
1. Edit `ROBOT_HOST` in `control_loop_connector.py`
   - Docker URSim: `127.0.0.1`
   - Real robot: e.g. `192.168.0.12`
2. On the robot/sim: enable RTDE, load the matching UR program, start the register-read thread, press Play.
3. Run:
```powershell
python control_loop_connector.py
```

## Run Dashboard_X
```powershell
# Real robot example
python Dashboard_X.py --host 192.168.0.12 --port 80 --scheme http --cfg endpoints.yaml

# Docker URSim mapped to host port 8080/8000
python Dashboard_X.py --host 127.0.0.1 --port 8080 --scheme http --cfg endpoints.yaml
```

Then at the prompt: `help`, `cmd power_on`, `cmd play`, etc.

Put the robot in Remote mode before sending REST commands.
