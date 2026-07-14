# UR7e Violin Bowing Robot
Note: ur_rtde only works on Python 3.12 on Windows

## Overview
This project controls a Universal Robots UR7e to provide consistent, repeatable bow stroke for collecting acoustic data from violin bows.

By automating the bowing process, the system eliminates much of the variability introduced by human performance, enabling more controlled comparisons of different bow materials and their effects on sound production. 

This project is a work in progress and serves as a platform for future research into the relationship between bow properties, bowing parameters, and the resulting acoustic characteristics of bowed string instruments

## Repository Structure

```text
BowRobot/
├── Matlab
│   ├── Data             # Raw acoustic data
│   ├── Functions        # Templates for analysis and plotting
│   └── Analysis         # Scripts and experimental results
├── SW                   # Hardware files
├── UR Files
│   ├── Code             # Main source code directory
│   └── UR7e             # Official Universal Robots documentation
├── RoboDK Simulation    # Robot simulation models
└── README.md
```

## Getting Started (Code Setup)
The following instructions describe how to download the repository and how to set-up the Python environment required to run the project

From your terminal:

```bash
git clone https://github.com/IanBallinger/Robotics_final_spring2026_thursPM.git
cd Robotics_final_spring2026_thursPM
git submodule update --init --recursive
code .
```

Windows:
"winget" is included with Windows 11. If it is not installed, it can be downloaded from the [Github releases page](https://github.com/microsoft/winget-cli/releases) for Microsoft's Windows Package Manager
```bash
winget install Python.Python.3.12
py -3.12 -m venv venv
.\venv\Scripts\activate
pip install -r requirements.txt
```

Linux/MacOS:
Ensure that Python 3.12 installed.
```bash
python3.12 -m venv venv
source ./venv/bin/activate
pip install -r requirements.txt
```
## Running the Code
Activate the virtual environemnt and run the desired script

Windows PowerShell:

	.\venv\Scripts\python.exe

Linux or macOS:

	./venv/bin/python

## API Reference
This project uses the SDU Robotics ur_rtde library


https://sdurobotics.gitlab.io/ur_rtde/api/api.html

