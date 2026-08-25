# UR7e Violin Bowing Robot
Note: ur_rtde only works on Python 3.12 on Windows

## Overview
This project controls a Universal Robots UR7e to provide consistent, repeatable bow stroke for collecting acoustic data from violin bows.

By automating the bowing process, the system eliminates much of the variability introduced by human performance, enabling more controlled comparisons of different bow materials and their effects on sound production. 

This project is a work in progress and serves as a platform for future research into the relationship between bow properties, bowing parameters, and the resulting acoustic characteristics of bowed string instruments

## Repository Structure
The repository is organized as follows:

```text
BowRobot/
├── AcousticData
│   ├── Data                # Raw acoustic data
│   ├── Functions           # Templates for analysis and plotting
│   └── Analysis            # Scripts and experimental results
├── Sensing                 # Documentation for the project's sensors
├── SW                      # Hardware files
├── UR Files
│   ├── Code                
│   │   ├── program_main    # Main source code directory
│   └── UR7e                # Official Universal Robots documentation
├── RoboDK Simulation       # Robot simulation models
└── README.md
```

## Getting Started (Code Setup)
How to download the repository and how to set-up the Python environment required to run the project

From your terminal:

```bash
git clone https://github.com/AmandaChung2005/BowRobot.git
cd BowRobot
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

Connecting to the Windows Subsytem for LInux
```bash
wsl
cd /mnt/c/Users/path/to/BowRobot/Code
source venv_linux/bin/activate
python your_script.py
```

Linux/MacOS:
Ensure that Python 3.12 is installed.
```bash
python3.12 -m venv venv
source ./venv/bin/activate
pip install -r requirements.txt
```

## UR7e Setup
Robot Network Settings:
IP Address: 192.168.56.101
Subnet Mask: 255.255.255.0

Windows Ethernet Settings:
IP Address: 192.168.56.100
Subnet Mask: 255.255.255.0


## Running the Code
Activate the virtual environment and run the desired script

Windows PowerShell:

	.\venv\Scripts\python.exe

Linux or macOS:

	./venv/bin/python

## Uploading to GitHub
How to commit and push changes to the remote repository

 ```bash
 cd path/to/BowRobot
 git add .
 git commit -m "describe your change"
 git push origin <branch-name>
 ```


## API Reference
This project uses the SDU Robotics ur_rtde library


https://sdurobotics.gitlab.io/ur_rtde/api/api.html

