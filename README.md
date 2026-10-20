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
├── Sensing                 # Documentation for project's sensors
├── SW                      # Hardware files
├── UR Files
│   ├── Code                
│   │   ├── program_main    # Main source code directory
│   └── UR7e                # Official Universal Robots documentation
├── RoboDK Simulation       # Robot simulation models
├── requirements.txt        # Python dependencies
└── README.md
```

## Getting Started (Code Setup)
This section explains how to download the repository and how to set-up the Python environment required to run the project

### Prerequisites
Before setting up the project, install the following:
1. Git: ```https://git-scm.com/downloads```
2. Code Editor  
The recommended code editor is Visual Studio Code: ```https://code.visualstudio.com/```
	- Install the "Python" Extension
3. Python 3.12
	- Windows  
	```winget``` is included with Windows 11. If it is not installed, it can be downloaded from the [Github releases page](https://github.com/microsoft/winget-cli/releases) for Microsoft's Windows Package Manager

	  ```bash
	  winget install Python.Python.3.12
	  ```
	- Linux (Ubuntu/Debian-based distributions)
	  ```bash
	  sudo apt update
	  sudo apt install python3.12 python3.12-venv
	  ```
	- MacOS  
	  Install Python 3.12 using Homebrew  
	  If Homebrew is not already installed:
	  ```bash
	  /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
	  ```
	  Then install Python 3.12:
	  ```bash
	  brew install python@3.12
	  ```

### Cloning the Repository
Open a terminal and run:
```bash
git clone https://github.com/AmandaChung2005/BowRobot.git
cd BowRobot
git submodule update --init --recursive
code .
```

```code .``` opens the repoistory in your text editor

### Python Environment Setup
A virtual environment keeps the project's Python dependencies separate from other Python installations on your computer
#### Windows:
Creating the virtual environment:
```bash
py -3.12 -m venv venv
```
Activating the virtual environement:
```bash
.\venv\Scripts\activate
```
If PowerShell reports that script execution is disabled, run:
```bash
Set-ExecutionPolicy -ExecutionPolicy RemoteSigned -Scope CurrentUser
```

Installing the project dependencies:
```bash
pip install -r requirements.txt
```
#### Linux/macOS
Creating the virtual environment:
```bash
python3.12 -m venv venv
```

Activating the virtual environement:
```bash
source ./venv/bin/activate
```

Installing the project dependencies:
```bash
pip install -r requirements.txt
```

## Selecting the Virtual Environment in VS Code
1. Press Ctrl + Shift + P
2. Search for Python: Select Interpreter
3. Select the Python interpreter inside the project's ```venv``` folder:
	- Windows  
		```bash
		venv\Scripts\python.exe
		```
	- Linux/macOS
		```bash
		venv/bin/python
		```

## UR7e Setup
### Robot Network Settings:
1. Open settings
2. Navigate to System -> Network
3. Set:
	```bash
	IP Address: 192.168.56.101
	Subnet Mask: 255.255.255.0
	```

### Computer's Ethernet Settings:
1. Open Network/Ethernet settings
2. Open the Ethernet connection properties
3. Open the IPv4/TCP-IP settings
4. Change the IPv4 configuration from Automatic/DHCP to Manual/Static
5. Set:  
	```bash
	IP Address: 192.168.56.100
	Subnet Mask: 255.255.255.0
	```
6. Leave the gateway/router and DNS fields blank unless required by your network configuration
7. Save/Apply the settings

### Enabling Ethernet Access for VS Code (macOS)
1. Open System Settings
2. Navigate to Privacy & Settings
3. Select Local Network
4. Find Visual Studio Code and toggle access on

### Running the Code
The main program should be run through run.py  
Make sure the virtual environment is activated before running the program
### From Terminal
1. Navigate to the root of the repository:
	```bash
	cd path/to/BowRobot
	```
2. Activate the virtual environment if it's not already active
3. Run the program
	#### Windows
	```bash
	python .\UR_Files\Code\program_main\run.py
	```
	#### Linux/macOS
	```bash
	python ./UR_Files/Code/program_main/run.py
	```

### From VS Code
1. Open the ```BowRobot``` repository in VS Code
click the play butto on run.py
2. Open: ```UR_Files/Code/program_main/run.py```
3. Make sure the project's ```venv``` is selected as the Python interpreter
4. Click "Run Python File" in the upper-right corner of VS Code

## Pulling Updates from GitHub
To download the latest changes from GitHub:
```bash
cd path/to/BowRobot
git pull
```

## Commiting and Pushing Changes to GitHub
 ```bash
 cd path/to/BowRobot
 git add .
 git commit -m "describe your change"
 git push origin <branch-name>
 ```

## API Reference
This project uses the SDU Robotics "ur_rtde" library to communicate with and control the Universal Robots UR7e through the Real-Time Data Exchange (RTDE) interface  
  
Official Documentation:
https://sdurobotics.gitlab.io/ur_rtde/api/api.html

