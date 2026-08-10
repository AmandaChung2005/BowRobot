import sys
from pathlib import Path

code_path = Path(__file__).parent/"arm control"
sys.path.append(str(code_path))

import arm_control.robot_interface
import config

if config.setup:
    import arm_control.calibration
else:
    import arm_control.main