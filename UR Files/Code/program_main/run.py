import sys
from pathlib import Path

code_path = Path(__file__).parent/"arm control"
sys.path.append(str(code_path))

import robot_interface
import config

if config.setup:
    import calibration
else:
    import main