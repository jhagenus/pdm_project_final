import sys
from pathlib import Path
import numpy as np

# Adding the needed directories to the system
project_dir = Path(__file__).resolve().parent.parent
gym_dir = project_dir.parent.parent
sys.path.append(str(project_dir))
sys.path.append(str(gym_dir))