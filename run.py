"""Launch RoarmStudio from a source checkout."""
import os
import runpy
from pathlib import Path
import sys

root = Path(__file__).resolve().parent
os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS", "0")
if sys.platform == "win32" and not any(arg in sys.argv for arg in ("--smoke", "--diagnose")):
    os.environ["QT_QPA_PLATFORM"] = "windows"
sys.path.insert(0, str(root / "source"))
runpy.run_path(str(root / "source" / "launcher.py"), run_name="__main__")
