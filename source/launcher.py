"""Windowed entry with persistent diagnostics, including import failures."""
import os
import sys
import traceback
from pathlib import Path

os.environ.setdefault("OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS","0")
base = Path(sys.executable).parent if getattr(sys, 'frozen', False) else Path(__file__).resolve().parent.parent
log = (base/'runtime.log').open('a', encoding='utf-8', buffering=1)
sys.stdout = sys.stderr = log


def exception_hook(kind, value, tb):
    traceback.print_exception(kind, value, tb, file=log)


sys.excepthook = exception_hook
try:
    from app import main
    main()
except Exception:
    traceback.print_exc(file=log)
    raise SystemExit(1)
