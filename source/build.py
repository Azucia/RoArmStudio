"""Run with the Python environment containing requirements.txt."""
import importlib.metadata
import pathlib
import shutil
import subprocess
import sys

root = pathlib.Path(__file__).resolve().parent
destination = root.parent
site_dir = pathlib.Path(importlib.metadata.distribution('pyorbbecsdk2').locate_file(''))
args = [sys.executable, '-m', 'PyInstaller', '--noconfirm', '--clean', '--windowed',
        '--name', 'RoarmStudio', '--distpath', str(destination/'程序'),
        '--workpath', str(destination/'work'/'studio-build'),
        '--specpath', str(destination/'work'),
        '--icon', str(destination/'assets/arm-drawing-rounded.ico'),
        '--hidden-import', 'pyorbbecsdk', '--collect-binaries', 'cv2_enumerate_cameras', '--exclude-module', 'PySide6.QtWebEngineCore',
        '--exclude-module', 'PySide6.QtWebEngineWidgets',
        '--add-binary', f'{site_dir / "OrbbecSDK.dll"};.',
        '--add-data', f'{site_dir / "extensions"};extensions',
        '--add-data', f'{site_dir / "OrbbecSDKConfig.xml"};.',
        str(root/'launcher.py')]
subprocess.run(args, check=True)
# Qt 6.12 uses Windows' system ICU. Do not ship an unrelated ICU picked up
# from another application's PATH (e.g. Poppler), which has different exports.
wrong_icu = destination/'程序'/'RoarmStudio'/'_internal'/'icuuc.dll'
if wrong_icu.exists():
    wrong_icu.unlink()

shutil.copytree(destination/"assets", wrong_icu.parent.parent/"assets", dirs_exist_ok=True)
