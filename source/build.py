"""Run with the Python environment containing requirements.txt."""
import importlib.metadata
import pathlib
import site
import subprocess
import sys

root = pathlib.Path(__file__).resolve().parent
destination = root.parent
site_dir = pathlib.Path(importlib.metadata.distribution('pyorbbecsdk2').locate_file(''))
args = [sys.executable, '-m', 'PyInstaller', '--noconfirm', '--clean', '--windowed',
        '--name', 'RoArmStudio', '--distpath', str(destination/'程序'),
        '--workpath', str(destination.parent.parent/'work'/'studio-build'),
        '--specpath', str(destination.parent.parent/'work'),
        '--hidden-import', 'pyorbbecsdk', '--exclude-module', 'PySide6.QtWebEngineCore',
        '--exclude-module', 'PySide6.QtWebEngineWidgets',
        '--add-binary', f'{site_dir / "OrbbecSDK.dll"};.',
        '--add-data', f'{site_dir / "extensions"};extensions',
        '--add-data', f'{site_dir / "OrbbecSDKConfig.xml"};.',
        str(root/'launcher.py')]
subprocess.run(args, check=True)
# Qt 6.12 uses Windows' system ICU. Do not ship an unrelated ICU picked up
# from another application's PATH (e.g. Poppler), which has different exports.
wrong_icu = destination/'程序'/'RoArmStudio'/'_internal'/'icuuc.dll'
if wrong_icu.exists():
    wrong_icu.unlink()
