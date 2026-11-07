# Third-party components

RoArm Studio uses PySide6/Qt, Shiboken6, NumPy, OpenCV, pyserial and the official Orbbec SDK Python bindings. Third-party license notices are included in the `licenses` directory and in the distributed Windows package.

Waveshare RoArm protocol references and Orbbec SDK references are listed in `source/来源.md`. This application is an independent desktop interface and is not an official Waveshare or Orbbec product. No vendor firmware is modified or flashed by this application.

Windows installers are built with Inno Setup. Installation is per user; captured user data is not part of the installer-managed file list.

UVC enumeration uses cv2-enumerate-cameras 1.4.0 (MIT). Its notice is included in licenses/cv2-enumerate-cameras-LICENSE.txt.

The read-only RoArm-M3 model uses Waveshare's MIT-declared roarm_description geometry. Its attribution, license and source revision are in assets/roarm_m3/NOTICE.md.
