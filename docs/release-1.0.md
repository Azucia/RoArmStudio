# RoarmStudio 1.0 · Windows x64

[下载 Windows 安装包](https://github.com/Azucia/RoArmStudio/releases/download/v1.0/RoarmStudio-1.0-Setup-x64.exe) · [查看全部 1.0 下载](https://github.com/Azucia/RoArmStudio/releases/tag/v1.0)

1.0 把 XYZ 逆解搬到了电脑。官方 Web 松开按键时会把已经下沉的反馈高度写回目标，在我们的带载机械臂上，每点一步都会再掉 Z。电脑端保留目标高度和首次接管前的保持偏差，后续 XY 点动不会逐步降低目标。详见仓库首页的源码说明和实机对照。

- **Setup-x64.exe** 为桌面安装程序，面向 Windows x64。
- **Windows-x64.zip** 为免安装版。完整解压后运行 RoarmStudio.exe。
- **Teach-app.bin** 为可选 ESP32 **应用分区**固件。包含直接释放、原位恢复扭矩、七舵机角度反馈和 USB 串口更新。首次装好后，后续更新可用 tools/update_firmware.py，无需再按 BOOT。它不是整片镜像，不能写到 0x0。安装桌面程序不会刷写机械臂。
- **SHA256SUMS.txt** 给出上述文件的 SHA-256。
- 源码包内含固件源码及七舵机角度监视页。回放对带载肩、肘的小幅稳定误差分别判断，未到位时会报告具体关节与角度。

By Azucia
