# 构建与仓库内容

- `source/`：桌面控制、电脑逆解、目标保持、补偿、示教、相机及自动测试。
- `assets/`、`docs/`：应用图标、截图、使用和发布说明。
- `firmware/RoArm-M3_example/`：本次 ESP32 固件的完整应用源码。
- `firmware/RoarmStudio-1.0-Teach-app.bin`：与实机写入校验一致的应用镜像。
- `firmware/verification.json`：刷写后的七舵机、示教与 USB 更新能力只读回读。它不是完整运动验收记录。
- `tools/`：独立关节监视页与 USB 固件更新客户端。
- `licenses/`：桌面依赖许可证；`firmware/dependencies.json` 记录固件依赖版本和来源。
- `.github/workflows/windows-release.yml`：Windows 测试、打包、安装启动验证与 Release 发布流程。

## Windows

使用 Python 3.12 x64，安装 `source/requirements.txt` 后执行：

```powershell
python tools/run_tests.py
python source/build.py
```

程序输出到 `程序/RoarmStudio/`。使用 Inno Setup 6 编译 `source/installer.iss`，传入 `SourceRoot` 为仓库绝对路径、`ReleaseOutput` 为安装包输出目录。完整步骤以发布工作流为准。

测试入口同时执行桌面和 USB 更新客户端的离线测试，使用临时设置文件并禁止真实串口、相机访问。普通源码变更和 PR 由独立测试工作流检查，不依赖修改版本号或发布安装包。

## ESP32

此镜像以 Arduino CLI、ESP32 Arduino core 3.3.10-cn、板型 `esp32:esp32:esp32` 编译。安装 `firmware/dependencies.json` 列出的库版本，再编译：

```powershell
arduino-cli compile --fqbn esp32:esp32:esp32 --output-dir firmware-build firmware/RoArm-M3_example
```

应用大小为 1,235,840 字节，SHA-256 为 `5bc22714ffc9af518e3e3dc7f2417a1bc1735fa001cccca11d37ef23d95d387d`。应用槽容量需至少 0x140000。不同工具链构建不保证二进制逐字节一致。刷写方法见 [固件说明](../firmware/README.md)。

仓库不包含个人相机采集、电脑连接配置、含设备网络设置的整片闪存备份、临时构建缓存。可安装程序发布在 GitHub Release，源代码与构建入口保存在 Git 中。
