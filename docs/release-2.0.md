# RoarmStudio 2.0 · Windows x64

- 「设备/连接」可启动仅本机可访问的接口，并复制地址和连接密钥。Studio 管理机械臂、手部相机和彩色深度融合相机。
- 本机 API 提供两路 MJPEG 与 JPEG 帧、对齐深度 PNG、设备状态、XYZ 移动、回中、开合夹爪和动作结果。控制权需显式取得，界面上的「急停」可取消待发动作并收回控制权。
- 发布独立 Python SDK wheel。GitHub 首页提供可复制的安装、取帧、动作及完整 HTTP 调用示例。
- XYZ 支持降标求解与黄色状态提示，关节控制可选择夹爪朝前或朝下 90° 回中。示教回放按轨迹时间推进，保留反馈和跟随误差检查。

安装包为 `RoarmStudio-2.0-Setup-x64.exe`，便携版为 `RoarmStudio-2.0-Windows-x64.zip`。SDK 可单独安装 `roarmstudio_sdk-2.0.0-py3-none-any.whl`。模拟串口与图像联调及 Windows 启动检查通过，实机联调待验证。
