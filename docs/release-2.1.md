# RoarmStudio 2.1 · Windows x64

- 示教保存时自动简化中间点，并保留原始时间轴、起止点、停顿、明显转折和夹爪动作。
- 简化时在原采样时刻检查关节与末端模型误差。合并后的末端模型偏差上限为 1 mm。示教页显示原始采样数与保留的关键点数。
- 现有示教轨迹可以照常载入。本机 API 沿用 `/v1`，Python SDK wheel 随本次发布更新为 2.1.0。

下载安装包或完整解压便携版后运行 `RoarmStudio.exe`。SDK 安装和调用示例见 [GitHub 首页](https://github.com/Azucia/RoArmStudio#v20-本机-api-与-python-sdk)。

离线轨迹测试与 Windows 安装启动检查通过。实机回放待验证。
