# RoarmStudio 2.0

RoarmStudio 是为 RoArm-M3-Pro 开发的 Windows 桌面工作台。**在V1.0版本以后，有关XYZ的逆运动学全部在软件端计算。** 用户给出目标XYZ，软件端将**需要到达的坐标**换算成机械臂的六个关节角，通过 USB 串口直接发一条 JSON 指令（`T=102`）。机械臂通过 `T=105` 回报关节角和模型坐标。这条控制路径无需刷写固件。

## 为什么官方 Web 每点一步都掉 Z？

在带相机机械臂上，用官方 Web 按键反复横移，**每点一步，末端都会再往下掉一点**。改用电脑逆解后，后续 XY 横移基本能守住高度。差别在于松开按钮以后，谁记住原来要保持的高度。

**机械臂为什么会先低一点？** 肩和肘要顶住手臂自重和相机负载。舵机存在机械虚位，负载下的实测角也会偏离指令角。位置环的 PID 参数决定它如何对付这个偏差。一种可能的机制是：如果主要靠比例项，而且积分或重力补偿不足，就要留下一点角度差，控制器才能持续输出足够的 PWM 来顶住手臂。此时“要求它停在的角度”和“它实际停住的角度”可以长期不同，导致普遍的执行不到位。

**为什么官方 Web 会把一次偏差变成步步下沉？** 保存的主控源码里，按键松开后，`constantHandle()` 会执行 `goalZ = lastZ`，把反馈得到的高度写回目标。假设目标是 200 mm，负载让反馈停在 195 mm。再点 Y 时，就会从 **195 mm** 横移；如果又低一点，下次还会从更低的位置开始。200 和 195 是说明原理的例子。仅仅改成发送 JSON，如果每次仍以反馈高度重建目标，也会保留这个问题。

**RoarmStudio做了什么**

1. **记住目标高度。** 在同一段点动中，X/Y 指令只改变选中的轴。下一步继续沿用原来的目标 Z，读到的下沉留作反馈，不会改写下一步的目标。
2. **第一步接住原来的负载。** 回中后，软件记住上一条真正发出的关节目标。开始点动时，把它与带载实测姿态对齐，保留原先托住手臂所需的偏差。这样，第一次点 Z+10 是从原先的保持目标往上走。
3. **给负载留出提前量。** Z 前馈可设为 0～50 mm，用以抵消末端负载变化导致的Z轴非受控下降。

## 一组实机对照

同一台带载机械臂，使用相同速度和加速度，各发一条关节目标，随后读取 8 秒反馈。

| 指令 | 主控反馈 |
| --- | --- |
| 重发原来托住手臂的关节目标 | Z 保持在 190.859 mm |
| 把刚读到的六个实测角直接作为新目标，要求的空间位移为零 | Z 从 190.859 降到 181.227 mm，下降 9.632 mm；肩变动约 1.23°，肘约 0.35° |

这解释了为什么 **即使没有要求向下运动，替换保持目标也可能让机械臂下沉**。另一段旧版日志里，回中的保持目标折算为 Z 223.1 mm，实测只有 195.7 mm。旧算法点 Z+10 再加 8 mm 前馈，只算到 213.7 mm，反而比原来的保持目标低 9.4 mm。1.0 对应的首次接管算法在这组输入下算到 233.1 mm。

以上毫米数来自舵机编码器和主控几何模型，同时实际观测中下沉得到改善；但是由于缺乏闭环绝对位置参考，剩余的单步误差仍会随姿态、负载和装配变化。

## 使用

Windows x64 用户可以从 [v1.1 发布页](https://github.com/Azucia/RoArmStudio/releases/tag/v1.1) 下载安装包或免安装版。电脑端源码启动方式：

```powershell
py -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r source/requirements.txt
.\.venv\Scripts\python.exe run.py
```

打开「控制/连接」，连接机械臂并启用「电脑逆解」。让机械臂有空间安全回中，再开始 XYZ 点动。使用方法见[操作说明](使用说明.md)。

- **XYZ 和关节控制**：1 mm / 10 mm 点动、绝对坐标输入、关节角度控制、带载 Z 前馈。不可达目标会在发送前被拒绝。
- **示教与回放**：保存关节轨迹、调整回放倍率，按实测到位情况推进。倍率改变计划时间，实际速度仍受关节速度限制。
- **相机与采集**：AR0234 彩色预览、Gemini 2 彩色/深度/对齐叠加、快照与记录；支持按设备身份恢复连接。
- **界面与诊断**：三种外观；串口命令和关节反馈日志可用于复盘一次动作。

## 后续计划：给其他软件使用的 SDK

计划把电脑端已验证的控制逻辑开放为 SDK。第一阶段拟提供连接与状态读取、绝对 XYZ 和相对点动、动作取消与错误反馈。外部视觉、自动化和采集软件可以复用同一套目标保持与带载补偿，不必各写一遍逆解。

安全示教协议、可选固件源码和独立角度监视页分别见 [firmware](firmware) 与 [tools](tools)。完整目录、依赖及打包方法见[构建说明](docs/build.md)。USB 更新客户端可用 `python tools/test_update_firmware.py` 离线测试。

By Azucia

或许我们的目标是"一种便捷的机械臂逆解解决方案”，我希望看到的是，一堆随便锯的铝型材+混搭的不知名舵机，只要输入正确的关节骨骼数据，就能在RoArm Studio的驱动下达到目标XYZ。

## v1.2 更新

- 重做黑灰视觉工作台。标题与窗口按钮合为一行，顶部标签只切换左上控制区。轮盘、实时 XYZ 数值和精确输入框都已放大。
- 顶部状态条显示连接、反馈间隔和扭矩状态。左下可切换日志、关节角度和温度，右下显示手部相机与彩色深度融合画面。
- 右上使用 Waveshare 的 RoArm-M3 模型显示实际关节姿态。工作区网格保留前方区域，支持滚轮缩放、左键拖动旋转和右键拖动平移。
- 新增前方 210° 软件限位。回放可显示「严格执行」或「降标执行」。红色「急停」可取消待发动作和回放，已发送的目标可能继续执行。

[下载 v1.2 Windows x64 安装包与便携版](https://github.com/Azucia/RoArmStudio/releases/tag/v1.2)

## v2.0 本机 API 与 Python SDK

在 RoArmStudio 顶部打开「设备/连接」，连接机械臂和两台相机。点击「启动接口」和「复制连接信息」。Studio 提供两个固定画面名称，`hand` 是手部彩色，`workspace` 是彩色深度融合。

安装包、便携版和 SDK 位于 [v2.0 发布页](https://github.com/Azucia/RoArmStudio/releases/tag/v2.0)，SDK 源码位于 [sdk](sdk)。接口仅监听 `127.0.0.1`，每次启动都会生成新的连接密钥。

### 复制运行 · Python

先在 Windows PowerShell 执行以下命令。第二行把刚才从 Studio 复制的连接信息保存到当前目录。

```powershell
py -m pip install "https://github.com/Azucia/RoArmStudio/releases/download/v2.0/roarmstudio_sdk-2.0.0-py3-none-any.whl"
Get-Clipboard -Raw | Set-Content -Encoding utf8 studio-connection.json
```

保存为 `read_studio.py`，执行 `py read_studio.py`。下面读取状态、两路画面及对齐深度。

```python
from pathlib import Path
from roarmstudio_sdk import Studio

connection = Path("studio-connection.json").read_text(encoding="utf-8-sig")
with Studio.from_connection(connection) as studio:
    print(studio.status())
    for camera in ("hand", "workspace"):
        jpg, host_ns = studio.frame(camera)
        Path(f"{camera}.jpg").write_bytes(jpg)
        print(camera, host_ns)
    png, host_ns, scale_mm = studio.depth()
    Path("workspace-depth.png").write_bytes(png)
    print("深度单位，毫米/像素值", scale_mm, "时间戳", host_ns)
```

持续读取时遍历 `studio.stream("hand")` 或 `studio.stream("workspace")`，每次得到 `(jpeg_bytes, host_ns)`。停止读取时关闭迭代器。

确认机械臂在线、路径有空间后，可执行以下动作示例。它从反馈位置请求 X 方向移动 1 mm，并等到反馈确认。按业务需要也可以调用 `jog_xyz(dx, dy, dz)`、`home("down")`、`grip(True)`。相对点动需要已发送的保持目标，可通过明确回中建立。

```python
from pathlib import Path
from roarmstudio_sdk import Studio

with Studio.from_connection(Path("studio-connection.json").read_text(encoding="utf-8-sig")) as studio:
    studio.acquire()
    x, y, z = studio.status()["xyz_mm"]
    action = studio.move_xyz(x + 1, y, z)
    result = studio.wait(action["id"])
    print(result)
```

`status()` 可随时查询设备，`action(id)` 查询单次动作。离开 `with` 时自动调用 `release()`。需要主动取消时调用 `cancel_pending()`。调用方自己轮询时可用 `renew()` 续期，`wait()` 会自动续期。接口拒绝时抛出 `SDKError`，可读取 `code` 和 `status`。

### 复制运行 · HTTP

其他语言可以直接使用以下 HTTP API。在 PowerShell 执行以下命令，读取状态并保存两路画面。

```powershell
$studio = Get-Content -Raw studio-connection.json | ConvertFrom-Json
$headers = @{ Authorization = "Bearer $($studio.token)" }
Invoke-RestMethod "$($studio.url)/v1/status" -Headers $headers
Invoke-WebRequest "$($studio.url)/v1/cameras/hand/frame.jpg" -Headers $headers -OutFile hand.jpg
Invoke-WebRequest "$($studio.url)/v1/cameras/workspace/frame.jpg" -Headers $headers -OutFile workspace.jpg
```

持续订阅将以下地址交给支持 HTTP 请求头的 MJPEG 客户端。停止订阅时关闭连接即可。

```powershell
curl.exe --no-buffer -H "Authorization: Bearer $($studio.token)" "$($studio.url)/v1/cameras/hand/stream.mjpg" --output hand.mjpg
```

确认机械臂在线且运动路径有空间后，以下示例申请控制权、提交 XYZ 动作、续期并等待结果。

```powershell
$lease = Invoke-RestMethod "$($studio.url)/v1/lease" -Method Post -Headers $headers -ContentType "application/json" -Body '{}'
$headers["X-Control-Lease"] = $lease.lease
try {
    $state = Invoke-RestMethod "$($studio.url)/v1/status" -Headers $headers
    if ($null -eq $state.xyz_mm) { throw "机械臂缺少有效位置反馈" }
    $body = @{
        request_id = [guid]::NewGuid().ToString()
        kind = "move_xyz"
        xyz_mm = @(($state.xyz_mm[0] + 1), $state.xyz_mm[1], $state.xyz_mm[2])
        allow_degraded = $false
    } | ConvertTo-Json -Compress
    $action = Invoke-RestMethod "$($studio.url)/v1/actions" -Method Post -Headers $headers -ContentType "application/json" -Body $body
    while ($action.state -in @("queued", "sent")) {
        Start-Sleep -Milliseconds 250
        $action = Invoke-RestMethod "$($studio.url)/v1/actions/$($action.id)" -Headers $headers
        if ($action.state -in @("queued", "sent")) {
            Invoke-RestMethod "$($studio.url)/v1/lease/renew" -Method Post -Headers $headers -ContentType "application/json" -Body '{}' | Out-Null
        }
    }
    $action
} finally {
    Invoke-RestMethod "$($studio.url)/v1/lease/release" -Method Post -Headers $headers -ContentType "application/json" -Body '{}' -ErrorAction SilentlyContinue
}
```

### HTTP 接口一览

所有请求带 `Authorization: Bearer <token>`。POST 使用 `Content-Type: application/json`，空请求体传 `{}`。标有「控制权」的请求还需 `X-Control-Lease: <lease>`。

| 方法和路径 | 请求与结果 |
| --- | --- |
| GET `/v1/status` | 返回 `connected`、`feedback_age_ms`、`feedback_host_ns`、`xyz_mm`、`joint_deg`、`torque_enabled`、`external_control`、`cameras`。过期反馈的坐标、角度和扭矩为 null。 |
| GET `/v1/cameras/hand/frame.jpg` | 手部 JPEG，`X-Frame-Host-Ns` 为主机时间戳。 |
| GET `/v1/cameras/workspace/frame.jpg` | 彩色深度融合 JPEG，同样带时间戳。 |
| GET `/v1/cameras/hand/stream.mjpg` | 手部 MJPEG，帧各自带时间戳。 |
| GET `/v1/cameras/workspace/stream.mjpg` | 彩色深度融合 MJPEG，帧各自带时间戳。 |
| GET `/v1/cameras/workspace/depth.png` | 与工作区彩色像素对齐的 16 位深度 PNG。像素值乘以 `X-Depth-Scale-Mm` 得到毫米。 |
| POST `/v1/lease` | 返回 `lease` 和 `ttl_s`。一次一个外部控制方。 |
| POST `/v1/lease/renew` | 控制权。续期 15 秒。Python `wait()` 自动续期。 |
| POST `/v1/lease/release` | 控制权。取消待发动作并释放控制权。 |
| POST `/v1/actions` | 控制权。接收 `request_id`、`kind` 及动作参数，返回动作编号与计划。 |
| GET `/v1/actions/{id}` | 查询动作状态、串口写出结果和接受的 XYZ。 |
| POST `/v1/cancel` | 控制权。取消待发动作并释放控制权。已写出串口的目标可能继续执行。 |

动作 JSON 示例。每次新动作生成一个新的 UUID。相同控制权内，用相同 UUID 和参数重试返回同一动作。

```json
{"request_id":"18a60ddb-c0a7-4da8-9cc0-69f77a16640d","kind":"move_xyz","xyz_mm":[345,0,223],"allow_degraded":false}
```

| `kind` | 参数 |
| --- | --- |
| `move_xyz` | `xyz_mm: [x,y,z]`，底座坐标系中的毫米。可选 `allow_degraded`，默认 false。 |
| `jog_xyz` | `xyz_mm: [dx,dy,dz]`，毫米增量。可选 `allow_degraded`，默认 false。 |
| `home` | 可选 `direction` 为 `forward` 或 `down`，省略时使用界面设置。 |
| `grip` | `closed: true` 收紧，`false` 打开。 |

结果包含 `id`、`state`、`serial_written`、`degraded`、`accepted_xyz_mm`、`planned_xyz_mm`、`planned_pitch_deg` 和 `message`。状态为 `queued`、`sent`、`reached`、`degraded_reached`、`cancelled`、`failed` 或 `timed_out`。XYZ 到位以新反馈确认，位置容差为 3 mm，并检查关节跟随。动作超时为 30 秒。动作结果保留到下一次取得控制权或停止接口。控制权到期、界面取消、反馈过期或扭矩状态异常都会阻止后续发送。

错误以 JSON 返回 `error` 和 `message`。常见错误为 `unauthorized`（401）、`control_busy`、`action_busy`、`invalid_lease`（409）、`motion_rejected`（422）、`camera_offline`（503）。画面时间戳来自主机，两路画面分别更新。
