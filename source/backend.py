"""RoArm M3 protocol and camera workers. No motion on connection."""
import json
import math
import queue
import threading
import time
import uuid
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import serial

KEYS = ('b', 's', 'e', 't', 'r', 'g')
NAMES = ('底座', '肩部', '肘部', '腕部俯仰', '腕部旋转', '夹爪')
# Conservative operating envelope, in firmware/native degrees (no SDK gripper inversion).
LIMITS = ((-180, 180), (-90, 90), (0, 180), (-90, 90), (-180, 180), (math.degrees(1.08), 180))
MOTION_TYPES = (101, 102, 1041)
FEEDBACK_TIMEOUT = .6


def torque_disabled(pose):
    return any((pose or {}).get('torswitch'+k) == 0 for k in ('B', 'S', 'E', 'T', 'R', 'G'))


def check_radians(values):
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError('需要六个关节弧度')
    for i, value in enumerate(values):
        joint_command(i, math.degrees(value))
    return values


def all_command(values, speed=150, acc=5):
    check_radians(values)
    command = hold_command(dict(zip(KEYS, values)))
    command.update(spd=speed, acc=acc)
    return command


def pose_command(values, measured_angles=None):
    ranges = ((-500, 500), (-500, 500), (-600, 600), (-90, 90), (-180, 180), LIMITS[-1])
    if len(values) != 6:
        raise ValueError('末端目标需要六个数值')
    for i, (v, (low, high)) in enumerate(zip(values, ranges)):
        if not math.isfinite(v):
            raise ValueError(f'{("X", "Y", "Z", "俯仰", "旋转", "夹爪")[i]}不是有限数值')
        # Cartesian jog preserves measured joints, including small encoder offsets
        # beyond a command limit. Never clamp them or silently move the gripper.
        preserved = (i >= 3 and measured_angles is not None and len(measured_angles) == 3
                     and abs(v) <= 360 and math.isclose(v, measured_angles[i-3], abs_tol=1e-9, rel_tol=0))
        if not low <= v <= high and not preserved:
            unit = 'mm' if i < 3 else '°'
            raise ValueError(f'{("X", "Y", "Z", "俯仰", "旋转", "夹爪")[i]}目标 {v:.3f}{unit} 超出软件范围 {low:.3f}～{high:.3f}{unit}')
    return dict(T=1041, **dict(zip(('x', 'y', 'z', 't', 'r', 'g'), values[:3]+[math.radians(v) for v in values[3:]])))


def public_command(command):
    return {k: ('***' if 'password' in k.lower() else v) for k, v in command.items()}


def valid_pose(data):
    return (data.get('T') == 1051 and all(
        isinstance(data.get(k), (int, float)) and math.isfinite(data[k])
        for k in KEYS))


def joint_command(index, degrees, speed=150):
    lo, hi = LIMITS[index]
    if not math.isfinite(degrees) or not lo <= degrees <= hi:
        raise ValueError(f'{NAMES[index]}超出软件限位 {lo}～{hi}°')
    return dict(T=101, joint=index + 1, rad=math.radians(degrees), spd=speed, acc=5)


def hold_command(pose):
    return dict(T=102, **dict(zip(('base', 'shoulder', 'elbow', 'wrist', 'roll', 'hand'),
                                [pose[k] for k in KEYS])), spd=150, acc=5)


class Shared:
    def __init__(self):
        self.lock = threading.Lock()
        self.frames = {}
        self.pose = None
        self.pose_time = 0
        self.pose_host_ns = 0
        self.events = queue.Queue()
        self.frame_times = {}
        self.torque_reply = None

    def publish(self, key, images, **meta):
        with self.lock:
            now = time.monotonic()
            times = self.frame_times.setdefault(key, deque(maxlen=120))
            if key in self.frames and now-self.frames[key]['monotonic'] > 1:
                times.clear()
            # perf_counter has sub-frame precision on Windows; monotonic can tick
            # only every 15.6ms and multiple delivered frames may share one tick.
            times.append(time.perf_counter())
            fps = (len(times)-1)/(times[-1]-times[0]) if len(times)>1 and times[-1]>times[0] else 0
            self.frames[key] = dict(images=images, host_ns=time.time_ns(),
                                    monotonic=now, fps=fps, **meta)

    def event(self, message):
        self.events.put(message)


class Worker(threading.Thread):
    def __init__(self, shared):
        super().__init__(daemon=True)
        self.shared = shared
        self.stop = threading.Event()


class CommandReceipt:
    """Local serial delivery result, not a device acknowledgement or arrival."""
    def __init__(self):
        self.done = threading.Event()
        self.error = None

    def finish(self, error=None):
        self.error = error
        self.done.set()


def discard_command(command, reason):
    receipt = command.get('_receipt')
    if receipt is not None:
        receipt.finish(reason)


class Arm(Worker):
    def __init__(self, shared, port):
        super().__init__(shared)
        self.port = port
        self.commands = queue.Queue(maxsize=8)
        self.connected = False
        self.teaching = False
        self.command_lock = threading.RLock()
        self.epoch = 0
        self.playing = False
        self.torque_pending = False
        self.torque_fault = False
        self.torque_state = '未确认'
        self.last_joint_target = None
        self.command_sequence = 0
        self.host_busy = False
        self.reference_generation = 0
        self.trace_file = None

    def trace(self, event, **values):
        if self.trace_file is None:
            return
        try:
            self.trace_file.write(json.dumps(dict(event=event, host_ns=time.time_ns(),
                monotonic=time.monotonic(), **values), ensure_ascii=False)+'\n')
            self.trace_file.flush()
        except OSError:
            try:
                self.trace_file.close()
            except OSError:
                pass
            self.trace_file = None
            self.shared.event('串口诊断日志写入失败，已停止记录；控制方式未改变。')

    def submit(self, command, token=None):
        with self.command_lock:
            return self._submit(command, token)

    def cancel_replay(self):
        with self.command_lock:
            self.epoch += 1
            self.playing = False
            self.host_busy = False
            while not self.commands.empty():
                try:
                    discard_command(self.commands.get_nowait(), '指令已取消')
                except queue.Empty:
                    break

    def begin_replay(self):
        with self.command_lock:
            if self.teaching or self.torque_pending or self.torque_fault:
                raise ValueError('扭矩未确认保持，不能开始回放')
            self.cancel_replay()
            self.playing = True
            return self.epoch

    def _submit(self, command, token=None):
        if not self.connected or self.stop.is_set():
            raise ValueError('机械臂尚未连接')
        if token is not None and (token != self.epoch or not self.playing):
            raise ValueError('回放已取消')
        if command.get('T') == 210:
            raise ValueError('已禁用旧扭矩指令：当前固件可能先移动关节；请使用经过确认的安全示教接口')
        if (self.torque_pending or self.torque_fault) and command.get('T') in MOTION_TYPES:
            raise ValueError('扭矩状态正在确认或确认失败，已禁止运动')
        if self.teaching and command.get('T') in MOTION_TYPES:
            raise ValueError('请先结束手动示教')
        if command.get('T') in MOTION_TYPES and torque_disabled(self.shared.pose):
            raise ValueError('检测到关节扭矩关闭，请先确认恢复保持')
        if self.playing and token is None:
            raise ValueError('请先停止回放')
        # Replace pending motion, but preserve ordered device-setting commands.
        if command.get('T') in MOTION_TYPES:
            settings = []
            while True:
                try:
                    pending = self.commands.get_nowait()
                    if pending.get('T') not in MOTION_TYPES:
                        settings.append(pending)
                    else:
                        discard_command(pending, '指令已被后续目标替代')
                except queue.Empty:
                    break
            for pending in settings:
                self.commands.put_nowait(pending)
        command = dict(command)
        receipt = None
        if command.get('T') in MOTION_TYPES:
            command['_epoch'] = self.epoch
            receipt = command['_receipt'] = CommandReceipt()
        try:
            self.commands.put_nowait(command)
        except queue.Full:
            raise ValueError('待发送设置较多，请稍后重试')
        return receipt

    def _write_pending(self, ser):
        """Check cancellation and feedback again at the serial write boundary."""
        with self.command_lock:
            stale = time.monotonic()-self.shared.pose_time > FEEDBACK_TIMEOUT
            if self.stop.is_set():
                self.cancel_replay()
                return
            if stale and self.last_joint_target is not None:
                self.last_joint_target = None
                self.reference_generation += 1
            if self.playing and (stale or torque_disabled(self.shared.pose)):
                self.cancel_replay()
                self.shared.event('反馈过期或关节扭矩关闭，已取消回放；不会自动续播。')
            try:
                cmd = self.commands.get_nowait()
            except queue.Empty:
                return
            epoch = cmd.pop('_epoch', self.epoch)
            host_intent = cmd.pop('_host_intent', False)
            receipt = cmd.pop('_receipt', None)
            rejected = None
            if epoch != self.epoch:
                rejected = '已丢弃取消前的运动指令'
            elif cmd.get('T') in MOTION_TYPES and stale:
                rejected = '反馈已过期，已丢弃运动指令'
            elif cmd.get('T') in MOTION_TYPES and (self.teaching or self.torque_pending or
                    self.torque_fault or torque_disabled(self.shared.pose)):
                rejected = '扭矩未确认保持，已丢弃运动指令'
            if rejected:
                self.shared.event(rejected)
                if receipt is not None:
                    receipt.finish(rejected)
            else:
                packet = (json.dumps(cmd)+'\n').encode()
                try:
                    if ser.write(packet) != len(packet):
                        raise IOError('串口指令未完整写出，控制基准已失效')
                except Exception as e:
                    self.last_joint_target = None
                    self.reference_generation += 1
                    if receipt is not None:
                        receipt.finish(str(e))
                    raise
                self.trace('serial_write', command=public_command(cmd), host_intent=host_intent)
                if cmd.get('T') == 102:
                    self.last_joint_target = [cmd[k] for k in ('base', 'shoulder', 'elbow', 'wrist', 'roll', 'hand')]
                elif cmd.get('T') == 101 and self.last_joint_target is not None:
                    self.last_joint_target[int(cmd['joint'])-1] = cmd['rad']
                elif cmd.get('T') in (1041, 210, 211):
                    self.last_joint_target = None
                if cmd.get('T') in MOTION_TYPES:
                    self.command_sequence += 1
                    if not self.host_busy and not host_intent:
                        self.reference_generation += 1
                self.shared.event('发送 '+json.dumps(public_command(cmd), ensure_ascii=False))
                if receipt is not None:
                    receipt.finish()

    def _accept_pose(self, data):
        with self.command_lock:
            gap = time.monotonic()-self.shared.pose_time > FEEDBACK_TIMEOUT
            if gap or any(data.get('torswitch'+key) != 1 for key in ('B', 'S', 'E', 'T', 'R', 'G')):
                self.last_joint_target = None
                self.reference_generation += 1
            # Detect the gap before refreshing the timestamp, including when the
            # serial thread itself was delayed and did not run its write guard.
            if self.playing and (gap or torque_disabled(data)):
                self.cancel_replay()
                self.shared.event('反馈中断或关节扭矩关闭，已取消回放；不会自动续播。')
            with self.shared.lock:
                self.shared.pose = data
                self.shared.pose_time = time.monotonic()
                self.shared.pose_host_ns = time.time_ns()
            if self.teaching and not self.torque_pending and not self.torque_fault:
                switches = ('torswitchB', 'torswitchS', 'torswitchE', 'torswitchT', 'torswitchR', 'torswitchG')
                if any(data.get(key) != 0 for key in switches):
                    self.torque_fault = True
                    self.torque_state = '示教扭矩状态异常 · 请继续托住'
                    self.shared.event(self.torque_state)

    def run(self):
        ser = None
        try:
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout, ser.write_timeout = self.port, 115200, .03, .3
            ser.dtr = ser.rts = False
            ser.open()
            try:
                folder=Path(__file__).resolve().parent.parent/'diagnostics'
                folder.mkdir(parents=True,exist_ok=True)
                self.trace_path=folder/(time.strftime('joint-session-%Y%m%d-%H%M%S-')+uuid.uuid4().hex[:8]+'.jsonl')
                self.trace_file=self.trace_path.open('x',encoding='utf-8')
                self.trace('session',port=self.port,transport_class=type(ser).__module__+'.'+type(ser).__name__,
                    measurement='encoder feedback and board XYZ, not external measurement',
                    version='direct-joint-passive-log-20261103')
            except OSError:
                self.shared.event('未能创建串口诊断日志；控制方式未改变。')
            self.connected = True
            self.shared.event(f'机械臂串口已打开：{self.port}；等待实际反馈')
            last_query = 0
            awaiting_feedback = False
            buf = bytearray()
            while not self.stop.is_set():
                self._write_pending(ser)
                if time.monotonic()-last_query >= (2 if awaiting_feedback else .1):
                    ser.write(b'{"T":105}\n')
                    last_query = time.monotonic()
                    awaiting_feedback = True
                buf.extend(ser.read(max(1, min(ser.in_waiting, 8192))))
                while b'\n' in buf:
                    line, _, buf = buf.partition(b'\n')
                    try:
                        start = line.find(b'{')
                        data = json.loads(line[start:]) if start >= 0 else {}
                        if isinstance(data, dict) and data.get('T') == 2111:
                            with self.shared.lock:
                                self.shared.torque_reply = data
                        if isinstance(data, dict) and valid_pose(data):
                            awaiting_feedback=False
                            self.trace('feedback',feedback=data)
                            self._accept_pose(data)
                    except (ValueError, UnicodeError):
                        pass
                if len(buf) > 65536:
                    buf.clear()
        except Exception as e:
            self.trace('connection_error',error=str(e))
            self.shared.event(f'机械臂连接错误：{e}')
        finally:
            with self.command_lock:
                self.connected = False
                self.cancel_replay()
                self.last_joint_target = None
            if ser:
                ser.close()
            self.trace('session_end')
            if self.trace_file:
                try:
                    self.trace_file.close()
                except OSError:
                    pass
                self.trace_file = None
            self.shared.event('机械臂串口已关闭（未发送释放扭矩指令）')


class TorqueChange(Worker):
    """Never use legacy T210. Only the patched, read-back-verified protocol may
    release torque or latch freshly measured raw servo positions on restore."""
    def __init__(self, shared, arm, release):
        super().__init__(shared)
        self.arm, self.release = arm, release
        self.error = None
        with arm.command_lock:
            if not arm.connected or arm.torque_pending:
                raise ValueError('机械臂未连接或正在切换扭矩')
            arm.cancel_replay()
            arm.torque_pending = True
            arm.torque_state = '正在核对固件'

    def exchange(self, cmd):
        request = uuid.uuid4().hex
        self.arm.submit(dict(T=211, cmd=cmd, request=request))
        deadline = time.monotonic()+2.5
        while time.monotonic()<deadline and not self.stop.is_set():
            if not self.arm.connected:
                raise RuntimeError('机械臂连接已中断')
            with self.shared.lock:
                reply = self.shared.torque_reply
            if reply and reply.get('request') == request:
                if reply.get('protocol') != 'studio-teach-v1':
                    raise RuntimeError('固件安全示教接口版本不匹配')
                return reply
            self.stop.wait(.02)
        raise RuntimeError('未收到安全示教确认；旧固件已被拦截，不会回退发送 T210')

    def run(self):
        changed = False
        supported = False
        try:
            probe = self.exchange(2)
            supported = True
            if not probe.get('ok'):
                raise RuntimeError('固件无法读到全部七个舵机，已取消扭矩切换')
            self.arm.torque_state = '正在释放' if self.release else '正在锁定当前位置'
            changed = True
            reply = self.exchange(0 if self.release else 1)
            expected = 0 if self.release else 1
            if (not reply.get('ok') or reply.get('states') != [expected]*7):
                raise RuntimeError('七个舵机扭矩回读不一致，未确认切换成功；请继续托住机械臂')
            with self.arm.command_lock:
                self.arm.teaching = self.release
                self.arm.torque_fault = False
                self.arm.torque_state = '已释放 · 七舵机确认' if self.release else '已保持当前位置 · 七舵机确认'
            self.shared.event(self.arm.torque_state)
        except Exception as e:
            self.error = str(e)
            with self.arm.command_lock:
                self.arm.torque_fault = self.arm.torque_fault or changed or supported
                self.arm.torque_state = '切换未确认' if changed else ('舵机状态未确认' if supported else '需要安全示教固件')
            self.shared.event(f'手动示教：{e}')
        finally:
            with self.arm.command_lock:
                self.arm.torque_pending = False


class UVC(Worker):
    def __init__(self, shared, index, fps=60, preferred=None):
        super().__init__(shared)
        self.index = index
        self.fps = fps
        self.preferred=preferred
        self.identity=None
        self.remembered=False

    def run(self):
        cap = None
        com_initialized = False
        try:
            # DirectShow and this device's MF driver require COM on the capture thread.
            import ctypes
            hr = ctypes.windll.ole32.CoInitializeEx(None, 2)
            if hr not in (0, 1):
                raise RuntimeError(f'相机采集线程初始化失败：COM {hr:#x}')
            com_initialized = True
            backend = cv2.CAP_DSHOW
            index = self.index
            from cv2_enumerate_cameras import enumerate_cameras
            camera=None
            backends=(cv2.CAP_MSMF,cv2.CAP_DSHOW) if self.preferred or index==-1 else (cv2.CAP_DSHOW,)
            for backend in backends:
                candidates=list(enumerate_cameras(backend))
                if self.preferred:
                    matches=[d for d in candidates if d.path and d.path.casefold()==self.preferred]
                elif self.index==-1:
                    matches=[d for d in candidates if 'DECXIN' in d.name.upper() or 'AR0234' in d.name.upper()]
                else:
                    matches=[d for d in candidates if d.index==self.index or d.index==backend+self.index]
                if len(matches)!=1:
                    continue
                camera=matches[0]
                index=camera.index
                cap=cv2.VideoCapture(index,backend)
                if cap.isOpened():
                    break
                cap.release()
            if cap is None:
                raise RuntimeError('未找到指定的 UVC 相机')
            if not cap.isOpened():
                raise RuntimeError('无法打开所选 UVC 摄像头')
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            cap.set(cv2.CAP_PROP_FPS, self.fps)
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            self.shared.event(f'AR0234 请求 {self.fps}fps；驱动报告 {cap.get(cv2.CAP_PROP_FPS):.1f}fps，以实际收帧率为准')
            failures = 0
            received = False
            while not self.stop.is_set():
                ok, image = cap.read()
                if not ok:
                    failures += 1
                    if failures > 30:
                        raise RuntimeError('摄像头持续无帧，请检查连接或占用')
                    self.stop.wait(.05)
                    continue
                failures = 0
                if not received:
                    self.shared.event(f'AR0234 / UVC 已收到实际图像，索引 {index}')
                    self.identity=camera.path.casefold() if camera.path else None
                    received = True
                self.shared.publish('uvc', {'rgb': image})
        except Exception as e:
            self.shared.event(f'AR0234：{e}')
        finally:
            if cap:
                cap.release()
            if com_initialized:
                ctypes.windll.ole32.CoUninitialize()


def color_bgr(frame, sdk):
    data = np.asanyarray(frame.get_data())
    h, w = frame.get_height(), frame.get_width()
    fmt = frame.get_format()
    if fmt == sdk.OBFormat.MJPG:
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    elif fmt == sdk.OBFormat.RGB:
        image = cv2.cvtColor(data.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    elif fmt == sdk.OBFormat.BGR:
        image = data.reshape(h, w, 3).copy()
    elif fmt == sdk.OBFormat.YUYV:
        image = cv2.cvtColor(data.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUY2)
    else:
        raise RuntimeError(f'未支持的彩色格式：{fmt}')
    if image is None:
        raise RuntimeError('彩色帧解码失败')
    return image


class Gemini(Worker):
    def __init__(self, shared, fps=60, preferred=None):
        super().__init__(shared)
        self.fps = fps
        self.preferred=preferred
        self.identity=None
        self.remembered=False

    def run(self):
        pipeline = None
        started = False
        dev = None
        previous_mode = None
        try:
            import pyorbbecsdk as ob
            context = ob.Context()
            devices = context.query_devices()
            candidates = []
            for i in range(devices.get_count()):
                dev = devices.get_device_by_index(i)
                info = dev.get_device_info()
                if 'gemini 2' in info.get_name().lower() and (not self.preferred or info.get_serial_number()==self.preferred):
                    candidates.append(dev)
            if len(candidates) != 1:
                raise RuntimeError(f'找到 {len(candidates)} 台 Gemini 2；请只接一台')
            dev = candidates[0]
            info = dev.get_device_info()
            previous_mode = dev.get_depth_work_mode()
            target_mode = 'Binned Sparse Default' if self.fps == 60 else 'Unbinned Dense Default'
            if str(previous_mode) != target_mode:
                dev.set_depth_work_mode(target_mode)
            self.shared.event(f'Gemini 深度模式：{dev.get_depth_work_mode()}；目标 {self.fps}fps')
            pipeline = ob.Pipeline(dev)
            config = ob.Config()
            cp = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
            try:
                color = cp.get_video_stream_profile(640, 480, ob.OBFormat.MJPG, self.fps)
            except Exception:
                color = cp.get_video_stream_profile(640, 480, ob.OBFormat.RGB, self.fps)
            depth = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR).get_video_stream_profile(640, 400, ob.OBFormat.Y16, self.fps)
            config.enable_stream(color)
            config.enable_stream(depth)
            config.set_frame_aggregate_output_mode(ob.OBFrameAggregateOutputMode.FULL_FRAME_REQUIRE)
            pipeline.start(config)
            started = True
            align = ob.AlignFilter(align_to_stream=ob.OBStreamType.COLOR_STREAM)
            self.shared.event(f'Gemini 已打开：{info.get_name()} / {info.get_serial_number()}；软件 D2C 对齐')
            last = time.monotonic()
            while not self.stop.is_set():
                frames = pipeline.wait_for_frames(250)
                if not frames:
                    if time.monotonic()-last > 8:
                        raise RuntimeError('连续 8 秒没有 RGB-D 帧')
                    continue
                aligned = align.process(frames)
                if not aligned:
                    continue
                if hasattr(aligned, 'as_frame_set'):
                    aligned = aligned.as_frame_set()
                c, d = aligned.get_color_frame(), aligned.get_depth_frame()
                if not c or not d:
                    continue
                rgb = color_bgr(c, ob)
                raw = np.frombuffer(d.get_data(), np.uint16).reshape(d.get_height(), d.get_width()).copy()
                mm = raw.astype(np.float32)*d.get_depth_scale()
                if rgb.shape[:2] != raw.shape:
                    raise RuntimeError('D2C 输出尺寸不一致，停止叠加以免误导')
                self.identity=info.get_serial_number()
                self.shared.publish('gemini', {'rgb': rgb, 'depth_raw': raw, 'depth_mm': mm},
                                    depth_scale=d.get_depth_scale(), color_device_ms=c.get_timestamp(),
                                    depth_device_ms=d.get_timestamp(), aligned=True)
                last = time.monotonic()
        except Exception as e:
            self.shared.event(f'Gemini 2：{e}')
        finally:
            if started:
                try:
                    pipeline.stop()
                except Exception as e:
                    self.shared.event(f'Gemini 停止取流：{e}')
            if dev is not None and previous_mode is not None:
                try:
                    dev.set_depth_work_mode(previous_mode)
                except Exception as e:
                    self.shared.event(f'Gemini 恢复原深度模式失败：{e}')


def depth_views(rgb, mm, far=2000, alpha=.45):
    valid = (mm > 0) & (mm <= far)
    v = np.clip(mm / far * 255, 0, 255).astype(np.uint8)
    depth = cv2.applyColorMap(v, cv2.COLORMAP_TURBO)
    depth[~valid] = 0
    overlay = rgb.copy()
    overlay[valid] = cv2.addWeighted(rgb, 1-alpha, depth, alpha, 0)[valid]
    return depth, overlay


class Recorder(Worker):
    """Independent stream timestamps; no claim of hardware synchronisation."""
    def __init__(self, shared, folder):
        super().__init__(shared)
        self.folder = Path(folder)
        self.count = 0
        self.error = None

    def run(self):
        try:
            self.folder.mkdir(parents=True, exist_ok=False)
            (self.folder/'manifest.json').write_text(json.dumps({
                'version': 1, 'created_ns': time.time_ns(), 'arm': 'RoArm-M3-Pro',
                'joint_order': KEYS, 'joint_unit': 'native radians; gripper is not inverted',
                'depth': 'uint16 PNG, multiply by depth_scale for millimetres; D2C aligned',
                'timing': 'Host receipt timestamps; cameras and joints are NOT hardware synchronized',
                'sampling': 'Latest distinct frames, up to 10 Hz per source; gaps are possible',
            }, ensure_ascii=False, indent=2), encoding='utf-8')
            seen = {}
            with (self.folder/'timeline.jsonl').open('w', encoding='utf-8') as log:
                while not self.stop.is_set():
                    with self.shared.lock:
                        frames = dict(self.shared.frames)
                        pose, pt, pns = self.shared.pose, self.shared.pose_time, self.shared.pose_host_ns
                    for source, packet in frames.items():
                        stamp = packet['host_ns']
                        if seen.get(source) == stamp or time.monotonic()-packet['monotonic'] > 1:
                            continue
                        seen[source] = stamp
                        entry = {k: v for k, v in packet.items() if k not in ('images', 'monotonic')}
                        entry.update(source=source, files={})
                        for kind, image in packet['images'].items():
                            if kind == 'depth_mm':
                                continue
                            filename = f'{source}_{stamp}_{kind}.'+('png' if kind == 'depth_raw' else 'jpg')
                            ok, encoded = cv2.imencode('.png' if kind == 'depth_raw' else '.jpg', image)
                            if not ok:
                                raise IOError('图像写入失败')
                            (self.folder/filename).write_bytes(encoded.tobytes())
                            entry['files'][kind] = filename
                        log.write(json.dumps(entry)+'\n')
                        self.count += 1
                    if pose and pt != seen.get('arm') and time.monotonic()-pt < .6:
                        seen['arm'] = pt
                        log.write(json.dumps(dict(source='arm', host_ns=pns,
                                                  feedback_monotonic=pt, feedback=pose))+'\n')
                    log.flush()
                    self.stop.wait(.1)
            self.shared.event(f'采集已保存：{self.folder}')
        except Exception as e:
            self.error = str(e)
            self.shared.event(f'采集失败：{e}')
