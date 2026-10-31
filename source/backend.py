"""RoArm M3 protocol and camera workers. No motion on connection."""
import json
import math
import queue
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import serial

KEYS = ('b', 's', 'e', 't', 'r', 'g')
NAMES = ('底座', '肩部', '肘部', '腕部俯仰', '腕部旋转', '夹爪')
# Conservative operating envelope, in firmware/native degrees (no SDK gripper inversion).
LIMITS = ((-180, 180), (-90, 90), (0, 180), (-90, 90), (-180, 180), (math.degrees(1.08), 180))
MOTION_TYPES = (101, 102, 1041)


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


def pose_command(values):
    ranges = ((-500, 500), (-500, 500), (-600, 600), (-90, 90), (-180, 180), LIMITS[-1])
    if len(values) != 6 or any(not math.isfinite(v) or not low <= v <= high
                               for v, (low, high) in zip(values, ranges)):
        raise ValueError('末端目标超出软件范围')
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

    def publish(self, key, images, **meta):
        with self.lock:
            self.frames[key] = dict(images=images, host_ns=time.time_ns(),
                                    monotonic=time.monotonic(), **meta)

    def event(self, message):
        self.events.put(message)


class Worker(threading.Thread):
    def __init__(self, shared):
        super().__init__(daemon=True)
        self.shared = shared
        self.stop = threading.Event()


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

    def submit(self, command, token=None):
        with self.command_lock:
            return self._submit(command, token)

    def cancel_replay(self):
        with self.command_lock:
            self.epoch += 1
            self.playing = False
            while not self.commands.empty():
                try:
                    self.commands.get_nowait()
                except queue.Empty:
                    break

    def begin_replay(self):
        with self.command_lock:
            self.cancel_replay()
            self.playing = True
            return self.epoch

    def _submit(self, command, token=None):
        if not self.connected:
            raise ValueError('机械臂尚未连接')
        if token is not None and (token != self.epoch or not self.playing):
            raise ValueError('回放已取消')
        if self.teaching and command.get('T') in MOTION_TYPES:
            raise ValueError('请先结束手动示教')
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
                except queue.Empty:
                    break
            for pending in settings:
                self.commands.put_nowait(pending)
        command = dict(command)
        if command.get('T') in MOTION_TYPES:
            command['_epoch'] = self.epoch
        try:
            self.commands.put_nowait(command)
        except queue.Full:
            raise ValueError('待发送设置较多，请稍后重试')

    def run(self):
        ser = None
        try:
            ser = serial.Serial()
            ser.port, ser.baudrate, ser.timeout, ser.write_timeout = self.port, 115200, .03, .3
            ser.dtr = ser.rts = False
            ser.open()
            self.connected = True
            self.shared.event(f'机械臂串口已打开：{self.port}；等待实际反馈')
            last_query = 0
            buf = bytearray()
            while not self.stop.is_set():
                try:
                    cmd = self.commands.get_nowait()
                    with self.command_lock:
                        epoch = cmd.pop('_epoch', self.epoch)
                        if epoch != self.epoch:
                            self.shared.event('已丢弃取消前的运动指令')
                        elif cmd.get('T') in MOTION_TYPES and time.monotonic()-self.shared.pose_time > .6:
                            self.shared.event('反馈已过期，已丢弃运动指令')
                        else:
                            ser.write((json.dumps(cmd)+'\n').encode())
                            self.shared.event('发送 '+json.dumps(public_command(cmd), ensure_ascii=False))
                except queue.Empty:
                    pass
                if time.monotonic()-last_query >= .1:
                    ser.write(b'{"T":105}\n')
                    last_query = time.monotonic()
                buf.extend(ser.read(max(1, min(ser.in_waiting, 8192))))
                while b'\n' in buf:
                    line, _, buf = buf.partition(b'\n')
                    try:
                        start = line.find(b'{')
                        data = json.loads(line[start:]) if start >= 0 else {}
                        if isinstance(data, dict) and valid_pose(data):
                            with self.shared.lock:
                                self.shared.pose = data
                                self.shared.pose_time = time.monotonic()
                                self.shared.pose_host_ns = time.time_ns()
                    except (ValueError, UnicodeError):
                        pass
                if len(buf) > 65536:
                    buf.clear()
        except Exception as e:
            self.shared.event(f'机械臂连接错误：{e}')
        finally:
            self.connected = False
            if ser:
                ser.close()
            self.shared.event('机械臂串口已关闭（未发送释放扭矩指令）')


class UVC(Worker):
    def __init__(self, shared, index):
        super().__init__(shared)
        self.index = index

    def run(self):
        cap = None
        try:
            cap = cv2.VideoCapture(self.index, cv2.CAP_DSHOW)
            if not cap.isOpened():
                raise RuntimeError('无法打开所选 UVC 摄像头')
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, 30)
            self.shared.event(f'AR0234 / UVC 已打开，索引 {self.index}')
            failures = 0
            while not self.stop.is_set():
                ok, image = cap.read()
                if not ok:
                    failures += 1
                    if failures > 30:
                        raise RuntimeError('摄像头持续无帧，请检查连接或占用')
                    self.stop.wait(.05)
                    continue
                failures = 0
                self.shared.publish('uvc', {'rgb': image})
        except Exception as e:
            self.shared.event(f'AR0234：{e}')
        finally:
            if cap:
                cap.release()


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
    def run(self):
        pipeline = None
        started = False
        try:
            import pyorbbecsdk as ob
            context = ob.Context()
            devices = context.query_devices()
            candidates = []
            for i in range(devices.get_count()):
                dev = devices.get_device_by_index(i)
                info = dev.get_device_info()
                if 'gemini 2' in info.get_name().lower():
                    candidates.append(dev)
            if len(candidates) != 1:
                raise RuntimeError(f'找到 {len(candidates)} 台 Gemini 2；请只接一台')
            dev = candidates[0]
            info = dev.get_device_info()
            pipeline = ob.Pipeline(dev)
            config = ob.Config()
            cp = pipeline.get_stream_profile_list(ob.OBSensorType.COLOR_SENSOR)
            try:
                color = cp.get_video_stream_profile(640, 480, ob.OBFormat.RGB, 30)
            except Exception:
                color = cp.get_default_video_stream_profile()
            depth = pipeline.get_stream_profile_list(ob.OBSensorType.DEPTH_SENSOR).get_default_video_stream_profile()
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
                self.shared.publish('gemini', {'rgb': rgb, 'depth_raw': raw, 'depth_mm': mm},
                                    depth_scale=d.get_depth_scale(), color_device_ms=c.get_timestamp(),
                                    depth_device_ms=d.get_timestamp(), aligned=True)
                last = time.monotonic()
        except Exception as e:
            self.shared.event(f'Gemini 2：{e}')
        finally:
            if started:
                pipeline.stop()


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
