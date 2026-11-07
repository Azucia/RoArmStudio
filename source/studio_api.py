"""Authenticated, loopback-only device API. Device I/O stays in Studio."""
import hmac
import json
import math
import secrets
import threading
import time
import uuid
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import cv2

from backend import FEEDBACK_TIMEOUT, KEYS, all_command, depth_views, joint_command
from backend import torque_disabled
from host_control import base_in_sector, forward
from host_direct import holding_offset, plan_xyz


LEASE_SECONDS = 15
ACTION_SECONDS = 30
SOURCES = {'hand': 'uvc', 'workspace': 'gemini'}


class APIError(Exception):
    def __init__(self, status, code, message):
        super().__init__(message)
        self.status, self.code = status, code


class DeviceAPI:
    def __init__(self, shared, get_arm):
        self.shared, self.get_arm = shared, get_arm
        self.lock = threading.RLock()
        self.image_lock = threading.Lock()
        self.images = {}
        self.profile = (150, 5, 0., False, 2000, .45)
        self.server = None
        self.secret = ''
        self.lease = None
        self.actions = {}
        self.active = None
        self.intent = None
        self.execution_detail = ''

    def start(self, port):
        server = LocalServer(('127.0.0.1', port), self)
        self.secret = secrets.token_urlsafe(32)
        self.server = server
        threading.Thread(target=server.serve_forever, kwargs={'poll_interval': .05},
                         daemon=True).start()
        return f'http://127.0.0.1:{server.server_port}'

    def stop(self):
        with self.lock:
            server, self.server = self.server, None
            self.secret = ''
            self.release()
            self.actions.clear()
        if server is not None:
            server.shutdown()
            server.server_close()

    def telemetry(self):
        with self.shared.lock:
            pose = self.shared.pose
            stamp = self.shared.pose_time
            host_ns = self.shared.pose_host_ns
        return pose, stamp, host_ns

    def release(self, state='cancelled', reason='控制权已释放'):
        if self.lease is not None:
            _, arm, epoch = self.lease
            with arm.command_lock:
                if arm.epoch == epoch:
                    arm.cancel_replay()
            self.lease = None
        if self.active is not None:
            self.finish(state, reason)
        self.intent = None
        self.execution_detail = ''

    def finish(self, state, message=None):
        result, receipt, _, _, _ = self.active
        result.update(state=state, serial_written=receipt.written_at is not None, message=message)
        self.active = None

    def poll(self):
        with self.lock:
            if self.lease is None:
                return
            _, arm, epoch = self.lease
            pose, stamp, _ = self.telemetry()
            now = time.monotonic()
            if (arm is not self.get_arm() or arm.epoch != epoch or not arm.connected
                    or arm.stop.is_set() or now >= (arm.external_deadline or 0)
                    or pose is None or now - stamp > FEEDBACK_TIMEOUT or torque_disabled(pose)
                    or not base_in_sector(pose.get('b'), feedback=True)):
                self.release('cancelled', '控制权、连接或机械臂反馈已失效')
                return
            if self.active is None:
                return
            result, receipt, joint_targets, xyz, deadline = self.active
            if receipt.done.is_set():
                if receipt.error:
                    self.finish('failed', receipt.error)
                    self.intent = None
                    return
                result.update(state='sent', serial_written=True)
                actual = self.xyz_feedback(pose)
                if (stamp > receipt.written_at
                        and all(abs(pose[key] - target) < .07 for key, target in joint_targets.items())
                        and (xyz is None or (actual is not None and math.dist(actual, xyz) <= 3))):
                    self.finish('degraded_reached' if result['degraded'] else 'reached')
                    return
            if now >= deadline:
                self.release('timed_out', '动作超时，已取消待发指令')

    @staticmethod
    def xyz_feedback(pose):
        if pose is None:
            return None
        values = [pose.get(key) for key in ('x', 'y', 'z')]
        return values if all(type(v) in (int, float) and math.isfinite(v) for v in values) else None

    def status(self):
        arm = self.get_arm()
        pose, stamp, host_ns = self.telemetry()
        connected = bool(arm and arm.connected and not arm.stop.is_set())
        age = max(0., time.monotonic() - stamp) if connected and stamp else None
        fresh = age is not None and age <= FEEDBACK_TIMEOUT
        with self.shared.lock:
            cameras = {name: self.shared.frames.get(source) for name, source in SOURCES.items()}
        return {
            'api_version': 1, 'connected': connected, 'feedback_age_ms': None if age is None else round(age * 1000),
            'feedback_host_ns': host_ns if fresh else None,
            'xyz_mm': self.xyz_feedback(pose) if fresh else None,
            'joint_deg': [math.degrees(pose[k]) for k in KEYS] if fresh and pose else None,
            'torque_enabled': (all(pose.get('torswitch'+key) == 1 for key in ('B','S','E','T','R','G'))
                               if fresh and pose else None),
            'external_control': self.lease is not None,
            'cameras': {name: {'online': bool(packet and time.monotonic() - packet['monotonic'] < 1)}
                        for name, packet in cameras.items()},
        }

    def acquire(self):
        if self.lease is not None:
            raise APIError(409, 'control_busy', '已有外部控制方')
        arm = self.get_arm()
        if not arm or not arm.connected or arm.stop.is_set():
            raise APIError(409, 'arm_offline', '机械臂未连接')
        with arm.command_lock:
            pose, stamp, _ = self.telemetry()
            if (not pose or time.monotonic() - stamp > FEEDBACK_TIMEOUT
                    or not base_in_sector(pose.get('b'), feedback=True)
                    or any(pose.get('torswitch'+key) != 1 for key in ('B','S','E','T','R','G'))):
                raise APIError(409, 'arm_not_ready', '需要新鲜反馈及确认扭矩保持')
            target = arm.last_joint_target
            if not arm.commands.empty() or (target is not None and
                    any(abs(pose[k] - v) >= .07 for k, v in zip(KEYS, target))):
                raise APIError(409, 'arm_busy', '请等待当前动作结束')
            try:
                epoch = arm.begin_replay()
            except ValueError as error:
                raise APIError(409, 'arm_busy', str(error)) from error
            arm.external_deadline = time.monotonic() + LEASE_SECONDS
            key = secrets.token_urlsafe(24)
            self.lease = key, arm, epoch
        self.actions.clear()
        return {'lease': key, 'ttl_s': LEASE_SECONDS}

    def require_lease(self, key):
        if self.lease is None or not same_secret(key, self.lease[0]):
            raise APIError(409, 'invalid_lease', '请先取得控制权')
        _, arm, epoch = self.lease
        with arm.command_lock:
            if arm.epoch != epoch or time.monotonic() >= (arm.external_deadline or 0):
                raise APIError(409, 'invalid_lease', '控制权已失效')
            arm.external_deadline = time.monotonic() + LEASE_SECONDS
        return arm, epoch

    def act(self, body, key):
        arm, epoch = self.require_lease(key)
        try:
            action_id = str(uuid.UUID(body['request_id']))
        except (AttributeError, KeyError, TypeError, ValueError) as error:
            raise APIError(400, 'invalid_request', 'request_id 必须为 UUID') from error
        if action_id in self.actions:
            old_body, result = self.actions[action_id]
            if old_body != body:
                raise APIError(409, 'request_conflict', '同一编号已用于另一动作')
            return result.copy()
        if len(self.actions) >= 4096:
            raise APIError(409, 'lease_full', '请释放并重新取得控制权')
        if self.active is not None:
            raise APIError(409, 'action_busy', '等待当前动作结束或取消')

        with arm.command_lock:
            pose, stamp, _ = self.telemetry()
            if (arm.epoch != epoch or not pose or time.monotonic() - stamp > FEEDBACK_TIMEOUT
                    or any(pose.get('torswitch'+k) != 1 for k in ('B','S','E','T','R','G'))):
                raise APIError(409, 'arm_not_ready', '机械臂反馈或扭矩已失效')
            return self.submit_action(body, action_id, arm, epoch, pose)

    def submit_action(self, body, action_id, arm, epoch, pose):
        kind = body.get('kind')
        speed, acc, bias, home_down, _, _ = self.profile
        measured = [pose[k] for k in KEYS]
        accepted, detail, planned, intent = None, '', None, None
        try:
            if kind in ('move_xyz', 'jog_xyz'):
                xyz = body.get('xyz_mm')
                if (not isinstance(xyz, list) or len(xyz) != 3 or
                        any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 600 for v in xyz)):
                    raise ValueError('xyz_mm 需要三个有效毫米数值，范围 ±600')
                degraded = body.get('allow_degraded', False)
                if type(degraded) is not bool:
                    raise ValueError('allow_degraded 需要布尔值')
                actual = self.xyz_feedback(pose)
                if actual is None or not isinstance(pose.get('tit'), (int, float)):
                    raise ValueError('缺少末端坐标或俯仰反馈')
                values = actual + [math.degrees(pose[k]) for k in ('tit', 'r', 'g')]
                offset = [0.] * 6
                if kind == 'jog_xyz':
                    if self.intent is None:
                        offset = holding_offset(values, arm.last_joint_target)
                    else:
                        previous, offset = self.intent
                        values = previous.copy()
                    values[:3] = [a + b for a, b in zip(values[:3], xyz)]
                else:
                    values[:3] = xyz
                cmd, joints, accepted, detail = plan_xyz(values, measured, speed, acc, bias,
                    offset, arm.last_joint_target, allow_degraded=degraded)
                cmd['_host_intent'] = True
                planned = forward(joints)
                targets = dict(zip(KEYS, joints))
                intent = accepted, offset
            elif kind == 'home':
                direction = body.get('direction', 'down' if home_down else 'forward')
                if direction not in ('forward', 'down'):
                    raise ValueError('direction 必须为 forward 或 down')
                joints = [0, 0, math.pi/2, math.pi/2 if direction == 'down' else 0, 0, math.pi]
                cmd = all_command(joints, 100, 5)
                targets = dict(zip(KEYS, joints))
            elif kind == 'grip':
                closed = body.get('closed')
                if type(closed) is not bool:
                    raise ValueError('closed 需要布尔值')
                cmd = joint_command(5, 180 if closed else math.degrees(1.08), 100)
                targets = {'g': cmd['rad']}
            else:
                raise ValueError('未知动作')
            receipt = arm.submit(cmd, token=epoch)
        except (OverflowError, ValueError) as error:
            raise APIError(422, 'motion_rejected', str(error)) from error
        result = {'id': action_id, 'state': 'queued', 'degraded': bool(detail),
                  'serial_written': False, 'accepted_xyz_mm': accepted[:3] if accepted else None,
                  'planned_xyz_mm': planned[:3] if planned else None,
                  'planned_pitch_deg': math.degrees(planned[3]) if planned else None,
                  'message': detail or None}
        self.intent = intent
        self.execution_detail = detail
        self.actions[action_id] = body, result
        self.active = result, receipt, targets, result['accepted_xyz_mm'], time.monotonic() + ACTION_SECONDS
        self.shared.event(f'对外连接：{kind} 已提交' + (f' · 降标执行 · {detail}' if detail else ''))
        return result.copy()

    def command(self, method, path, body, key, server):
        with self.lock:
            if server is not self.server:
                raise APIError(503, 'service_stopped', '本机接口已停止')
            self.poll()
            if method == 'GET' and path == '/v1/status':
                return self.status()
            if method == 'POST' and path == '/v1/lease':
                return self.acquire()
            if method == 'POST' and path == '/v1/lease/renew':
                self.require_lease(key)
                return {'ttl_s': LEASE_SECONDS}
            if method == 'POST' and path in ('/v1/lease/release', '/v1/cancel'):
                self.require_lease(key)
                self.release()
                return {'cancelled_pending': True}
            if method == 'POST' and path == '/v1/actions':
                return self.act(body, key)
            if method == 'GET' and path.startswith('/v1/actions/'):
                record = self.actions.get(path.rsplit('/', 1)[1])
                if record is not None:
                    return record[1].copy()
            raise APIError(404, 'not_found', '接口或动作不存在')

    def frame(self, name, depth=False):
        with self.shared.lock:
            packet = self.shared.frames.get(SOURCES[name])
        if not packet or time.monotonic() - packet['monotonic'] >= 1:
            raise APIError(503, 'camera_offline', '没有新鲜画面')
        stamp = packet['host_ns']
        _, _, _, _, far, alpha = self.profile
        key = (name, depth)
        version = (stamp, far, alpha)
        with self.image_lock:
            cached = self.images.get(key)
            if cached and cached[0] == version:
                return cached[1]
            images = packet['images']
            if depth:
                if 'depth_raw' not in images:
                    raise APIError(503, 'depth_unavailable', '当前帧缺少深度')
                image = images['depth_raw']
            elif name == 'workspace':
                _, image = depth_views(images['rgb'], images['depth_mm'], far, alpha)
            else:
                image = images['rgb']
            fmt, mime = ('.png', 'image/png') if depth else ('.jpg', 'image/jpeg')
            ok, data = cv2.imencode(fmt, image)
            if not ok:
                raise APIError(503, 'encode_failed', '图像编码失败')
            headers = {'X-Frame-Host-Ns': str(stamp)}
            if depth:
                headers['X-Depth-Scale-Mm'] = str(packet['depth_scale'])
            result = data.tobytes(), mime, headers
            self.images[key] = version, result
            return result


def same_secret(value, expected):
    return hmac.compare_digest(value.encode(), expected.encode())


class LocalServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = False

    def __init__(self, address, api):
        self.api = api
        self.slots = threading.BoundedSemaphore(12)
        self.stream_slots = threading.BoundedSemaphore(6)
        super().__init__(address, Handler)

    def process_request(self, request, address):
        if not self.slots.acquire(blocking=False):
            self.shutdown_request(request)
            return
        try:
            super().process_request(request, address)
        except BaseException:
            self.slots.release()
            raise

    def process_request_thread(self, request, address):
        try:
            super().process_request_thread(request, address)
        finally:
            self.slots.release()


class Handler(BaseHTTPRequestHandler):
    def setup(self):
        self.request.settimeout(2)
        super().setup()

    def log_message(self, *args):
        pass

    def reply(self, status, data, mime='application/json', headers=None):
        self.send_response(status)
        self.send_header('Content-Type', mime)
        self.send_header('Content-Length', str(len(data)))
        self.send_header('Cache-Control', 'no-store')
        for key, value in (headers or {}).items():
            self.send_header(key, value)
        self.end_headers()
        self.wfile.write(data)

    def json_reply(self, status, value):
        self.reply(status, json.dumps(value, ensure_ascii=False, allow_nan=False).encode())

    def do_GET(self):
        self.handle_api()

    def do_POST(self):
        self.handle_api()

    def handle_api(self):
        try:
            api = self.server.api
            secret = api.secret
            if (not secret or api.server is not self.server or
                    self.headers.get('Host') != f'127.0.0.1:{self.server.server_port}' or
                    not same_secret(self.headers.get('Authorization', ''), f'Bearer {secret}')):
                raise APIError(401, 'unauthorized', '需要本机地址及有效连接密钥')
            body = {}
            if self.command == 'POST':
                if self.headers.get_content_type() != 'application/json' or self.headers.get('Transfer-Encoding'):
                    raise APIError(415, 'invalid_content_type', '需要 JSON 请求')
                try:
                    length = int(self.headers.get('Content-Length', '0'))
                    if not 0 < length <= 8192:
                        raise ValueError()
                    body = json.loads(self.rfile.read(length))
                    if not isinstance(body, dict):
                        raise ValueError()
                except (UnicodeError, ValueError, RecursionError) as error:
                    raise APIError(400, 'invalid_json', 'JSON 请求无效') from error
            parts = self.path.split('/')
            if self.command == 'GET' and len(parts) == 5 and parts[1:3] == ['v1', 'cameras'] and parts[3] in SOURCES:
                name, resource = parts[3:]
                if resource == 'stream.mjpg':
                    self.stream(api, name)
                    return
                if resource == 'frame.jpg' or (name == 'workspace' and resource == 'depth.png'):
                    self.reply(200, *api.frame(name, resource == 'depth.png'))
                    return
            result = api.command(self.command, self.path, body, self.headers.get('X-Control-Lease', ''), self.server)
            self.json_reply(200, result)
        except APIError as error:
            try:
                self.json_reply(error.status, {'error': error.code, 'message': str(error)})
            except OSError:
                pass
        except OSError:
            pass

    def stream(self, api, name):
        if not self.server.stream_slots.acquire(blocking=False):
            raise APIError(503, 'stream_busy', '画面订阅已满')
        try:
            api.frame(name)
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            last = None
            while api.server is self.server:
                try:
                    data, mime, headers = api.frame(name)
                except APIError:
                    break
                stamp = headers['X-Frame-Host-Ns']
                if stamp != last:
                    prefix = f'--frame\r\nContent-Type: {mime}\r\nContent-Length: {len(data)}\r\nX-Frame-Host-Ns: {stamp}\r\n\r\n'
                    self.wfile.write(prefix.encode() + data + b'\r\n')
                    last = stamp
                time.sleep(.05)
        finally:
            self.server.stream_slots.release()
