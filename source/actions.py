"""Native-angle teaching files and cancellable, rate-limited trajectory playback."""
import json
import math
import time
from pathlib import Path
from backend import Worker, KEYS, NAMES, all_command, check_radians, FEEDBACK_TIMEOUT, torque_disabled
from host_control import XYZ_DEGRADED_MM, forward, xy_in_sector


REPLAY_PERIOD = .05
STEPS_PER_RAD = 2048 / math.pi
STRICT_TOLERANCE = .07
LOADED_TOLERANCES = (STRICT_TOLERANCE, math.radians(6), math.radians(6),
                     STRICT_TOLERANCE, STRICT_TOLERANCE, STRICT_TOLERANCE)
KEYFRAME_WINDOW = 64
ENCODER_STEP = math.tau / 4096
JOINT_SIMPLIFY_TOLERANCE = math.radians(.25)
TCP_SIMPLIFY_TOLERANCE_MM = 1.
PITCH_SIMPLIFY_TOLERANCE = math.radians(.5)
DWELL_SECONDS = .3


def load_trajectory(path, sdk=False):
    path = Path(path)
    if path.stat().st_size > 50*1024*1024:
        raise ValueError('轨迹文件超过 50 MB')
    data = json.loads(path.read_text(encoding='utf-8-sig'))
    if isinstance(data, dict) and data.get('format') == 'roarm-studio-native-v1':
        records = data['records']
        sdk = False
    elif isinstance(data, list) and sdk:
        records = [{'time': r['timestamped'], 'radians': list(r['radians'])} for r in data]
    else:
        raise ValueError('不是 Studio 轨迹；官方 SDK 文件请先选择对应导入格式')
    if not 2 <= len(records) <= 100000:
        raise ValueError('轨迹需要 2～100000 个采样点')
    result = []
    first = float(records[0]['time'])
    previous = -1
    for record in records:
        stamp = float(record['time'])-first
        values = list(record['radians'])
        if sdk:
            values[-1] = math.pi-values[-1]
        check_radians(values)
        if not math.isfinite(stamp) or stamp <= previous or stamp > 36000:
            raise ValueError('轨迹时间必须递增，总时长不超过 10 小时')
        result.append(dict(time=stamp, radians=values))
        previous = stamp
    return result


def simplify_trajectory(records):
    """Keep original timestamps and bound interpolation error at recorded samples."""
    if len(records) <= 2:
        return records
    times = [record['time'] for record in records]
    joints = [record['radians'] for record in records]
    tool_poses = [forward(angles) for angles in joints]
    last = len(records) - 1
    anchors = {0, last}
    dwell = 0
    for index in range(1, len(records)):
        gap = times[index] - times[index - 1] > FEEDBACK_TIMEOUT
        if gap:
            anchors.update((index - 1, index))
        moved = any(abs(a - b) > ENCODER_STEP for a, b in zip(joints[index], joints[dwell]))
        if gap or moved:
            if times[index - 1] - times[dwell] >= DWELL_SECONDS:
                anchors.update((dwell, index - 1))
            dwell = index
    if times[last] - times[dwell] >= DWELL_SECONDS:
        anchors.add(dwell)

    # Process bounded windows so even a long recording with many turns is cheap to save.
    fixed = sorted(anchors.union(range(0, last, KEYFRAME_WINDOW)))
    kept = set(fixed)
    spans = list(zip(fixed, fixed[1:]))
    while spans:
        left, right = spans.pop()
        start, end = joints[left], joints[right]
        duration = times[right] - times[left]
        worst_error, split = 1., None
        for index in range(left + 1, right):
            ratio = (times[index] - times[left]) / duration
            estimate = [a + (b - a) * ratio for a, b in zip(start, end)]
            errors = [abs(a - b) for a, b in zip(estimate, joints[index])]
            tool = forward(estimate)
            error = max(max(errors[:5]) / JOINT_SIMPLIFY_TOLERANCE,
                        errors[5] / ENCODER_STEP,
                        math.dist(tool[:3], tool_poses[index][:3]) / TCP_SIMPLIFY_TOLERANCE_MM,
                        abs(tool[3] - tool_poses[index][3]) / PITCH_SIMPLIFY_TOLERANCE)
            if not xy_in_sector(*tool[:2]):
                error = math.inf
            if error > worst_error:
                worst_error, split = error, index
        if split is not None:
            kept.add(split)
            spans.extend(((left, split), (split, right)))
    return [records[index] for index in sorted(kept)]


def save_trajectory(path, records, sample_count=None):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix('.tmp')
    data = dict(format='roarm-studio-native-v1', records=records)
    if sample_count is not None:
        data['sample_count'] = sample_count
    temp.write_text(json.dumps(data, ensure_ascii=False), encoding='utf-8')
    temp.replace(path)


TEACH_SLOTS = tuple(range(1, 13))


class TeachLibrary:
    def __init__(self, directory):
        self.directory = Path(directory)

    def path(self, slot):
        if type(slot) is not int or slot not in TEACH_SLOTS:
            raise ValueError('示教槽位必须为 1–12')
        return self.directory / f'teach-{slot:02d}.json'

    def load(self, slot):
        return load_trajectory(self.path(slot))

    def slots(self):
        return [{'slot': slot, 'name': f'示教{slot}', 'occupied': self.path(slot).is_file()}
                for slot in TEACH_SLOTS]


class TeachRecorder(Worker):
    def __init__(self, shared, path):
        super().__init__(shared)
        self.path = Path(path)
        self.count = 0
        self.error = None

    def run(self):
        records, last = [], 0
        started = time.monotonic()
        try:
            while not self.stop.is_set():
                with self.shared.lock:
                    pose, pt = self.shared.pose, self.shared.pose_time
                if pose and pt != last and time.monotonic()-pt < .6:
                    values = [pose[k] for k in KEYS]
                    check_radians(values)
                    records.append(dict(time=pt-started, radians=values))
                    last = pt
                    self.count = len(records)
                if len(records) >= 100000:
                    break
                self.stop.wait(.1)
            if len(records) < 2:
                raise ValueError('有效关节反馈不足两个采样点，未保存可回放轨迹')
            offset = records[0]['time']
            for row in records:
                row['time'] -= offset
            keyframes = simplify_trajectory(records)
            save_trajectory(self.path, keyframes, len(records))
            self.shared.event(f'示教轨迹已保存：{self.path}，{len(records)} 个采样 → {len(keyframes)} 个关键点')
        except Exception as e:
            self.error = str(e)
            self.shared.event(f'示教录制：{e}')


class Playback(Worker):
    def __init__(self, shared, arm, records, factor=.5, speed=150, acc=5, *, external_token=None):
        super().__init__(shared)
        self.arm, self.records = arm, records
        self.factor, self.speed, self.acc = factor, speed, acc
        self.external = external_token is not None
        self.token = external_token if self.external else arm.begin_replay()
        self.progress = 0.
        self.loaded_tolerance = False
        self.xyz_error_mm = 0.
        self.error = None
        self.cancelled = False
        self.serial_written = False
        self.receipt = None
        self._target_angles = None

    def ensure(self):
        if self.stop.is_set() or self.arm.stop.is_set() or self.arm.epoch != self.token:
            raise InterruptedError('回放已停止')
        if self.external and time.monotonic() >= (self.arm.external_deadline or 0):
            raise InterruptedError('外部控制权已到期')
        if not self.arm.connected or time.monotonic()-self.shared.pose_time > FEEDBACK_TIMEOUT:
            raise RuntimeError('超过 0.6 秒未收到反馈，回放已终止；恢复连接后不会自动续播')
        if self.arm.teaching or self.arm.torque_pending or self.arm.torque_fault:
            raise RuntimeError('扭矩状态未确认，回放已终止')
        if torque_disabled(self.shared.pose):
            raise RuntimeError('检测到关节扭矩关闭，请先恢复保持再回放')

    def send(self, values):
        self.ensure()
        with self.arm.command_lock:
            receipt = self.arm.submit(all_command(values, self.speed, self.acc), token=self.token)
            self.receipt = receipt
        deadline = time.monotonic()+1
        # Feedback can arrive before a queued command is written. Do not start
        # arrival checks (or complete and clear the queue) for an unsent target.
        while not receipt.done.wait(.01):
            self.ensure()
            if time.monotonic() > deadline:
                raise RuntimeError('回放指令未确认完整写出串口，已取消后续动作')
        if receipt.error:
            self.ensure()
            raise RuntimeError(receipt.error)
        self.serial_written = True
        self.ensure()
        self._target_angles = tuple(values)

    @property
    def target_angles(self):
        if self.is_alive() and not self.stop.is_set() and self.arm.epoch == self.token:
            return self._target_angles

    @property
    def any_serial_write(self):
        return self.serial_written or (self.receipt is not None and self.receipt.written_at is not None)

    @property
    def degraded(self):
        return self.xyz_error_mm > XYZ_DEGRADED_MM

    def wait_target(self, target, timeout):
        end = time.monotonic()+timeout
        anchor=None
        stamp_before=self.shared.pose_time
        target_xyz = forward(target)[:3]
        while True:
            self.ensure()
            with self.shared.lock:
                stamp=self.shared.pose_time
                pose=self.shared.pose
            if pose and stamp>stamp_before:
                q=[pose[k] for k in KEYS]
                errors=[abs(v-t) for v,t in zip(q,target)]
                xyz_error = math.dist(forward(q)[:3], target_xyz)
                if max(errors) < STRICT_TOLERANCE:
                    self.xyz_error_mm = xyz_error
                    return
                # Only the loaded shoulder/elbow may settle slightly outside 4 degrees.
                if all(e <= limit for e, limit in zip(errors, LOADED_TOLERANCES)):
                    if anchor is None or max(abs(v-w) for v,w in zip(q,anchor[1]))>math.radians(.35):
                        anchor=(stamp,q)
                    elif stamp-anchor[0]>=.45:
                        self.xyz_error_mm = xyz_error
                        if not self.loaded_tolerance:
                            self.loaded_tolerance = True
                            self.shared.event('带载关节已停稳，按实测容差继续回放')
                        return
                else:
                    anchor=None
            if time.monotonic() > end:
                diffs=[abs(pose[k]-v) for k,v in zip(KEYS,target)]
                i=diffs.index(max(diffs))
                raise RuntimeError(f'{NAMES[i]}未到位：目标 {math.degrees(target[i]):.1f}°，实测 {math.degrees(pose[KEYS[i]]):.1f}°，回放已停止')
            self.stop.wait(.05)

    def wait_until(self, deadline):
        while True:
            self.ensure()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            self.stop.wait(min(remaining, REPLAY_PERIOD))

    def wait_if_lagging(self, target):
        with self.shared.lock:
            pose = self.shared.pose
        limits = LOADED_TOLERANCES if self.loaded_tolerance else (STRICT_TOLERANCE,) * len(KEYS)
        if any(abs(pose[k] - value) > limit for k, value, limit in zip(KEYS, target, limits)):
            self.wait_target(target, 5)

    def run(self):
        try:
            self.ensure()
            first = self.records[0]['radians']
            delta = max(abs(self.shared.pose[k]-v) for k, v in zip(KEYS, first))
            self.shared.event('回放：先移动到首帧位置')
            self.send(first)
            self.wait_target(first, delta * STEPS_PER_RAD / self.speed + 15)
            last_target = first
            last_update = time.monotonic()
            total_time = self.records[-1]['time']
            for i in range(1, len(self.records)):
                a, b = self.records[i-1], self.records[i]
                duration = max((b['time']-a['time'])/self.factor,
                               max(abs(x-y) for x, y in zip(a['radians'], b['radians'])) * STEPS_PER_RAD / self.speed,
                               REPLAY_PERIOD)
                samples = max(1, int(duration / REPLAY_PERIOD))
                interval = duration / samples
                for sample in range(1, samples + 1):
                    self.wait_until(last_update + interval)
                    self.wait_if_lagging(last_target)
                    # Resume from the current time after a tracking pause, without a burst of overdue targets.
                    last_update = time.monotonic()
                    ratio = sample / samples
                    last_target = (b['radians'] if sample == samples else
                                   [x + (y - x) * ratio for x, y in zip(a['radians'], b['radians'])])
                    self.send(last_target)
                    if i < len(self.records) - 1 or sample < samples:
                        self.progress = (a['time'] + (b['time'] - a['time']) * ratio) / total_time
            if len(self.records) > 1:
                self.wait_target(last_target, 5)
            self.progress = 1.
            self.shared.event('示教回放完成')
        except InterruptedError:
            self.cancelled = True
            self.shared.event('示教回放已停止')
        except Exception as e:
            self.error = str(e)
            self.shared.event(f'回放：{e}')
        finally:
            with self.arm.command_lock:
                if self.arm.epoch == self.token and (not self.external or self.error or self.cancelled):
                    self.arm.cancel_replay()
                    # Keep the last motor target, including its load-holding bias.
                    # Encoder angles are observations, not replacement targets.
                    if self.error:
                        self.shared.event('已取消回放待发动作；已发送的目标可能继续执行，未改写保持目标。')
