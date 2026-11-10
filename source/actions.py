"""Native-angle teaching files and cancellable, rate-limited trajectory playback."""
import json
import logging
import math
import time
from pathlib import Path
from backend import Worker, KEYS, LIMITS, NAMES, all_command, check_radians, FEEDBACK_TIMEOUT, torque_disabled
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
TRACKING_CONFIRM_SECONDS = .25
TRACKING_SLOW_RATE = .5
TRACKING_RECOVERY_SECONDS = 1.
BIAS_STABLE_SECONDS = .45
BIAS_STABLE_RAD = math.radians(.25)
BIAS_MAX_RAD = math.radians(1)
BIAS_RANGE_RAD = math.radians(15)
WRIST_INDEX = KEYS.index('t')
WRIST_TEACH_MARGIN_DEG = 3.
GRIP_PRELOAD_RAD = math.radians(10)
ARM_KEYS = KEYS[:-1]
ARM_TOLERANCES = LOADED_TOLERANCES[:-1]
DWELL_CORRECTION_LIMIT = math.radians(2.5)
DWELL_CORRECTION_MAX_ERROR = math.radians(3.)
DWELL_CORRECTION_MAX_TCP_MM = 25.
DWELL_CORRECTION_ATTEMPTS = 2


def load_trajectory(path, sdk=False, *, include_metadata=False):
    path = Path(path)
    if path.stat().st_size > 50*1024*1024:
        raise ValueError('轨迹文件超过 50 MB')
    data = json.loads(path.read_text(encoding='utf-8-sig'))
    metadata = {}
    if isinstance(data, dict) and data.get('format') == 'roarm-studio-native-v1':
        records = data['records']
        clip = data.get('wrist_clip_deg', 0.)
        if type(clip) in (float, int) and 0 < clip <= WRIST_TEACH_MARGIN_DEG:
            metadata['wrist_clip_deg'] = clip
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
    return (result, metadata) if include_metadata else result


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


def dwell_starts(records):
    """Find holds in the recorded timebase, including simplified long holds."""
    start = 0
    for index in range(1, len(records) + 1):
        if index < len(records):
            moved = any(abs(a - b) > ENCODER_STEP for a, b in
                        zip(records[index]['radians'], records[start]['radians']))
            if not moved:
                continue
        if records[index - 1]['time'] - records[start]['time'] >= DWELL_SECONDS:
            yield start
        start = index


def save_trajectory(path, records, sample_count=None, *, wrist_clip_deg=0.):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temp = path.with_suffix('.tmp')
    data = dict(format='roarm-studio-native-v1', records=records)
    if sample_count is not None:
        data['sample_count'] = sample_count
    if wrist_clip_deg:
        data['wrist_clip_deg'] = wrist_clip_deg
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

    def load(self, slot, *, include_metadata=False):
        return load_trajectory(self.path(slot), include_metadata=include_metadata)

    def slots(self):
        return [{'slot': slot, 'name': f'示教{slot}', 'occupied': self.path(slot).is_file()}
                for slot in TEACH_SLOTS]


class TeachRecorder(Worker):
    def __init__(self, shared, path):
        super().__init__(shared)
        self.path = Path(path)
        self.count = 0
        self.error = None
        self.wrist_clip_deg = 0.

    def run(self):
        records, last = [], 0
        started = time.monotonic()
        try:
            while not self.stop.is_set():
                with self.shared.lock:
                    pose, pt = self.shared.pose, self.shared.pose_time
                if pose and pt != last and time.monotonic()-pt < .6:
                    values = [pose[k] for k in KEYS]
                    wrist = math.degrees(values[WRIST_INDEX])
                    low, high = LIMITS[WRIST_INDEX]
                    if wrist < low or wrist > high:
                        clipped = min(high, max(low, wrist))
                        deviation = abs(wrist - clipped)
                        if deviation > WRIST_TEACH_MARGIN_DEG:
                            raise ValueError(f'腕部俯仰反馈 {wrist:.2f}°，超过示教贴边范围；本次未保存')
                        values[WRIST_INDEX] = math.radians(clipped)
                        if self.wrist_clip_deg == 0.:
                            self.shared.event('示教腕部贴边：这段回放目标将限制在 ±90°，请查看保存提示')
                        self.wrist_clip_deg = max(self.wrist_clip_deg, deviation)
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
            save_trajectory(self.path, keyframes, len(records), wrist_clip_deg=self.wrist_clip_deg)
            self.shared.event(f'示教轨迹已保存：{self.path}，{len(records)} 个采样 → {len(keyframes)} 个关键点')
        except Exception as e:
            self.error = str(e)
            logging.getLogger(__name__).warning('示教录制失败：%s', self.error)
            self.shared.event(f'示教录制：{e}')


class ReplayLoadBias:
    """Learn shoulder/elbow hold error only while target and feedback are steady."""

    def __init__(self):
        self.anchor = None
        self.last_stamp = None
        self.learned_at = None
        self.learned = (0., 0.)
        self.applied = (0., 0.)

    def target(self, nominal, measured, stamp):
        nominal = tuple(nominal)
        if self.learned_at is not None and abs(nominal[5] - self.learned_at[5]) > BIAS_STABLE_RAD:
            self.learned_at = None
        if self.anchor is not None and any(
                abs(a-b) > BIAS_STABLE_RAD for a, b in zip(nominal, self.anchor[0])):
            self.anchor = None
        if stamp != self.last_stamp:
            self.last_stamp = stamp
            if self.anchor is None or any(
                    abs(a-b) > BIAS_STABLE_RAD for a, b in zip(measured, self.anchor[1])):
                self.anchor = (nominal, tuple(measured), stamp)
            elif stamp - self.anchor[2] >= BIAS_STABLE_SECONDS:
                errors = [want - have for want, have in zip(nominal, measured)]
                if all(abs(error) <= limit for error, limit in zip(errors, ARM_TOLERANCES)):
                    self.learned_at = nominal
                    self.learned = tuple(
                        max(-BIAS_MAX_RAD, min(BIAS_MAX_RAD, bias + .5 * errors[joint]))
                        for joint, bias in zip((1, 2), self.applied))
                self.anchor = None
        weight = 0.
        if self.learned_at is not None:
            distance = max(abs(a-b) for a, b in zip(nominal[1:5], self.learned_at[1:5]))
            weight = max(0., 1. - distance / BIAS_RANGE_RAD)
        self.applied = tuple(
            old + max(-BIAS_STABLE_RAD, min(BIAS_STABLE_RAD, weight * bias - old))
            for old, bias in zip(self.applied, self.learned))
        return (nominal[0], nominal[1] + self.applied[0], nominal[2] + self.applied[1], *nominal[3:])


class Playback(Worker):
    def __init__(self, shared, arm, records, factor=.5, speed=150, acc=5, *,
                 external_token=None, load_compensation=False):
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
        self.last_motor_target = None
        self.dwell_correction = None
        self.load_bias = ReplayLoadBias() if load_compensation else None
        self.grip_reference = None
        self.grip_closing = False
        self.reset_tracking()

    def reset_tracking(self):
        self.tracking_stamp = None
        self.tracking_since = [None, None, None]
        self.tracking_rate = 1.

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
        motor_target = values
        if self.load_bias is not None and self.serial_written:
            with self.shared.lock:
                measured = [self.shared.pose[key] for key in KEYS]
                stamp = self.shared.pose_time
            motor_target = self.load_bias.target(values, measured, stamp)
        grip = self.preload_grip(values[-1])
        motor_target = [*motor_target[:-1], grip]
        if self.dwell_correction is not None:
            anchor, offsets = self.dwell_correction
            distance = max(abs(a-b) for a, b in zip(values, anchor))
            weight = max(0., 1. - distance / BIAS_RANGE_RAD)
            if not weight:
                self.dwell_correction = None
            for joint, offset in zip((1, 2), offsets):
                motor_target[joint] += weight * (values[joint] + offset - motor_target[joint])
        self.write_motor_target(values, motor_target, self.speed, self.acc)

    def write_motor_target(self, values, motor_target, speed, acceleration):
        self.ensure()
        command = all_command(motor_target, speed, acceleration)
        if tuple(motor_target) != tuple(values):
            command['_teach_nominal'] = list(values)
        with self.arm.command_lock:
            receipt = self.arm.submit(command, token=self.token)
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
        self.last_motor_target = tuple(motor_target)

    def preload_grip(self, nominal):
        if self.grip_reference is None:
            self.grip_reference = nominal
        elif abs(nominal - self.grip_reference) >= ENCODER_STEP * .5:
            self.grip_closing = nominal > self.grip_reference
            self.grip_reference = nominal
        return min(math.radians(LIMITS[-1][1]), nominal + GRIP_PRELOAD_RAD) if self.grip_closing else nominal

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

    def wait_target(self, target, timeout, *, final=False, label='示教终点', correct_dwell=False):
        end = time.monotonic()+timeout
        anchor=None
        stamp_before=self.shared.pose_time
        target_xyz = forward(target)[:3]
        correction_errors = []
        while True:
            self.ensure()
            with self.shared.lock:
                stamp=self.shared.pose_time
                pose=self.shared.pose
            if pose and stamp>stamp_before:
                q=[pose[k] for k in KEYS]
                errors=[abs(v-t) for v,t in zip(q[:-1],target)]
                xyz_error = math.dist(forward(q)[:3], target_xyz)
                strict = max(errors) < STRICT_TOLERANCE
                if strict and not final:
                    self.xyz_error_mm = xyz_error
                    return
                if final:
                    self.xyz_error_mm = xyz_error
                within_joints = strict or all(e <= limit for e, limit in zip(errors, ARM_TOLERANCES))
                arrived = within_joints and (not final or xyz_error <= XYZ_DEGRADED_MM)
                correctable = (correct_dwell and within_joints and self.last_motor_target is not None
                               and max(errors[1:3]) <= DWELL_CORRECTION_MAX_ERROR
                               and max(errors[0], *errors[3:]) <= math.radians(2.)
                               and XYZ_DEGRADED_MM < xyz_error <= DWELL_CORRECTION_MAX_TCP_MM)
                if arrived or correctable:
                    if anchor is None or max(abs(v-w) for v,w in zip(q[:-1],anchor[1]))>math.radians(.35):
                        anchor=(stamp,q[:-1])
                    elif stamp-anchor[0]>=.45:
                        self.xyz_error_mm = xyz_error
                        if correctable:
                            if correction_errors and xyz_error >= correction_errors[-1] - .25:
                                raise RuntimeError(f'{label}微调未改善：{correction_errors[-1]:.1f} → '
                                                   f'{xyz_error:.1f} mm，已停止后续动作')
                            if len(correction_errors) >= DWELL_CORRECTION_ATTEMPTS:
                                raise RuntimeError(f'{label}微调后仍未到位：{correction_errors[0]:.1f} → '
                                                   f'{xyz_error:.1f} mm（要求 ≤ {XYZ_DEGRADED_MM:g} mm）')
                            correction_errors.append(xyz_error)
                            self.correct_dwell_target(target, q, xyz_error)
                            with self.shared.lock:
                                stamp_before = self.shared.pose_time
                            anchor = None
                            continue
                        if correction_errors:
                            self.shared.event(f'{label}微调到位：{correction_errors[0]:.1f} → {xyz_error:.1f} mm')
                        if not strict and not self.loaded_tolerance:
                            self.loaded_tolerance = True
                            self.shared.event('带载关节已停稳，按实测容差继续回放')
                        return
                else:
                    anchor=None
            if time.monotonic() > end:
                diffs=[abs(pose[k]-v) for k,v in zip(ARM_KEYS,target)]
                i=diffs.index(max(diffs))
                if final:
                    raise RuntimeError(
                        f'{label}未到位：编码器估算末端偏差 {self.xyz_error_mm:.1f} mm'
                        f'（要求 ≤ {XYZ_DEGRADED_MM:g} mm 且停稳）；'
                        f'{NAMES[i]}目标 {math.degrees(target[i]):.1f}°，反馈 {math.degrees(pose[KEYS[i]]):.1f}°')
                raise RuntimeError(f'{NAMES[i]}未到位：目标 {math.degrees(target[i]):.1f}°，实测 {math.degrees(pose[KEYS[i]]):.1f}°，回放已停止')
            self.stop.wait(.05)

    def correct_dwell_target(self, target, observed, xyz_error):
        motor = list(self.last_motor_target)
        for joint in (1, 2):
            offset = (motor[joint] - target[joint]) + (target[joint] - observed[joint])
            motor[joint] = target[joint] + max(-DWELL_CORRECTION_LIMIT, min(DWELL_CORRECTION_LIMIT, offset))
        if max(abs(motor[joint] - self.last_motor_target[joint]) for joint in (1, 2)) < ENCODER_STEP:
            raise RuntimeError(f'示教停顿点微调已达限额，末端仍偏 {xyz_error:.1f} mm')
        self.write_motor_target(target, motor, min(self.speed, 100), min(self.acc, 5))
        offsets = tuple(motor[joint] - target[joint] for joint in (1, 2))
        self.dwell_correction = (tuple(target), offsets)
        self.shared.event(f'示教停顿点微调：末端偏 {xyz_error:.1f} mm，'
                          f'肩部偏置 {math.degrees(offsets[0]):+.2f}°，肘部偏置 {math.degrees(offsets[1]):+.2f}°')

    def wait_until(self, deadline):
        while True:
            self.ensure()
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return
            self.stop.wait(min(remaining, REPLAY_PERIOD))

    def check_tracking(self, target):
        self.ensure()
        with self.shared.lock:
            pose, stamp = self.shared.pose, self.shared.pose_time
        if stamp == self.tracking_stamp:
            return
        self.tracking_stamp = stamp
        errors = [abs(pose[key] - value) for key, value in zip(ARM_KEYS, target)]
        ratio = max(error / limit for error, limit in zip(errors, ARM_TOLERANCES))
        for i, condition in enumerate((ratio > 1., ratio > 2., ratio <= .8)):
            if not condition:
                self.tracking_since[i] = None
            elif self.tracking_since[i] is None:
                self.tracking_since[i] = stamp

        late, excessive, recovered = [
            start is not None and stamp - start >= TRACKING_CONFIRM_SECONDS
            for start in self.tracking_since
        ]
        if excessive:
            self.recover_tracking(target, stamp)
            return True
        if late and self.tracking_rate == 1.:
            self.tracking_rate = TRACKING_SLOW_RATE
            self.shared.event('示教途中跟随落后，轨迹时钟已降至计划速度的 50%')
        elif recovered and self.tracking_rate != 1.:
            self.tracking_rate = 1.
            self.shared.event('示教途中跟随恢复，轨迹时钟已恢复计划速度')

    def recover_tracking(self, target, stamp):
        """Freeze the trajectory and let the arm finish its last written target."""
        self.shared.event('示教途中跟随超差，暂停发点，等待追上当前目标')
        deadline = time.monotonic() + TRACKING_RECOVERY_SECONDS
        good_frames = 0
        while True:
            self.ensure()
            with self.shared.lock:
                pose, latest = self.shared.pose, self.shared.pose_time
            errors = [abs(pose[key] - value) for key, value in zip(ARM_KEYS, target)]
            if stamp < latest <= deadline:
                stamp = latest
                in_range = all(error <= limit for error, limit in zip(errors, ARM_TOLERANCES))
                good_frames = good_frames + 1 if in_range else 0
                if good_frames >= 2:
                    self.reset_tracking()
                    self.tracking_stamp = stamp
                    self.tracking_rate = TRACKING_SLOW_RATE
                    self.shared.event('示教已追上当前目标，以半速继续')
                    return
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                joint = max(range(len(ARM_KEYS)), key=lambda i: errors[i] / ARM_TOLERANCES[i])
                raise RuntimeError(f'示教途中跟随超差：{NAMES[joint]}未在 1 秒内追上，'
                                   f'偏差 {math.degrees(errors[joint]):.1f}°，已停止后续动作')
            self.stop.wait(min(remaining, REPLAY_PERIOD))

    def run(self):
        try:
            self.ensure()
            first = self.records[0]['radians']
            delta = max(abs(self.shared.pose[k]-v) for k, v in zip(ARM_KEYS, first))
            self.shared.event('回放：先移动到首帧位置')
            self.send(first)
            if len(self.records) == 1:
                self.wait_target(first, delta * STEPS_PER_RAD / self.speed + 15, final=True)
            else:
                self.wait_target(first, delta * STEPS_PER_RAD / self.speed + 15)
            last_target = first
            deadline = written_at = time.monotonic()
            holds = set(dwell_starts(self.records))
            total_time = self.records[-1]['time']
            for i in range(1, len(self.records)):
                a, b = self.records[i-1], self.records[i]
                if i > 1 and i - 1 in holds:
                    self.wait_target(last_target, 5, final=True, label='示教停顿点', correct_dwell=True)
                    self.reset_tracking()
                    deadline = written_at = time.monotonic()
                duration = max((b['time']-a['time'])/self.factor,
                               max(abs(x-y) for x, y in zip(a['radians'], b['radians'])) * STEPS_PER_RAD / self.speed,
                               REPLAY_PERIOD)
                samples = max(1, int(duration / REPLAY_PERIOD))
                interval = duration / samples
                for sample in range(1, samples + 1):
                    step = interval / self.tracking_rate
                    deadline += step
                    # Absorb ordinary dispatch jitter, but never burst overdue targets.
                    if deadline - written_at < REPLAY_PERIOD * .4:
                        deadline = written_at + step
                    self.wait_until(deadline)
                    if self.check_tracking(last_target):
                        deadline = time.monotonic()
                    ratio = sample / samples
                    last_target = (b['radians'] if sample == samples else
                                   [x + (y - x) * ratio for x, y in zip(a['radians'], b['radians'])])
                    self.send(last_target)
                    written_at = self.receipt.written_at if self.receipt is not None else time.monotonic()
                    if i < len(self.records) - 1 or sample < samples:
                        self.progress = (a['time'] + (b['time'] - a['time']) * ratio) / total_time
            if len(self.records) > 1:
                self.wait_target(last_target, 5, final=True)
            self.progress = 1.
            self.shared.event('示教回放完成')
        except InterruptedError:
            self.cancelled = True
            self.shared.event('示教回放已停止')
        except Exception as e:
            self.error = str(e)
            logging.getLogger(__name__).warning('示教回放失败（进度 %.1f%%）：%s', self.progress * 100, self.error)
            self.shared.event(f'回放：{e}')
        finally:
            with self.arm.command_lock:
                if self.arm.epoch == self.token and (not self.external or self.error or self.cancelled):
                    self.arm.cancel_replay()
                    # Keep the last motor target, including its load-holding bias.
                    # Encoder angles are observations, not replacement targets.
                    if self.error:
                        self.shared.event('已取消回放待发动作；已发送的目标可能继续执行，未改写保持目标。')
