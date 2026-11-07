"""Native-angle teaching files and cancellable, rate-limited trajectory playback."""
import json
import math
import time
from pathlib import Path
from backend import Worker, KEYS, NAMES, all_command, check_radians, FEEDBACK_TIMEOUT, torque_disabled


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
            self.path.parent.mkdir(parents=True, exist_ok=True)
            temp = self.path.with_suffix('.tmp')
            temp.write_text(json.dumps(dict(format='roarm-studio-native-v1', records=records),
                                       ensure_ascii=False), encoding='utf-8')
            temp.replace(self.path)
            self.shared.event(f'示教轨迹已保存：{self.path}，{len(records)} 点')
        except Exception as e:
            self.error = str(e)
            self.shared.event(f'示教录制：{e}')


class Playback(Worker):
    def __init__(self, shared, arm, records, factor=.5, speed=150, acc=5):
        super().__init__(shared)
        self.arm, self.records = arm, records
        self.factor, self.speed, self.acc = factor, speed, acc
        self.token = arm.begin_replay()
        self.progress = 0.
        self.degraded = False
        self.error = None

    def ensure(self):
        if self.stop.is_set() or self.arm.stop.is_set() or self.arm.epoch != self.token:
            raise InterruptedError('回放已停止')
        if not self.arm.connected or time.monotonic()-self.shared.pose_time > FEEDBACK_TIMEOUT:
            raise RuntimeError('超过 0.6 秒未收到反馈，回放已终止；恢复连接后不会自动续播')
        if self.arm.teaching or self.arm.torque_pending or self.arm.torque_fault:
            raise RuntimeError('扭矩状态未确认，回放已终止')
        if torque_disabled(self.shared.pose):
            raise RuntimeError('检测到关节扭矩关闭，请先恢复保持再回放')

    def send(self, values):
        self.ensure()
        receipt = self.arm.submit(all_command(values, self.speed, self.acc), token=self.token)
        deadline = time.monotonic()+1
        # Feedback can arrive before a queued command is written. Do not start
        # arrival checks (or complete and clear the queue) for an unsent target.
        while not receipt.done.wait(.01):
            self.ensure()
            if time.monotonic() > deadline:
                raise RuntimeError('回放指令未确认完整写出串口，已取消后续动作')
        self.ensure()
        if receipt.error:
            raise RuntimeError(receipt.error)

    def wait_target(self, target, timeout):
        end = time.monotonic()+timeout
        anchor=None
        stamp_before=self.shared.pose_time
        while True:
            self.ensure()
            with self.shared.lock:
                stamp=self.shared.pose_time
                pose=self.shared.pose
            if pose and stamp>stamp_before:
                q=[pose[k] for k in KEYS]
                errors=[abs(v-t) for v,t in zip(q,target)]
                if max(errors)<.07:
                    return
                # Only the loaded shoulder/elbow may settle slightly outside 4 degrees.
                tolerances=(.07,math.radians(6),math.radians(6),.07,.07,.07)
                if all(e<=limit for e,limit in zip(errors,tolerances)):
                    if anchor is None or max(abs(v-w) for v,w in zip(q,anchor[1]))>math.radians(.35):
                        anchor=(stamp,q)
                    elif stamp-anchor[0]>=.45:
                        self.degraded = True
                        self.shared.event('带载关节已停稳，按实测容差继续回放')
                        return
                else:
                    anchor=None
            if time.monotonic() > end:
                diffs=[abs(pose[k]-v) for k,v in zip(KEYS,target)]
                i=diffs.index(max(diffs))
                raise RuntimeError(f'{NAMES[i]}未到位：目标 {math.degrees(target[i]):.1f}°，实测 {math.degrees(pose[KEYS[i]]):.1f}°，回放已停止')
            self.stop.wait(.05)

    def run(self):
        try:
            self.ensure()
            first = self.records[0]['radians']
            delta = max(abs(self.shared.pose[k]-v) for k, v in zip(KEYS, first))
            self.shared.event('回放：先低速到达首帧位置')
            self.send(first)
            self.wait_target(first, delta*2048/math.pi/self.speed+15)
            for i in range(1, len(self.records)):
                a, b = self.records[i-1], self.records[i]
                duration = max((b['time']-a['time'])/self.factor,
                               max(abs(x-y) for x, y in zip(a['radians'], b['radians']))*2048/math.pi/self.speed,
                               .1)
                start = time.monotonic()
                while True:
                    ratio = min(1., (time.monotonic()-start)/duration)
                    self.send([x+(y-x)*ratio for x, y in zip(a['radians'], b['radians'])])
                    if ratio >= 1:
                        break
                    self.stop.wait(.1)
                self.wait_target(b['radians'], duration+5)
                self.progress = i/(len(self.records)-1)
            self.shared.event('示教回放完成')
        except InterruptedError:
            self.shared.event('示教回放已停止')
        except Exception as e:
            self.error = str(e)
            self.shared.event(f'回放：{e}')
        finally:
            with self.arm.command_lock:
                if self.arm.epoch == self.token:
                    self.arm.cancel_replay()
                    # Keep the last motor target, including its load-holding bias.
                    # Encoder angles are observations, not replacement targets.
                    if self.error:
                        self.shared.event('已取消回放待发动作；已发送的目标可能继续执行，未改写保持目标。')
