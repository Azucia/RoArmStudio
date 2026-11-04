"""Original-firmware PC kinematics with session-owned load bias, no T1041."""
import math,time,json,uuid
from pathlib import Path
from backend import Worker,KEYS
from host_control import forward,inverse,command,check_joints
from host_trajectory import plan,needs_pause

class HostPosition(Worker):
    def __init__(self,shared,arm,values):
        super().__init__(shared)
        self.arm=arm;self.error=None;self.trace_file=None;self.corrections=0
        self.goal=list(values[:3])+[math.radians(v) for v in values[3:]]
        inverse(self.goal) # validation before reserving the serial command stream
        if arm.last_joint_target is None:
            raise ValueError('电脑定位需要已知关节目标：首次连接请先点击回中建立基准。')
        with shared.lock:
            actual=[shared.pose[k] for k in KEYS] if shared.pose else None
        if actual is None:raise ValueError('缺少关节反馈')
        self.origin=forward(actual)
        self.token=arm.begin_replay()
        arm.host_busy=True

    def ensure(self):
        a=self.arm
        if self.stop.is_set() or a.stop.is_set() or a.epoch!=self.token:
            raise InterruptedError('电脑定位已取消后续指令')
        if not a.connected or a.teaching or a.torque_pending or a.torque_fault:
            raise RuntimeError('连接或扭矩状态不允许电脑定位')
        if hasattr(self,'deadline') and time.monotonic()>self.deadline:
            raise RuntimeError('本次电脑定位超过120秒预算')

    def record(self,kind,**data):
        if self.trace_file:
            self.trace_file.write(json.dumps(dict(event=kind,monotonic=time.monotonic(),
                wall_time_ns=time.time_ns(),**data),ensure_ascii=False)+'\n')
            self.trace_file.flush()

    def snapshot(self):
        self.ensure()
        with self.shared.lock:
            p=dict(self.shared.pose or {});age=time.monotonic()-self.shared.pose_time;stamp=self.shared.pose_host_ns
        if age>.4 or not all(p.get('torswitch'+k)==1 for k in ('B','S','E','T','R','G')):
            raise RuntimeError('反馈过期或扭矩状态异常')
        if not 1000<=p.get('v',0)<=1260:
            raise RuntimeError('供电反馈不在验证范围')
        q=[p.get(k) for k in KEYS]
        if not all(isinstance(v,(int,float)) and math.isfinite(v) for v in q):
            raise RuntimeError('关节反馈无效')
        if any(abs(p.get(k,0))>300 for k in ('tE','tT')) or abs(p.get('tS',0))>250:
            raise RuntimeError('关节负载超过验证范围')
        if max(abs(a-b) for a,b in zip(forward(q)[:3],self.origin[:3]))>35:
            raise RuntimeError('实测位移超过验证范围')
        if stamp!=getattr(self,'logged_stamp',None):
            self.record('feedback',feedback=p,feedback_stamp=stamp,age=age,computed_pose=forward(q))
            self.logged_stamp=stamp
        return q,stamp

    def settled(self):
        start=time.monotonic();history=[];last_stamp=None
        while time.monotonic()-start<4:
            q,stamp=self.snapshot()
            if stamp!=last_stamp:
                last_stamp=stamp;history.append((time.monotonic(),q));history=history[-5:]
            if time.monotonic()-start>.8 and len(history)==5 and history[-1][0]-history[0][0]>=.3:
                if max(max(r[1][i] for r in history)-min(r[1][i] for r in history) for i in range(4))<math.radians(.15):
                    return q
            self.stop.wait(.02)
        raise RuntimeError('等待稳定反馈超时；本次停止继续修正')

    def send(self,cmd,nominal=None):
        self.ensure();seq=self.arm.command_sequence
        if cmd.get('T') not in (101,102):raise RuntimeError('电脑模式只允许关节角度指令')
        if cmd.get('T')==102:
            with self.arm.command_lock:
                old=list(self.arm.last_joint_target) if self.arm.last_joint_target is not None else None
            if old is None or max(abs(cmd[k]-old[i]) for i,k in enumerate(('base','shoulder','elbow','wrist')))>math.radians(3):
                raise RuntimeError('本次路径需要过大的关节变化，请减小步长或避开该姿态')
        self.record('submit',command=cmd,nominal=nominal)
        self.arm.submit(cmd,token=self.token)
        end=time.monotonic()+1
        while self.arm.command_sequence<=seq:
            self.ensure()
            if time.monotonic()>end:raise RuntimeError('关节指令未发出')
            self.stop.wait(.01)
        self.record('serial_write_observed',sequence=self.arm.command_sequence)

    def paced_feedback(self,interval,previous_stamp):
        start=time.monotonic()
        while True:
            q,stamp=self.snapshot();elapsed=time.monotonic()-start
            if elapsed>=interval and stamp!=previous_stamp:return q,stamp
            if elapsed>max(.6,interval+.4):raise RuntimeError('没有收到新一帧关节反馈')
            self.stop.wait(.01)

    def correct(self,desired,targets):
        for count in range(13):
            actual=self.settled()
            if not needs_pause(desired,actual):return actual,targets
            if count==12 or self.corrections>=60:
                raise RuntimeError('本次关节修正预算已用完，保持最后受限目标')
            errors=[desired[i]-actual[i] for i in range(4)]
            index=max(range(4),key=lambda i:abs(errors[i]))
            change=max(-math.radians(.25),min(math.radians(.25),.5*errors[index]))
            proposed=targets.copy();proposed[index]+=change
            if abs(proposed[index]-desired[index])>math.radians(4):
                raise RuntimeError('本次关节修正达到4°限制，停止继续修正')
            check_joints(proposed)
            self.record('correction',joint=index+1,residual=errors)
            self.send(dict(T=101,joint=index+1,rad=proposed[index],spd=30,acc=3),desired)
            self.corrections+=1;targets=proposed

    def execute(self):
        actual=self.settled()
        with self.arm.command_lock:
            last=list(self.arm.last_joint_target) if self.arm.last_joint_target is not None else None
        if last is None:raise RuntimeError('控制基准已失效，请先回中建立基准')
        bias=[last[i]-actual[i] for i in range(4)]
        # Clipping this initial bias would create an unplanned first step.
        if max(abs(v) for v in bias)>math.radians(4):
            raise RuntimeError('带载偏移超出4°验证范围；未继续移动')
        reference=forward(actual)
        self.record('initial_reference',nominal=actual,last_motor_target=last,bias=bias,reference=reference)
        while True:
            samples=plan(reference,self.goal,bias,last[4:])
            self.record('plan',start=reference,goal=self.goal,samples=len(samples),duration=sum(s[0] for s in samples))
            restart=False
            for interval,waypoint,desired,targets in samples:
                _,stamp=self.snapshot()
                cmd=command(targets);cmd.update(spd=30,acc=3)
                self.send(cmd,desired)
                actual,_=self.paced_feedback(interval,stamp)
                reference=waypoint
                if needs_pause(desired,actual):
                    self.record('tracking_pause',reference=reference,nominal=desired,actual=actual)
                    actual,targets=self.correct(desired,targets)
                    # Preserve the accepted motor command at this nominal point.
                    # Re-learning from residual actual sag here would inject a
                    # discontinuous extra correction into the next trajectory.
                    bias=[targets[i]-desired[i] for i in range(4)]
                    if max(abs(v) for v in bias)>math.radians(4):
                        raise RuntimeError('更新后的带载偏移超出4°验证范围')
                    # Restart rest-to-rest, without catching up elapsed time or
                    # rebuilding the requested endpoint from measured sag.
                    restart=True;break
            if restart:continue
            actual,targets=self.correct(inverse(self.goal),targets)
            self.record('complete',goal=self.goal,actual=actual,computed_pose=forward(actual))
            self.shared.event('电脑定位完成（编码器计算值到位）；保持当前目标。')
            return

    def run(self):
        try:
            self.ensure()
            folder=Path(__file__).resolve().parent.parent/'diagnostics'
            folder.mkdir(parents=True,exist_ok=True)
            self.log_path=folder/(time.strftime('host-%Y%m%d-%H%M%S-')+uuid.uuid4().hex[:8]+'.jsonl')
            self.trace_file=self.log_path.open('x',encoding='utf-8')
            self.deadline=time.monotonic()+120
            self.record('start',version='host-paced-20261103',goal=self.goal,origin=self.origin,
                        measurement='encoder+FK, not external position',port=getattr(self.arm,'port',None),
                        evidence_kind='hardware' if getattr(self.arm,'port',None) else 'simulation')
            self.execute()
        except Exception as e:
            self.error=str(e)
            try:self.record('error',message=self.error)
            except OSError:pass
            self.shared.event('电脑定位本次结束：'+str(e))
        finally:
            if self.trace_file:self.trace_file.close()
            with self.arm.command_lock:
                if self.arm.epoch==self.token:
                    if self.error is not None:
                        self.arm.cancel_replay()
                    else:
                        self.arm.playing=False
                        self.arm.host_busy=False
