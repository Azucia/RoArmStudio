"""PC-side M3 kinematics and bounded joint correction. No device I/O on import."""
import math

KEYS=('b','s','e','t','r','g')
L2=math.hypot(236.82,30.0)
T2=math.atan2(30.0,236.82)
L3=144.49
LE=math.hypot(171.67,13.69)
TE=math.atan2(13.69,171.67)

def finite(values,size):
    values=list(values)
    if len(values)!=size or not all(isinstance(v,(int,float)) and math.isfinite(v) for v in values):
        raise ValueError('Invalid finite vector')
    return values

def forward(joints):
    b,s,e,w,r,g=finite(joints,6)
    radius=L2*math.sin(s+T2)+L3*math.sin(s+e)+LE*math.sin(s+e+w+TE)
    z=L2*math.cos(s+T2)+L3*math.cos(s+e)+LE*math.cos(s+e+w+TE)
    return [radius*math.cos(b),radius*math.sin(b),z,s+e+w-math.pi/2,r,g]

def check_joints(joints):
    q=finite(joints,6)
    limits=[(-math.pi,math.pi),(-math.pi/2,math.pi/2),(0,math.pi),(-math.pi/2,math.pi/2)]
    if any(not lo<=v<=hi for v,(lo,hi) in zip(q[:4],limits)):
        raise ValueError('Joint limit; command is not clamped')
    if abs(q[4])>math.pi+.02 or not 1.0<=q[5]<=3.2:
        raise ValueError('Preserved roll/gripper outside operating envelope')
    return q

def inverse(pose):
    x,y,z,pitch,roll,grip=finite(pose,6)
    radius=math.hypot(x,y)-LE*math.cos(pitch+TE)
    height=z+LE*math.sin(pitch+TE)
    distance=math.hypot(radius,height)
    if radius<=1e-6 or not abs(L2-L3)+1e-4<distance<L2+L3-1e-4:
        raise ValueError('Unreachable or near singular target')
    psi=math.acos((L2*L2+distance*distance-L3*L3)/(2*L2*distance))+T2
    omega=math.acos((L3*L3+distance*distance-L2*L2)/(2*distance*L3))
    s=math.pi/2-math.atan2(height,radius)-psi
    e=psi+omega
    q=check_joints([math.atan2(y,x),s,e,math.pi/2-s-e+pitch,roll,grip])
    if max(abs(a-b) for a,b in zip(forward(q)[:4],[x,y,z,pitch]))>1e-6:
        raise ValueError('Kinematic verification failed')
    return q

def command(joints):
    q=check_joints(joints)
    return dict(T=102,**dict(zip(('base','shoulder','elbow','wrist','roll','hand'),q)),spd=60,acc=5)

class JointCorrection:
    """Frozen desired joint pose; roll/gripper never participate in correction.

    Driver must wait for measured settling and acknowledge each write before next().
    Stops on stale, divergent, excessive or exhausted correction; no fallback motion.
    """
    def __init__(self,desired):
        self.desired=check_joints(desired)
        self.sent=self.desired.copy()
        self.count=0
        self.last_error=None

    def next(self,actual,age):
        actual=finite(actual,6)
        if not 0<=age<=.4:raise ValueError('Stale feedback')
        errors=[self.desired[i]-actual[i] for i in range(4)]
        norm=math.sqrt(sum(e*e for e in errors))
        if max(abs(e) for e in errors)>math.radians(3):raise ValueError('Tracking error too large')
        if norm<=math.radians(.2):return None
        if self.last_error is not None and norm>=self.last_error-math.radians(.02):
            raise ValueError('Correction not improving')
        if self.count>=6:raise ValueError('Correction budget exhausted')
        candidate=self.sent.copy()
        for i,e in enumerate(errors):
            delta=max(-math.radians(.5),min(math.radians(.5),.35*e))
            candidate[i]+=delta
            if abs(candidate[i]-self.desired[i])>math.radians(2):
                raise ValueError('Joint correction offset exceeds limit')
        check_joints(candidate)
        # No state is changed unless the complete candidate passed validation.
        self.sent=candidate;self.last_error=norm;self.count+=1
        return command(candidate)

class RetainedLoadBias:
    """Retain bias only from targets owned by the same live control session.

    Not a calibration file: invalidate on reconnect, manual movement, torque change,
    unknown commands or firmware reset. No bias is inferred from encoder position alone.
    """
    def __init__(self,last_sent,measured):
        self.last_sent=check_joints(last_sent)
        measured=finite(measured,6)
        self.bias=[a-b for a,b in zip(self.last_sent[:4],measured[:4])]
        if max(abs(v) for v in self.bias)>math.radians(4):
            raise ValueError('Learned bias outside validated 4deg range')
    def plan(self,pose):
        nominal=inverse(pose)
        candidate=[nominal[i]+self.bias[i] for i in range(4)]+self.last_sent[4:]
        return command(candidate)

class TargetIntent:
    """User-requested pose is separate from measured pose, without an arrival gate."""
    def __init__(self):self.clear()
    def clear(self):self.goal=None;self.key=None
    def values(self,actual,key):
        measured=finite(actual,6)
        return measured if self.goal is None or self.key!=key else self.goal.copy()
    def commit(self,values,key):
        self.goal=finite(values,6);self.key=key
