"""Pure reference planner; these limits do not guarantee physical motion rates."""
import math
from host_control import finite, inverse, forward, check_joints

PERIOD=.2

def needs_pause(desired, actual):
    desired=finite(desired,6); actual=finite(actual,6)
    p,a=forward(desired),forward(actual)
    return (math.dist(p[:3],a[:3])>.75 or abs(p[3]-a[3])>math.radians(.35) or
            max(abs(desired[i]-actual[i]) for i in range(4))>math.radians(.35))

def plan(start,goal,bias,preserved,period=PERIOD):
    """Preflight minimum-jerk Cartesian samples and nominal joint rate limits.

    Reference caps: 2 mm/s, 4 mm/s², 1 deg/s, 2 deg/s². Returns interval,
    Cartesian reference, nominal joints, biased motor targets. No device I/O.
    """
    start,goal=finite(start,6),finite(goal,6)
    bias,preserved=finite(bias,4),finite(preserved,2)
    if not math.isfinite(period) or period<.05: raise ValueError('Invalid interval')
    if max(abs(v) for v in bias)>math.radians(4): raise ValueError('Load bias exceeds 4deg')
    distance=math.dist(start[:3],goal[:3])
    q0=inverse(start); q1=inverse(goal)
    angular=max(abs(a-b) for a,b in zip(q0[:4],q1[:4]))
    # Include orientation-only motion in Cartesian/pitch reference duration.
    pitch=abs(goal[3]-start[3])
    duration=max(period,1.875*distance/2,math.sqrt(5.774*distance/4),
                 1.875*max(angular,pitch)/math.radians(1),
                 math.sqrt(5.774*max(angular,pitch)/math.radians(2)))
    for attempt in range(12):
        n=max(2,math.ceil(duration/period));dt=duration/n
        samples=[];previous=q0;previous_velocity=[0.]*4;valid=True
        for k in range(1,n+1):
            u=k/n;fraction=10*u**3-15*u**4+6*u**5
            pose=[a+(b-a)*fraction for a,b in zip(start,goal)]
            q=inverse(pose)
            velocity=[(q[i]-previous[i])/dt for i in range(4)]
            acceleration=[(velocity[i]-previous_velocity[i])/dt for i in range(4)]
            if max(map(abs,velocity))>math.radians(1) or max(map(abs,acceleration))>math.radians(2):valid=False
            motor=check_joints([q[i]+bias[i] for i in range(4)]+preserved)
            samples.append((dt,pose,q,motor));previous=q;previous_velocity=velocity
        if max(map(abs,previous_velocity))/dt>math.radians(2):valid=False
        if valid:return samples
        duration*=1.4
    raise ValueError('无法在参考速度限制内规划本次路径')
