"""One PC inverse-kinematics solve, one native all-joint command; no I/O."""
import math
from backend import all_command
from host_control import finite,forward,inverse

def prepare(values,measured,speed,acc,z_feedforward_mm=0):
    values=finite(values,6);measured=finite(measured,6)
    goal=values[:3]+[math.radians(v) for v in values[3:]]
    if not math.isfinite(z_feedforward_mm) or not 0 <= z_feedforward_mm <= 50:
        raise ValueError('Z 前馈范围为 0–50 mm')
    goal[2]+=z_feedforward_mm
    joints=inverse(goal)
    # Exactly the same function, joint convention and speed/acc as move_all().
    return all_command(joints,speed,acc),joints

def holding_offset(actual, last_joint_target):
    """Bridge measured and commanded coordinates once per relative-jog session."""
    if last_joint_target is None:
        raise ValueError('首次点动需要已知的保持目标，请先用电脑端回中并等机械臂停稳。')
    pose=forward(last_joint_target)
    held=pose[:3]+[math.degrees(v) for v in pose[3:]]
    offset=[h-a for h,a in zip(held,actual)]
    # The existing loaded target already supplies some or all of the Z feedforward.
    return offset
