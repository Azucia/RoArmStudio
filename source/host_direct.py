"""Resolve a PC-side XYZ target to one native all-joint command; no I/O."""
import math
from backend import TCP_TARGET_ERROR, all_command, check_radians, joint_command
from host_control import finite, forward, inverse, xy_in_sector


def prepare(values, measured, speed, acc, z_feedforward_mm=0, *, allow_degraded=False, reference=None):
    values = finite(values, 6)
    finite(measured, 6)
    goal=values[:3]+[math.radians(v) for v in values[3:]]
    if not math.isfinite(z_feedforward_mm) or not 0 <= z_feedforward_mm <= 50:
        raise ValueError('Z 前馈范围为 0–50 mm')
    goal[2] += z_feedforward_mm
    if not xy_in_sector(*goal[:2]):
        raise ValueError(TCP_TARGET_ERROR)
    # Roll and gripper opening are never relaxed, even when the XYZ solve is degraded.
    for i in (4, 5):
        joint_command(i, values[i])
    try:
        joints = inverse(goal)
    except ValueError:
        if not allow_degraded:
            raise
        joints = best_effort(goal, reference)
    # Exactly the same function, joint convention and speed/acc as move_all().
    return all_command(joints,speed,acc),joints


def nearest_pitch(goal):
    """Try the original pitch, then nearby pitches in one-degree increments."""
    candidate = goal.copy()
    for degrees in range(181):
        for sign in ((-1, 1) if degrees else (1,)):
            candidate[3] = goal[3] + sign * math.radians(degrees)
            try:
                return inverse(candidate)
            except ValueError:
                continue
    return None


def best_effort(goal, reference):
    joints = nearest_pitch(goal)
    if joints is not None:
        return joints
    if reference is None:
        raise ValueError('XYZ 不可达，且缺少已发送的保持目标。请先回中建立基准。')
    start = forward(check_radians(reference))[:3]
    travel = math.dist(start, goal[:3])

    def at_fraction(fraction):
        xyz = [a + (b - a) * fraction for a, b in zip(start, goal[:3])]
        if xy_in_sector(*xyz[:2]):
            return nearest_pitch(xyz + goal[3:])
        return None

    # Check the requested line from its far end before refining the boundary.
    # Reachability need not be monotonic over the entire line.
    lower, upper = 0., 1.
    for step in range(15, 0, -1):
        fraction = step / 16
        candidate = at_fraction(fraction)
        if candidate is not None:
            lower, joints = fraction, candidate
            break
        upper = fraction
    for _ in range(10):
        fraction = (lower + upper) / 2
        candidate = at_fraction(fraction)
        if candidate is None:
            upper = fraction
        else:
            lower, joints = fraction, candidate
    if joints is None or travel * lower < .05:
        raise ValueError('该方向已经到达可用工作区边界')
    return joints


def plan_xyz(values, measured, speed, acc, bias, offset, reference, *, allow_degraded):
    """Share load compensation and accepted target semantics between UI and API."""
    if not xy_in_sector(*values[:2]):
        raise ValueError(TCP_TARGET_ERROR)
    solve = [value + delta for value, delta in zip(values, offset)]
    extra = max(0., bias - offset[2])
    command, joints = prepare(solve, measured, speed, acc, extra,
                              allow_degraded=allow_degraded, reference=reference)
    planned = forward(joints)
    accepted = list(values)
    if math.dist(planned[:3], [solve[0], solve[1], solve[2] + extra]) > 1e-5:
        accepted[:3] = [planned[0] - offset[0], planned[1] - offset[1], planned[2] - offset[2] - extra]
    pitch = math.degrees(planned[3])
    detail = ''
    if accepted != values or abs(pitch - solve[3]) > 1e-5:
        detail = f'计划末端 X {planned[0]:.1f} · Y {planned[1]:.1f} · Z {planned[2]:.1f} mm，俯仰 {pitch:.1f}°'
    return command, joints, accepted, detail


def holding_offset(actual, last_joint_target):
    """Bridge measured and commanded coordinates once per relative-jog session."""
    if last_joint_target is None:
        raise ValueError('首次点动需要已知的保持目标，请先用电脑端回中并等机械臂停稳。')
    pose=forward(last_joint_target)
    held=pose[:3]+[math.degrees(v) for v in pose[3:]]
    offset=[h-a for h,a in zip(held,actual)]
    # The existing loaded target already supplies some or all of the Z feedforward.
    return offset
