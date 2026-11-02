"""ROS-independent control rules for the simulation baseline."""
import math

def clamp(value, limit):
    return max(-limit, min(limit, value))

def follow_command(x, y, yaw, target_x, target_y, distance=1.0):
    if not all(math.isfinite(v) for v in (x, y, yaw, target_x, target_y, distance)):
        return 0.0, 0.0
    dx, dy = target_x-x, target_y-y
    error = math.hypot(dx, dy)-distance
    if error <= 0.05:
        return 0.0, 0.0
    heading = math.atan2(dy, dx)-yaw
    heading = math.atan2(math.sin(heading), math.cos(heading))
    turn = clamp(1.5*heading, 1.0)
    speed = min(0.3, 0.5*error) if abs(heading) < 0.5 else 0.0
    return speed, turn

def guarded_command(linear, angular, command_age, scan_age, ranges,
                    angle_min, angle_increment, range_min, range_max):
    # Deliberately stop without fresh input or a fresh valid lidar scan.
    if (not all(math.isfinite(v) for v in (linear, angular)) or
            command_age > 0.5 or scan_age > 0.5 or not ranges):
        return 0.0, 0.0
    relevant = []
    for i, distance in enumerate(ranges):
        angle = math.atan2(math.sin(angle_min+i*angle_increment),
                           math.cos(angle_min+i*angle_increment))
        center = 0.0 if linear >= 0.0 else math.pi
        delta = math.atan2(math.sin(angle-center), math.cos(angle-center))
        if abs(delta) < 0.65:
            # +infinity is a conventional no-return reading; NaN is unusable.
            if distance == math.inf:
                relevant.append(range_max)
            elif math.isfinite(distance) and range_min <= distance <= range_max:
                relevant.append(distance)
    if not relevant:
        return 0.0, 0.0
    if min(relevant) < 0.45:
        linear = 0.0
    return clamp(linear, 0.4), clamp(angular, 1.2)
