"""ROS map/FLU metres <-> first-observation FRU centimetres.

FRU is left handed: reflect Y AND negate yaw. Origin is fixed for the whole
instruction, never moved each inference step. Returned action is an absolute
target in that mission frame, so do NOT add it to the current pose again.
"""
import math


def wrap(angle):
    return (angle + math.pi) % (2 * math.pi) - math.pi


def finite(values):
    try:
        return all(not isinstance(v, (bool, str, bytes)) and math.isfinite(float(v)) for v in values)
    except (TypeError, ValueError, OverflowError):
        return False


def rotate_xy(x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return c*x - s*y, s*x + c*y


def quaternion_rpy(q):
    if len(q) != 4 or not finite(q):
        raise ValueError("invalid quaternion")
    n = math.sqrt(sum(x*x for x in q))
    if n < 0.5 or n > 1.5:
        raise ValueError("quaternion norm invalid")
    x, y, z, w = [a/n for a in q]
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1., min(1., 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


class MissionFrame:
    def __init__(self, position, body_yaw, camera_yaw_offset=0.0):
        if len(position) != 3 or not finite(list(position) + [body_yaw, camera_yaw_offset]):
            raise ValueError("invalid origin")
        self.position = tuple(position)
        self.camera_yaw_offset = camera_yaw_offset
        self.yaw = wrap(body_yaw + camera_yaw_offset)

    def encode_pose(self, position, body_yaw):
        if len(position) != 3 or not finite(list(position) + [body_yaw]):
            raise ValueError("invalid pose")
        dx, dy, dz = [position[i] - self.position[i] for i in range(3)]
        x, y = rotate_xy(dx, dy, -self.yaw)
        yaw = -math.degrees(wrap(body_yaw + self.camera_yaw_offset - self.yaw))
        return [100*x, -100*y, 100*dz, yaw]

    def decode_action(self, action):
        if not isinstance(action, (list, tuple)) or len(action) != 4 or not finite(action):
            raise ValueError("invalid action")
        x, y = rotate_xy(action[0]/100., -action[1]/100., self.yaw)
        position = (self.position[0]+x, self.position[1]+y,
                    self.position[2]+action[2]/100.)
        yaw = wrap(self.yaw - action[3] - self.camera_yaw_offset)
        return position, yaw


def distance(a, b):
    return math.sqrt(sum((a[i]-b[i])**2 for i in range(3)))


def check_goal(goal, observation, current, origin, *, max_step,
               max_drift, max_radius, min_z, max_z, max_vertical_step):
    if any(len(v) != 3 or not finite(v) for v in (goal, observation, current, origin)):
        raise ValueError("invalid goal or pose")
    if distance(current, observation) > max_drift:
        raise ValueError("vehicle moved too far during inference")
    if distance(goal, observation) > max_step or distance(goal, current) > max_step:
        raise ValueError("goal step exceeds limit")
    if distance(goal, origin) > max_radius:
        raise ValueError("goal exceeds mission radius")
    if not min_z < goal[2] < max_z:
        raise ValueError("goal altitude outside allowed map range")
    if abs(goal[2]-current[2]) > max_vertical_step:
        raise ValueError("vertical step exceeds limit")
