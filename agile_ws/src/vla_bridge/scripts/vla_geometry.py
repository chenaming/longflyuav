#!/usr/bin/env python3
"""First-frame / ENU geometry for vla_bridge. No ROS imports."""

from __future__ import print_function

import math

import numpy as np


def wrap_pi(angle):
    return (float(angle) + math.pi) % (2.0 * math.pi) - math.pi


def yaw_from_quat(x, y, z, w):
    # ZYX yaw; units: radians.
    _roll, _pitch, yaw = rpy_from_quat(x, y, z, w)
    return yaw


def rpy_from_quat(x, y, z, w):
    """ROS geometry_msgs quaternion -> roll, pitch, yaw radians (ZYX / sxyz)."""
    return rpy_from_rotmat(quat_to_rotmat(x, y, z, w))


def rpy_from_rotmat(rot):
    rot = np.asarray(rot, dtype=np.float64).reshape(3, 3)
    sy = math.sqrt(float(rot[0, 0]) * float(rot[0, 0]) + float(rot[1, 0]) * float(rot[1, 0]))
    if sy > 1e-6:
        roll = math.atan2(float(rot[2, 1]), float(rot[2, 2]))
        pitch = math.atan2(-float(rot[2, 0]), sy)
        yaw = math.atan2(float(rot[1, 0]), float(rot[0, 0]))
    else:
        roll = math.atan2(-float(rot[1, 2]), float(rot[1, 1]))
        pitch = math.atan2(-float(rot[2, 0]), sy)
        yaw = 0.0
    return roll, pitch, yaw


def yaw_to_quat(yaw):
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quat_to_rotmat(x, y, z, w):
    xx, yy, zz = x * x, y * y, z * z
    xy, xz, yz = x * y, x * z, y * z
    wx, wy, wz = w * x, w * y, w * z
    return np.array(
        [
            [1.0 - 2.0 * (yy + zz), 2.0 * (xy - wz), 2.0 * (xz + wy)],
            [2.0 * (xy + wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz - wx)],
            [2.0 * (xz - wy), 2.0 * (yz + wx), 1.0 - 2.0 * (xx + yy)],
        ],
        dtype=np.float64,
    )


def apply_odom_to_enu(position, quaternion, rot, trans):
    p_enu, rpy_enu = apply_odom_to_enu_rpy(position, quaternion, rot, trans)
    return p_enu, rpy_enu[2]


def apply_odom_to_enu_rpy(position, quaternion, rot, trans):
    p_odom = np.asarray(position, dtype=np.float64).reshape(3)
    p_enu = rot.dot(p_odom) + trans
    r_enu = rot.dot(quat_to_rotmat(*quaternion))
    return p_enu, rpy_from_rotmat(r_enu)


def first_frame_proprio(p_enu, yaw_enu, p0, yaw0):
    # Plan 3.4.2: p_local = Rz(-psi0) * (p - p0); yaw in degrees.
    delta = np.asarray(p_enu, dtype=np.float64).reshape(3) - np.asarray(p0, dtype=np.float64).reshape(3)
    cos0 = math.cos(yaw0)
    sin0 = math.sin(yaw0)
    p_local = np.array(
        [
            cos0 * delta[0] + sin0 * delta[1],
            -sin0 * delta[0] + cos0 * delta[1],
            delta[2],
        ],
        dtype=np.float64,
    )
    yaw_local_deg = math.degrees(wrap_pi(yaw_enu - yaw0))
    return [float(p_local[0]), float(p_local[1]), float(p_local[2]), float(yaw_local_deg)]


def first_frame_log6(p_enu, rpy_enu, p0, rpy0):
    """Plan 3.4.2 pose plus roll/pitch relative to the first frame. Angles in degrees."""
    roll0, pitch0, yaw0 = rpy0
    roll, pitch, yaw = rpy_enu
    xyz_yaw = first_frame_proprio(p_enu, yaw, p0, yaw0)
    roll_l = math.degrees(wrap_pi(float(roll) - float(roll0)))
    pitch_l = math.degrees(wrap_pi(float(pitch) - float(pitch0)))
    return [xyz_yaw[0], xyz_yaw[1], xyz_yaw[2], roll_l, xyz_yaw[3], pitch_l]


def first_frame_to_enu(p_local, yaw_local_rad, p0, yaw0):
    """Plan 3.4.3 only. Do not apply a second transform_to_global."""
    local = np.asarray(p_local, dtype=np.float64).reshape(3)
    origin = np.asarray(p0, dtype=np.float64).reshape(3)
    cos0 = math.cos(yaw0)
    sin0 = math.sin(yaw0)
    # Rz(psi0) * p_local
    p_enu = origin + np.array(
        [
            cos0 * local[0] - sin0 * local[1],
            sin0 * local[0] + cos0 * local[1],
            local[2],
        ],
        dtype=np.float64,
    )
    yaw_enu = wrap_pi(float(yaw_local_rad) + float(yaw0))
    return p_enu, yaw_enu


def is_finite_vec4(values):
    if values is None or len(values) < 4:
        return False
    return all(math.isfinite(float(values[i])) for i in range(4))


def clamp_step(p_target, yaw_target, p_now, yaw_now, max_xy, max_z, max_yaw):
    """Limit one command relative to current ENU pose. Yaw args are radians."""
    target = np.asarray(p_target, dtype=np.float64).reshape(3).copy()
    now = np.asarray(p_now, dtype=np.float64).reshape(3)
    delta_xy = target[:2] - now[:2]
    dist_xy = float(np.linalg.norm(delta_xy))
    if dist_xy > max_xy and dist_xy > 1e-9:
        target[:2] = now[:2] + delta_xy * (float(max_xy) / dist_xy)
    dz = float(np.clip(target[2] - now[2], -float(max_z), float(max_z)))
    target[2] = now[2] + dz
    dyaw = wrap_pi(float(yaw_target) - float(yaw_now))
    dyaw = float(np.clip(dyaw, -float(max_yaw), float(max_yaw)))
    yaw = wrap_pi(float(yaw_now) + dyaw)
    return target, yaw


def lerp_pose(p0, yaw0, p1, yaw1, alpha):
    """Linear position lerp and shortest-angle yaw lerp. Yaw is radians."""
    a = float(np.clip(alpha, 0.0, 1.0))
    p0 = np.asarray(p0, dtype=np.float64).reshape(3)
    p1 = np.asarray(p1, dtype=np.float64).reshape(3)
    p = (1.0 - a) * p0 + a * p1
    dyaw = wrap_pi(float(yaw1) - float(yaw0))
    yaw = wrap_pi(float(yaw0) + a * dyaw)
    return p, yaw
