"""ROS 메시지·좌표·시각 변환을 위한 보조 함수입니다."""

import math
import numpy as np
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Int8MultiArray, MultiArrayDimension


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def rotation_matrix(quaternion):
    q = np.array([quaternion.x, quaternion.y, quaternion.z, quaternion.w], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-6:
        raise ValueError('invalid quaternion')
    x, y, z, w = q / norm
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def planar_pose(pose):
    r = rotation_matrix(pose.orientation)
    xyz = np.array([pose.position.x, pose.position.y, pose.position.z])
    if not np.isfinite(xyz).all():
        raise ValueError('invalid pose position')
    return np.array([xyz[0], xyz[1], math.atan2(r[1, 0], r[0, 0])])


def path_message(poses, frame, stamp, z=0.0):
    message = Path()
    message.header.frame_id, message.header.stamp = frame, stamp
    for x, y, yaw in np.asarray(poses).reshape(-1, 3):
        p = PoseStamped()
        p.header = message.header
        p.pose.position.x, p.pose.position.y, p.pose.position.z = float(x), float(y), float(z)
        p.pose.orientation.z, p.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        message.poses.append(p)
    return message


def direction_message(directions, plan_id):
    message = Int8MultiArray()
    dimension = MultiArrayDimension(label=f'plan_id={plan_id}', size=len(directions), stride=len(directions))
    message.layout.dim = [dimension]
    message.data = [int(d) for d in directions]
    return message
