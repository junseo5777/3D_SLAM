"""후륜축 중심을 기준으로 공유하는 macaron 차량 제원이며 단위는 m·rad입니다."""

import math
from dataclasses import dataclass
import numpy as np


@dataclass(frozen=True)
class Vehicle:
    wheel_base: float = 0.75  # 앞뒤 바퀴 축 사이 거리(m)
    width: float = 0.68  # 차체 전체 폭(m)
    length: float = 1.30  # 차체 전체 길이(m)
    rear_overhang: float = 0.24  # 후륜축에서 뒤 범퍼까지 거리(m)
    height: float = 0.50  # 지면에서 차량 최고점까지 높이(m)
    base_height: float = 0.14  # 지면에서 후륜축 중심까지 높이(m)
    max_steer: float = math.radians(21.0)  # 계산에 사용하는 조향 한계(rad)
    margin: float = 0.12  # 차체 바깥에 추가하는 충돌 여유(m)

    def __post_init__(self):
        values = (self.wheel_base, self.width, self.length, self.height,
                  self.base_height, self.max_steer)
        if not all(math.isfinite(v) and v > 0 for v in values):
            raise ValueError('vehicle dimensions and steering limit must be finite and positive')
        if (not math.isfinite(self.rear_overhang) or self.rear_overhang < 0
                or self.front_overhang < 0 or self.base_height >= self.height
                or self.max_steer >= math.pi / 2
                or not math.isfinite(self.margin) or self.margin < 0):
            raise ValueError('invalid overhang, base height, steering, or collision margin')

    @property
    def front_extent(self):
        return self.length - self.rear_overhang

    @property
    def front_overhang(self):
        return self.front_extent - self.wheel_base

    @property
    def min_radius(self):
        return self.wheel_base / math.tan(self.max_steer)

    def corners(self, pose, margin=None):
        padding = self.margin if margin is None else margin
        local = np.array([
            [-self.rear_overhang - padding, -self.width / 2 - padding],
            [self.front_extent + padding, -self.width / 2 - padding],
            [self.front_extent + padding, self.width / 2 + padding],
            [-self.rear_overhang - padding, self.width / 2 + padding],
        ])
        x, y, yaw = pose
        c, s = math.cos(yaw), math.sin(yaw)
        return local @ np.array([[c, s], [-s, c]]) + [x, y]


def footprint_clearance(poses, obstacles, vehicle):
    """모든 궤적 표본에서 전체 차체 사각형까지의 최소 부호 있는 거리를 계산합니다."""
    poses = np.asarray(poses, dtype=float).reshape(-1, 3)
    obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 2)
    if not np.isfinite(poses).all() or not np.isfinite(obstacles).all():
        return -math.inf
    if not len(poses):
        return -math.inf
    if not len(obstacles):
        return math.inf
    minimum = math.inf
    for x, y, yaw in poses:
        delta = obstacles - [x, y]
        c, s = math.cos(yaw), math.sin(yaw)
        px = c * delta[:, 0] + s * delta[:, 1]
        py = -s * delta[:, 0] + c * delta[:, 1]
        dx = np.maximum(np.maximum(-vehicle.rear_overhang - px,
                                   px - vehicle.front_extent), 0.0)
        dy = np.maximum(np.abs(py) - vehicle.width / 2, 0.0)
        distance = np.hypot(dx, dy) - vehicle.margin
        inside = ((px >= -vehicle.rear_overhang) & (px <= vehicle.front_extent)
                  & (np.abs(py) <= vehicle.width / 2))
        distance[inside] = -max(vehicle.margin, 1e-6)
        minimum = min(minimum, float(distance.min()))
    return minimum
