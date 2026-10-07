# Copyright 2026 junseo5777 and kimmc53
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""base_link 좌표계에서 기준 경로를 따르는 DWA 조향 후보를 계산합니다."""

import math
from dataclasses import dataclass

import numpy as np
from .vehicle import Vehicle, footprint_clearance


def normalize_angle(angle):
    """각도를 [-pi, pi] 범위로 정규화합니다."""
    return math.atan2(math.sin(angle), math.cos(angle))


def poses_to_local(poses, vehicle_pose):
    """지도 기준 (x, y, yaw)를 base_link 좌표로 변환합니다."""
    poses = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
    vehicle_x, vehicle_y, vehicle_yaw = vehicle_pose
    delta_x = poses[:, 0] - vehicle_x
    delta_y = poses[:, 1] - vehicle_y
    cosine = math.cos(vehicle_yaw)
    sine = math.sin(vehicle_yaw)

    local = np.empty_like(poses)
    local[:, 0] = cosine * delta_x + sine * delta_y
    local[:, 1] = -sine * delta_x + cosine * delta_y
    yaw_delta = poses[:, 2] - vehicle_yaw
    local[:, 2] = np.arctan2(np.sin(yaw_delta), np.cos(yaw_delta))
    return local


def poses_to_map(poses, vehicle_pose):
    """base_link 기준 (x, y, yaw)를 지도 좌표로 변환합니다."""
    poses = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
    vehicle_x, vehicle_y, vehicle_yaw = vehicle_pose
    cosine = math.cos(vehicle_yaw)
    sine = math.sin(vehicle_yaw)

    mapped = np.empty_like(poses)
    mapped[:, 0] = vehicle_x + cosine * poses[:, 0] - sine * poses[:, 1]
    mapped[:, 1] = vehicle_y + sine * poses[:, 0] + cosine * poses[:, 1]
    yaw_sum = poses[:, 2] + vehicle_yaw
    mapped[:, 2] = np.arctan2(np.sin(yaw_sum), np.cos(yaw_sum))
    return mapped


def nearest_path_index(
        poses,
        position,
        progress_index=0,
        search_back=5,
        search_ahead=80,
        end_index=None):
    """주변 경로 구간에서 가장 가까운 표본을 찾아 진행 위치가 급변하지 않게 합니다."""
    poses = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
    if len(poses) == 0:
        raise ValueError('poses cannot be empty')
    progress_index = int(np.clip(progress_index, 0, len(poses) - 1))
    first = max(0, progress_index - max(0, int(search_back)))
    last = min(
        len(poses) - 1,
        progress_index + max(0, int(search_ahead)),
    )
    if end_index is not None:
        last = min(last, max(first, int(end_index)))
    candidates = poses[first:last + 1, :2]
    distances = np.hypot(
        candidates[:, 0] - position[0],
        candidates[:, 1] - position[1],
    )
    local_index = int(np.argmin(distances))
    return first + local_index, float(distances[local_index])


def path_start_reacquire_metrics(poses, vehicle_pose):
    """첫 경로 위치에 대한 위치·차체 방향 오차를 계산합니다.
    
    경로가 다른 방향으로 시작점 근처에 돌아올 수 있으므로
    진행 위치를 되돌릴 때는 위치와 방향 오차를 함께 확인합니다."""
    poses = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
    if len(poses) == 0:
        raise ValueError('poses cannot be empty')
    vehicle_pose = np.asarray(
        vehicle_pose, dtype=np.float64
    ).reshape(3)
    position_error = float(np.hypot(
        poses[0, 0] - vehicle_pose[0],
        poses[0, 1] - vehicle_pose[1],
    ))
    yaw_error = abs(normalize_angle(poses[0, 2] - vehicle_pose[2]))
    return position_error, yaw_error


def direction_change_index(directions, start_index, current_direction):
    """기어가 처음 바뀌는 표본을 찾으며 바뀌지 않으면 마지막 표본을 반환합니다."""
    directions = np.asarray(directions, dtype=np.int8).reshape(-1)
    if len(directions) == 0:
        raise ValueError('directions cannot be empty')
    start_index = int(np.clip(start_index, 0, len(directions) - 1))
    current_direction = 1 if current_direction >= 0 else -1
    changes = np.flatnonzero(
        directions[start_index:] != current_direction
    )
    if len(changes):
        return start_index + int(changes[0])
    return len(directions) - 1


def path_distance(poses, start_index, end_index):
    """순서가 정해진 두 경로 표본 사이의 누적 이동 거리를 계산합니다."""
    poses = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
    start_index = int(np.clip(start_index, 0, len(poses) - 1))
    end_index = int(np.clip(end_index, start_index, len(poses) - 1))
    if end_index <= start_index:
        return 0.0
    steps = np.diff(poses[start_index:end_index + 1, :2], axis=0)
    return float(np.hypot(steps[:, 0], steps[:, 1]).sum())


def lookahead_index(poses, start_index, end_index, distance):
    """지정 거리 이상 앞선 첫 경로 표본을 찾습니다."""
    poses = np.asarray(poses, dtype=np.float64).reshape(-1, 3)
    start_index = int(np.clip(start_index, 0, len(poses) - 1))
    end_index = int(np.clip(end_index, start_index, len(poses) - 1))
    target_distance = max(0.0, float(distance))
    accumulated = 0.0
    for index in range(start_index + 1, end_index + 1):
        accumulated += float(np.hypot(
            poses[index, 0] - poses[index - 1, 0],
            poses[index, 1] - poses[index - 1, 1],
        ))
        if accumulated >= target_distance:
            return index
    return end_index


def filter_vehicle_self_points(
        points,
        vehicle_length,
        vehicle_width,
        rear_overhang,
        padding=0.04):
    """차체 사각 영역 내부의 센서 반사점을 제거합니다."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 2)
    if len(points) == 0:
        return points
    rear = -float(rear_overhang) - float(padding)
    front = float(vehicle_length) - float(rear_overhang) + float(padding)
    half_width = 0.5 * float(vehicle_width) + float(padding)
    is_self = (
        (points[:, 0] >= rear)
        & (points[:, 0] <= front)
        & (np.abs(points[:, 1]) <= half_width)
    )
    return points[~is_self]


@dataclass
class DWAResult:
    """기준 경로를 따라 선택한 지역 추종 궤적과 조향 결과입니다."""

    success: bool
    reason: str
    trajectory: np.ndarray
    steer: float
    score: float
    clearance: float
    candidates: tuple
    valid_candidates: tuple


class PathTrackingDWA:
    """SLAM의 base_link 좌표계에서 기존 DWA 추종 계산을 수행합니다."""

    def __init__(self, vehicle=None):
        """DWA 계산에 사용할 차량 제원과 추종 파라미터를 설정합니다."""
        self.vehicle = vehicle or Vehicle()
        self.wheel_base = self.vehicle.wheel_base  # 공통 차량 축거(m)
        self.max_steer = self.vehicle.max_steer  # 공통 조향 한계(rad)
        self.dwa_count = 13  # 현재 조향 주변에서 생성할 후보 수
        self.step = 0.13  # 예측 궤적의 표본 간격(m)
        self.prediction_distance = 0.7  # 지역 궤적을 예측할 거리(m)

        # 충돌 검사는 공통 차량 사각형과 collision_margin을 함께 사용합니다.
        self.vehicle_length = self.vehicle.length  # 공통 차체 길이(m)
        self.vehicle_width = self.vehicle.width  # 공통 차체 폭(m)
        self.rear_overhang = self.vehicle.rear_overhang  # 후륜축부터 뒤 범퍼 거리(m)
        self.safe_dist = 0.23  # 장애물 근접 비용을 적용할 여유 거리(m)

        # 모든 항을 "작을수록 좋은 비용"으로 통일 -> 가중치 전부 양수.
        self.w1 = 0.1   # 경로 방향 오차의 비용 가중치
        self.w2 = 5.0   # 중간점 장애물 근접 비용 가중치
        self.w3 = 4.0   # 끝점 장애물 근접 비용 가중치
        self.w4 = 3.0   # 전방 목표점과의 거리 비용 가중치

    def validate(self):
        """차량 제원과 추종 파라미터의 유효 범위를 확인합니다."""
        positive = {
            'wheel_base': self.wheel_base,
            'max_steer': self.max_steer,
            'dwa_count': self.dwa_count,
            'step': self.step,
            'prediction_distance': self.prediction_distance,
            'vehicle_length': self.vehicle_length,
            'vehicle_width': self.vehicle_width,
            'safe_dist': self.safe_dist,
        }
        invalid = [name for name, value in positive.items() if value <= 0]
        if invalid:
            raise ValueError(
                f'DWA parameters must be positive: {", ".join(invalid)}'
            )
        if self.max_steer >= math.pi / 2.0:
            raise ValueError('max_steer must be less than pi/2')
        if self.dwa_count < 3:
            raise ValueError('dwa_count must be at least 3')
        if self.rear_overhang < 0.0:
            raise ValueError('rear_overhang cannot be negative')
        weights = (self.w1, self.w2, self.w3, self.w4)
        if any(weight < 0.0 for weight in weights):
            raise ValueError('DWA cost weights cannot be negative')

    def dynamic_window(self, steer=0.0):
        """기존 kmu/dwa.py와 같이 현재 조향을 중심으로 후보 범위를 만듭니다."""
        current = float(np.clip(
            steer, -self.max_steer, self.max_steer
        ))
        dwa_step = 2.0 * self.max_steer / (int(self.dwa_count) - 1)
        dwa_steer = current - self.max_steer + dwa_step * np.arange(
            int(self.dwa_count), dtype=np.float64
        )
        return dwa_steer[
            np.abs(dwa_steer) <= self.max_steer + 1.0e-12
        ]

    def rollout(self, steer, direction, distance=None):
        """일정 조향의 자전거 모델 궤적을 base_link 좌표계에서 예측합니다."""
        direction = 1.0 if direction >= 0 else -1.0
        if distance is None:
            distance = self.prediction_distance
        distance = max(float(distance), self.step)
        count = max(2, int(math.ceil(distance / self.step)) + 1)
        arc = direction * np.linspace(0.0, distance, count)
        curvature = math.tan(float(steer)) / self.wheel_base
        trajectory = np.zeros((count, 3), dtype=np.float64)
        if abs(curvature) < 1.0e-9:
            trajectory[:, 0] = arc
        else:
            yaw = curvature * arc
            trajectory[:, 0] = np.sin(yaw) / curvature
            trajectory[:, 1] = (1.0 - np.cos(yaw)) / curvature
            trajectory[:, 2] = yaw
        return trajectory


    def trajectory_clearance(self, trajectory, obstacles):
        return footprint_clearance(trajectory, obstacles, self.vehicle)

    def predict_path(self, steer=0.0, direction=1, distance=None):
        """동적 조향 범위의 모든 후보 궤적을 생성합니다."""
        all_path = []
        for dwa_steer in self.dynamic_window(steer):
            path = self.rollout(float(dwa_steer), direction, distance)
            all_path.append((float(dwa_steer), path))
        return all_path

    def _obstacle_cost(self, clearance):
        """safe_dist 여유를 침범한 비율로 장애물 비용을 계산합니다."""
        if not math.isfinite(clearance) or clearance >= self.safe_dist:
            return 0.0
        return (self.safe_dist - float(clearance)) / self.safe_dist

    def cost_function(
            self, point, target, obs, obs_weight, clearance=None):
        """예측 표본 하나의 방향 오차와 장애물 비용을 합산합니다."""
        point = np.asarray(point, dtype=np.float64).reshape(1, 3)
        target = np.asarray(target, dtype=np.float64).reshape(3)
        if clearance is None:
            clearance = self.trajectory_clearance(point, obs)
        heading_cost = abs(normalize_angle(point[0, 2] - target[2]))
        obs_cost = self._obstacle_cost(clearance)
        return self.w1 * heading_cost + obs_weight * obs_cost

    def dwa(
            self,
            target,
            obs,
            direction,
            steer=0.0,
            prediction_distance=None):
        """기존 추종 비용과 전방 목표 거리로 최종 후보 궤적을 선택합니다."""
        self.validate()
        target = np.asarray(target, dtype=np.float64).reshape(3)
        obs = np.asarray(obs, dtype=np.float64).reshape(-1, 2)

        best = None
        candidates = []
        valid_candidates = []
        all_path = self.predict_path(steer, direction, prediction_distance)
        for dwa_steer, trajectory in all_path:
            candidates.append(trajectory)

            # 중간·끝점 비용은 유지하며 충돌은 예측 궤적 전체에서 확인합니다.
            middle = trajectory[len(trajectory) // 2]
            endpoint = trajectory[-1]
            middle_clearance = self.trajectory_clearance(
                middle[None, :], obs
            )
            endpoint_clearance = self.trajectory_clearance(
                endpoint[None, :], obs
            )
            clearance = self.trajectory_clearance(trajectory, obs)
            if clearance < 0.0:
                valid_candidates.append(False)
                continue

            valid_candidates.append(True)
            total_cost = self.cost_function(
                middle,
                target,
                obs,
                self.w2,
                clearance=middle_clearance,
            ) + self.cost_function(
                endpoint,
                target,
                obs,
                self.w3,
                clearance=endpoint_clearance,
            )

            # 후보 끝점이 SLAM lookahead 점에 가까워지도록 유도한다.
            lookahead_cost = float(np.linalg.norm(
                endpoint[:2] - target[:2]
            ))
            total_cost += self.w4 * lookahead_cost

            if best is None or total_cost < best[0]:
                best = (total_cost, trajectory, dwa_steer, clearance)

        if best is None:
            return DWAResult(
                False,
                'no collision-free path-tracking rollout',
                np.empty((0, 3)),
                0.0,
                math.inf,
                -math.inf,
                tuple(candidates),
                tuple(valid_candidates),
            )
        return DWAResult(
            True,
            'selected',
            best[1],
            best[2],
            best[0],
            best[3],
            tuple(candidates),
            tuple(valid_candidates),
        )
