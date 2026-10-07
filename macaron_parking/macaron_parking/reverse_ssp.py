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

"""Shin Stanley Pursuit(SSP) 수식으로 저속 후진 경로를 추종합니다."""

import math
from dataclasses import dataclass

import numpy as np
from .vehicle import Vehicle


def normalize_angle(angle):
    """각도를 [-pi, pi] 범위로 정규화합니다."""
    return math.atan2(math.sin(angle), math.cos(angle))


def direction_segment_start(directions, index):
    """지정 표본이 속한 동일 기어 구간의 첫 표본을 찾습니다."""
    directions = np.asarray(directions, dtype=np.int8).reshape(-1)
    if len(directions) == 0:
        raise ValueError('directions cannot be empty')
    index = int(np.clip(index, 0, len(directions) - 1))
    direction = 1 if directions[index] >= 0 else -1
    start = index
    while start > 0:
        previous = 1 if directions[start - 1] >= 0 else -1
        if previous != direction:
            break
        start -= 1
    return start


@dataclass(frozen=True)
class ReverseSSPResult:
    """지도 좌표계에서 계산한 후진 SSP 조향·예측 결과입니다."""

    steer: float
    heading_error: float
    cross_track_error: float
    reference: np.ndarray
    rear_projection: np.ndarray
    virtual_front: np.ndarray
    front_projection: np.ndarray

    @property
    def tracking_point_at_endpoint(self):
        """가상 전륜의 경로 투영점이 마지막 구간에 도달했는지 확인합니다."""
        endpoint = np.asarray(self.reference[-1, :2], dtype=np.float64)
        projection = np.asarray(self.front_projection, dtype=np.float64)
        return bool(np.allclose(
            projection, endpoint, rtol=0.0, atol=1.0e-9
        ))


class ReverseSSP:
    """Hybrid A*의 후진 구간 하나를 SSP로 추종합니다.
    
    후진을 가상 차량의 전진 자전거 모델로 표현합니다. 가상 후륜축은
    실제 후륜축 중심인 base_link이며 방향은 차체 yaw + pi입니다.
    가상 전륜축은 후진 방향으로 축거만큼 떨어진 점입니다.
    가상 후륜의 경로 방향 오차와 가상 전륜의 횡방향 오차를 사용하며,
    가상 조향 부호를 반전하여 실제 후진 조향을 구합니다.
    
    차량 제원은 공통 macaron 모델을 사용합니다. 유효 속도는
    Stanley 계산용 보정값이며 이 클래스는 모터 명령을 발행하지 않습니다."""

    def __init__(self, vehicle=None):
        """차량 제원과 저속 SSP 추종 보정값을 설정합니다."""
        self.vehicle = vehicle or Vehicle()
        self.wheel_base = self.vehicle.wheel_base  # 공통 차량 축거(m)
        self.max_steer = self.vehicle.max_steer  # 공통 조향 한계(rad)

        # 기존 오프라인 추종 검증에서 사용한 저속 SSP 보정값입니다.
        self.gain = 1.17  # 횡방향 오차를 조향으로 보정하는 이득
        self.effective_speed = 0.20  # SSP 계산용 속도(m/s); 추종부에서 실제 설정값 적용
        self.speed_softening = 0.1  # 저속 조향 계산의 분모를 완화하는 속도(m/s)

        # 끝점에서도 추종 기준이 유지되도록 마지막 경로의 접선 방향을 연장합니다.
        self.endpoint_extension = 0.45  # 끝점 방향을 유지하도록 연장할 거리(m)
        self.preview_distance = 0.50  # 후진 표시·충돌 검사 궤적 길이(m)
        self.preview_spacing = 0.05  # 후진 예측 궤적의 표본 간격(m)
        self.validate()

    def validate(self):
        """차량 제원과 저속 추종 보정값의 유효 범위를 확인합니다."""
        positive = {
            'wheel_base': self.wheel_base,
            'max_steer': self.max_steer,
            'gain': self.gain,
            'effective_speed': self.effective_speed,
            'speed_softening': self.speed_softening,
            'endpoint_extension': self.endpoint_extension,
            'preview_distance': self.preview_distance,
            'preview_spacing': self.preview_spacing,
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            names = ', '.join(invalid)
            raise ValueError(
                f'reverse SSP parameters must be positive: {names}'
            )
        if self.max_steer >= math.pi / 2.0:
            raise ValueError('max_steer must be less than pi/2')

    def _extended_reference(self, reference):
        """마지막 후진 진행 방향으로 경로 표본 하나를 연장합니다."""
        reference = np.asarray(reference, dtype=np.float64).reshape(-1, 3)
        if len(reference) == 0:
            raise ValueError('reverse reference cannot be empty')
        endpoint = reference[-1]
        motion_yaw = normalize_angle(endpoint[2] + math.pi)
        extension = np.array([
            endpoint[0] + self.endpoint_extension * math.cos(motion_yaw),
            endpoint[1] + self.endpoint_extension * math.sin(motion_yaw),
            endpoint[2],
        ])
        return np.vstack((reference, extension))

    @staticmethod
    def _project(point, reference):
        """지도 점을 후진 경로에 투영하고 해당 방향을 보간합니다."""
        point = np.asarray(point, dtype=np.float64).reshape(2)
        reference = np.asarray(reference, dtype=np.float64).reshape(-1, 3)
        if len(reference) < 2:
            raise ValueError('reverse reference needs at least two poses')

        starts = reference[:-1, :2]
        vectors = reference[1:, :2] - starts
        lengths_squared = np.sum(vectors * vectors, axis=1)
        valid = lengths_squared > 1.0e-12
        if not np.any(valid):
            body_yaw = float(reference[0, 2])
            return reference[0, :2].copy(), normalize_angle(
                body_yaw + math.pi
            )

        factors = np.zeros(len(vectors), dtype=np.float64)
        factors[valid] = (
            np.sum((point - starts[valid]) * vectors[valid], axis=1)
            / lengths_squared[valid]
        )
        factors = np.clip(factors, 0.0, 1.0)
        projections = starts + factors[:, None] * vectors
        distances_squared = np.sum((point - projections) ** 2, axis=1)
        distances_squared[~valid] = math.inf
        index = int(np.argmin(distances_squared))

        yaw_delta = normalize_angle(
            float(reference[index + 1, 2] - reference[index, 2])
        )
        body_yaw = normalize_angle(
            float(reference[index, 2]) + factors[index] * yaw_delta
        )
        return projections[index], normalize_angle(body_yaw + math.pi)

    def track(self, reference, vehicle_pose):
        """지도 기준 차량 위치에서 후진 SSP 조향각을 계산합니다."""
        self.validate()
        vehicle_pose = np.asarray(vehicle_pose, dtype=np.float64).reshape(3)
        extended = self._extended_reference(reference)

        rear = vehicle_pose[:2]
        virtual_yaw = normalize_angle(float(vehicle_pose[2]) + math.pi)
        rear_projection, rear_path_yaw = self._project(rear, extended)
        heading_error = normalize_angle(rear_path_yaw - virtual_yaw)

        virtual_front = rear + self.wheel_base * np.array([
            math.cos(virtual_yaw), math.sin(virtual_yaw)
        ])
        front_projection, front_path_yaw = self._project(
            virtual_front, extended
        )
        left_normal = np.array([
            -math.sin(front_path_yaw), math.cos(front_path_yaw)
        ])
        cross_track_error = float(np.dot(
            virtual_front - front_projection, left_normal
        ))

        virtual_steer = (
            heading_error
            - math.atan2(
                self.gain * cross_track_error,
                self.effective_speed + self.speed_softening,
            )
        )
        reverse_steer = float(np.clip(
            -virtual_steer, -self.max_steer, self.max_steer
        ))
        return ReverseSSPResult(
            reverse_steer,
            heading_error,
            cross_track_error,
            extended,
            rear_projection,
            virtual_front,
            front_projection,
        )

    def predict(self, steer, distance=None):
        """RViz 표시용으로 일정 조향의 후진 궤적을 예측합니다."""
        if distance is None:
            distance = self.preview_distance
        distance = max(float(distance), self.preview_spacing)
        count = max(2, int(math.ceil(distance / self.preview_spacing)) + 1)
        arc = -np.linspace(0.0, distance, count)
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
