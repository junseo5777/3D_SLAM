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


"""KMU Hybrid A* 수치 계산부에 PCD 직접 충돌 검사를 연결합니다.

주차 경로 계산에 필요한 탐색·운동 모델·연결 비용만 포함합니다."""
import heapq
import math
import time
import numpy as np
from .vehicle import Vehicle

_QUAD_NODES, _QUAD_WEIGHTS = np.polynomial.legendre.leggauss(24)
_QUAD_T = 0.5 * (_QUAD_NODES + 1.0)
_QUAD_W = 0.5 * _QUAD_WEIGHTS
_QUAD_SHAPE = _QUAD_T ** 2 - _QUAD_T

def normalize_angle(angle):
    """각도를 [-pi, pi]로 감는다."""
    return math.atan2(math.sin(angle), math.cos(angle))


class Clothoid:
    """곡률이 호길이에 대해 선형으로 변하는 오일러 나선 한 조각."""

    def __init__(self, x, y, yaw, curvature, curvature_rate, length):
        """시작 pose와 곡률 파라미터로 클로소이드를 정의한다."""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.curvature = curvature
        self.curvature_rate = curvature_rate
        self.length = length

    @property
    def peak_curvature(self):
        """구간 양 끝 중 더 큰 곡률 크기. 곡률이 선형이라 끝점만 보면 된다."""
        end_curvature = self.curvature + self.curvature_rate * self.length
        return max(abs(self.curvature), abs(end_curvature))

    def sample(self, spacing):
        """일정 간격으로 (x, y, yaw)를 뽑아 (N, 3) 배열로 돌려준다."""
        count = max(2, int(math.ceil(self.length / max(spacing, 1e-6))) + 1)
        arc = np.linspace(0.0, self.length, count)
        yaw = (
            self.yaw
            + self.curvature * arc
            + 0.5 * self.curvature_rate * arc ** 2
        )
        cosine = np.cos(yaw)
        sine = np.sin(yaw)
        # 누적 사다리꼴 적분. 표본 간격이 촘촘해 끝점 오차는 1e-4 m 수준이다.
        step = np.diff(arc)
        x = self.x + np.concatenate((
            [0.0], np.cumsum(0.5 * (cosine[:-1] + cosine[1:]) * step)
        ))
        y = self.y + np.concatenate((
            [0.0], np.cumsum(0.5 * (sine[:-1] + sine[1:]) * step)
        ))
        return np.column_stack((x, y, yaw))


def _clothoid_angle(sharpness, base):
    """
    구적점마다의 클로소이드 헤딩 (n, 24) 배열.

    ``theta0 + (delta - A) t + A t^2 == base + A (t^2 - t)`` 로 묶으면 A와
    무관한 부분(``base``)을 한 번만 계산하면 되고, 근을 찾는 반복에서는
    곱셈과 덧셈 한 번씩만 남는다. 이 함수가 탐색 전체의 가장 뜨거운 지점이라
    numpy 호출 수를 최소로 줄여 놓았다.
    """
    return base + np.asarray(sharpness)[:, np.newaxis] * _QUAD_SHAPE


def _clothoid_sine(sharpness, base):
    """도착점의 횡방향 오차에 해당하는 sin 적분. 근을 찾을 때 쓴다."""
    return np.sin(_clothoid_angle(sharpness, base)) @ _QUAD_W


def _clothoid_cosine(sharpness, base):
    """현(chord) 방향 성분에 해당하는 cos 적분. 길이를 낼 때 쓴다."""
    return np.cos(_clothoid_angle(sharpness, base)) @ _QUAD_W


def _refine_roots(low, high, base, iterations=14):
    """
    부호가 바뀌는 구간들을 이분법으로 동시에 좁힌다.

    반복 횟수는 A를 1e-5 수준까지 좁히면 충분해서 정했다. 이 오차가 경로에
    남기는 위치 오차는 마이크로미터 단위이고, 남는 잔차는 _connect가 끝점에
    맞춰 흡수한다.
    """
    low = np.array(low, dtype=np.float64)
    high = np.array(high, dtype=np.float64)
    sine_low = _clothoid_sine(low, base)
    for _ in range(iterations):
        middle = 0.5 * (low + high)
        sine_middle = _clothoid_sine(middle, base)
        same_side = np.signbit(sine_middle) == np.signbit(sine_low)
        low = np.where(same_side, middle, low)
        sine_low = np.where(same_side, sine_middle, sine_low)
        high = np.where(same_side, high, middle)
    return 0.5 * (low + high)


def fit_clothoid(start, goal, max_curvature=None, span=4.0 * math.pi,
                 samples=161):
    """
    두 pose를 하나의 클로소이드로 잇는 G1 Hermite 보간.

    Bertolazzi & Frego(2015)의 표준화를 따른다. 헤딩을
    ``theta(t) = theta0 + (delta - A) t + A t^2`` 로 두면 도착점의 횡방향
    오차가 0이 되는 조건이 ``A``에 대한 1차원 방정식이 되고, 그 근을 찾으면
    곡률과 길이가 바로 나온다.

    전진 방향으로만 이어지므로 (길이가 양수) 후진 연결은 두 pose의 yaw를
    각각 pi만큼 돌려서 호출하면 된다.

    맞는 클로소이드가 없거나 ``max_curvature``를 넘으면 None을 준다.
    """
    x0, y0, yaw0 = start
    x1, y1, yaw1 = goal
    delta_x = x1 - x0
    delta_y = y1 - y0
    chord = math.hypot(delta_x, delta_y)
    if chord < 1.0e-9:
        return None

    heading = math.atan2(delta_y, delta_x)
    theta0 = normalize_angle(yaw0 - heading)
    theta1 = normalize_angle(yaw1 - heading)
    delta = theta1 - theta0

    base = theta0 + delta * _QUAD_T
    grid = np.linspace(-span, span, samples)
    sine = _clothoid_sine(grid, base)
    # 근이 표본점 위에 정확히 걸리면 이분법이 무너지므로 따로 골라낸다.
    sign = np.sign(sine)
    sign[np.abs(sine) <= 1.0e-12] = 0.0
    exact = grid[sign == 0.0]
    brackets = np.flatnonzero(sign[:-1] * sign[1:] < 0.0)
    if len(brackets) == 0 and len(exact) == 0:
        return None

    roots = [exact]
    if len(brackets):
        roots.append(
            _refine_roots(grid[brackets], grid[brackets + 1], base)
        )
    roots = np.concatenate(roots)
    cosine = _clothoid_cosine(roots, base)

    best = None
    best_peak = math.inf
    for sharpness, cosine_value in zip(roots, cosine):
        if cosine_value <= 1.0e-9:
            continue  # 길이가 음수 -> 전진으로는 못 잇는 해
        length = chord / float(cosine_value)
        curvature = (delta - sharpness) / length
        curvature_rate = 2.0 * sharpness / (length * length)
        peak = max(abs(curvature), abs((delta + sharpness) / length))
        if max_curvature is not None and peak > max_curvature:
            continue
        if peak < best_peak:
            best_peak = peak
            best = Clothoid(
                x0, y0, yaw0, curvature, curvature_rate, length
            )
    return best


class _Node:
    """트리의 한 상태. 부모로부터의 1스텝 정보를 함께 들고 있다."""

    __slots__ = (
        'x', 'y', 'yaw', 'steer', 'direction', 'arc', 'cost', 'parent',
        'phase', 'reverse_distance',
    )

    def __init__(self, x, y, yaw, steer, direction, arc, cost, parent,
                 phase=0, reverse_distance=0.0):
        """pose와 도달 비용, 그리고 부모에서 오는 호를 기록한다."""
        self.x = x
        self.y = y
        self.yaw = yaw
        self.steer = steer
        self.direction = direction
        self.arc = arc
        self.cost = cost
        self.parent = parent
        self.phase = phase
        self.reverse_distance = reverse_distance

    @property
    def pose(self):
        """(x, y, yaw) 튜플."""
        return (self.x, self.y, self.yaw)


class _Tree:
    """한쪽 방향의 탐색 트리."""

    def __init__(self, root, growth, heuristic):
        """루트 노드, 시간 진행 방향(+1/-1), 휴리스틱을 묶는다."""
        self.root = root
        self.growth = growth
        self.heuristic = heuristic
        self.nodes = {}
        self.closed = set()
        self.open = []
        self.counter = 0
        self.mate = None


class PlanResult:
    """플래너 실행 결과."""

    def __init__(self, success, poses=None, directions=None, reason='',
                 expansions=0, elapsed=0.0, cost=math.inf):
        """성공 여부와 경로, 통계를 담는다."""
        self.success = success
        self.poses = (
            np.zeros((0, 3)) if poses is None else np.asarray(poses)
        )
        self.directions = (
            np.zeros(0) if directions is None else np.asarray(directions)
        )
        self.reason = reason
        self.expansions = expansions
        self.elapsed = elapsed
        self.cost = cost

    @property
    def length(self):
        """경로의 총 주행 거리(m)."""
        if len(self.poses) < 2:
            return 0.0
        steps = np.diff(self.poses[:, :2], axis=0)
        return float(np.hypot(steps[:, 0], steps[:, 1]).sum())

    @property
    def gear_changes(self):
        """전/후진이 바뀌는 횟수."""
        if len(self.directions) < 2:
            return 0
        return int(np.count_nonzero(np.diff(self.directions) != 0))

    def as_tuples(self):
        """[(x, y, yaw, direction), ...] 형태로 변환한다."""
        return [
            (float(pose[0]), float(pose[1]), float(pose[2]), float(gear))
            for pose, gear in zip(self.poses, self.directions)
        ]

    def __repr__(self):
        """디버깅용 한 줄 요약."""
        if not self.success:
            return f'<PlanResult failed: {self.reason}>'
        return (
            f'<PlanResult {len(self.poses)} poses, {self.length:.2f} m, '
            f'gear changes {self.gear_changes}, '
            f'{self.expansions} expansions, {self.elapsed:.2f} s>'
        )


class _DistanceHeuristic:
    def __init__(self, cloud_map, target):
        self.target = target

    def __call__(self, x, y):
        return math.hypot(x - self.target[0], y - self.target[1])


class HybridAStar:
    """전후진 운동과 클로소이드 연결을 사용하는 양방향 가중치 탐색입니다."""

    def __init__(self, collision_map, vehicle=None):
        """지도를 물리고 탐색/비용 파라미터 기본값을 세운다."""
        self.map = collision_map
        self.vehicle = vehicle or collision_map.vehicle

        # 차량 기구학. dwa.py와 같은 자전거 모델을 쓴다.
        self.wheel_base = self.vehicle.wheel_base     # 축간 거리(m)
        self.max_steer = self.vehicle.max_steer     # 최대 조향각(rad)
        self.steer_count = 9               # 1스텝에서 시도할 조향 후보 수
        self.step = 0.25                   # 1스텝의 호 길이(m)
        self.substeps = 5                  # 한 스텝 안에서 충돌을 볼 표본 수
        self.allow_reverse = True          # 후진 스텝 허용 여부

        # 상태 격자. 같은 칸에 들어온 상태 중 싼 것만 남긴다.
        self.xy_resolution = 0.15          # 상태 중복 판정용 위치 격자(m)
        self.yaw_resolution = math.radians(5.0)  # 상태 중복 판정용 각도 격자

        # 비용 가중치. 전부 "작을수록 좋은 비용"이라 부호가 모두 양수다.
        self.w_length = 1.0        # 주행 거리
        self.w_steer = 0.35        # 조향 크기 (dwa.py의 w1 heading 항 대응)
        self.w_steer_change = 0.5  # 조향 변화량 -> 경로를 매끄럽게
        self.w_obstacle = 1.5      # 장애물 근접 페널티 (dwa.py의 w2/w3 대응)
        self.w_reverse = 1.7       # 후진 1 m당 추가 비용
        self.w_gear_change = 2.0   # 전/후진 전환 1회 비용
        self.safe_dist = 0.3      # 이 여유보다 가까우면 페널티 (dwa.py와 동일)
        self.collision_margin = self.vehicle.margin  # 충돌 판정에 더 얹는 여유(m)

        # 탐색 제어.
        self.heuristic_weight = 1.4     # >1이면 weighted A*, 빠르지만 준최적
        self.max_expansions = 40000     # 확장 상한
        self.time_limit = 20.0          # 탐색 시간 상한(s)

        # 클로소이드 연결.
        self.connect_interval = 5       # 몇 번 확장마다 반대편 루트로 쏴볼지
        self.connect_radius = 2.0       # 이 거리 안이면 매번 시도(m)
        self.connect_max_length = 5.0   # 연결 클로소이드 최대 길이(m)
        self.connect_candidates = 6     # 반대 트리에서 시도할 최대 노드 수
        self.connect_cell_span = 1      # 이웃 상태 칸 탐색 반경(칸)

        # 출력.
        self.output_spacing = 0.05      # 최종 경로 표본 간격(m)

        self._prepare()

    def _prepare(self):
        """
        탐색 직전에 파라미터에서 파생되는 배열을 만들어 둔다.

        가중치나 조향 개수를 생성 뒤에 바꿔도 plan()이 그 값을 쓰도록,
        파생값 계산을 생성자에 묶지 않고 매 plan()마다 다시 한다.
        """
        self._steers = np.asarray(self.steer_candidates(), dtype=np.float64)
        self._curvatures = np.tan(self._steers) / self.wheel_base
        self._gears = np.array(
            [1.0, -1.0] if self.allow_reverse else [1.0], dtype=np.float64
        )
        count = max(1, int(self.substeps))
        self._fractions = np.arange(1, count + 1, dtype=np.float64) / count

    @property
    def max_curvature(self):
        """조향 한계에서 나오는 최대 곡률(1/m)."""
        return math.tan(self.max_steer) / self.wheel_base

    def steer_candidates(self):
        """
        조향 후보를 균등하게 만든다. dwa.py dynamic_window의 전역 버전이다.

        DWA는 현재 조향 주변만 훑지만, global path는 매 스텝에서 전 범위를
        볼 수 있어야 좁은 주차 통로를 돌아나갈 수 있다. 대신 조향 변화량에
        비용(w_steer_change)을 매겨 경로가 튀는 것을 막는다.
        """
        if self.steer_count < 2:
            return [0.0]
        step = 2.0 * self.max_steer / (self.steer_count - 1)
        return [-self.max_steer + step * index
                for index in range(self.steer_count)]

    def _propagate(self, x, y, yaw, steer, arc):
        """자전거 모델을 호 길이 ``arc``만큼 정확히 적분한다. 음수면 후진."""
        if abs(steer) < 1.0e-6:
            return x + arc * math.cos(yaw), y + arc * math.sin(yaw), yaw
        curvature = math.tan(steer) / self.wheel_base
        next_yaw = yaw + curvature * arc
        next_x = x + (math.sin(next_yaw) - math.sin(yaw)) / curvature
        next_y = y - (math.cos(next_yaw) - math.cos(yaw)) / curvature
        return next_x, next_y, next_yaw

    def _state_index(self, x, y, yaw):
        """상태 중복 판정용 SE(2) 격자 인덱스."""
        return (
            int(math.floor(x / self.xy_resolution)),
            int(math.floor(y / self.xy_resolution)),
            int(math.floor(normalize_angle(yaw) / self.yaw_resolution)),
        )

    def _footprint_margin(self, x, y, yaw):
        return self.map.clearance((x, y, yaw))

    def _obstacle_penalty(self, margin):
        """DWA와 같은 방식으로 근접 페널티를 매긴다."""
        if margin >= self.safe_dist:
            return 0.0
        return 1.0 / max(margin, 0.02)

    def _path_is_free(self, poses):
        return self.map.path_is_free(poses)

    def plan(self, start, goal, cancel_check=None):
        """
        start에서 goal까지의 global path를 찾는다.

        ``start``/``goal``은 map frame의 (x, y, yaw)다. 반환값은 PlanResult로,
        ``poses``가 (N, 3) 배열, ``directions[i]``가 pose i에서 pose i+1로 갈 때
        써야 할 기어(+1 전진, -1 후진)다. 마지막 원소는 직전 값을 반복한다.
        cancel_check가 참이면 변경된 입력의 이전 탐색을 중단한다.
        """
        started_at = time.monotonic()
        if cancel_check is not None and cancel_check():
            return PlanResult(False, reason='planning cancelled')
        self._prepare()

        for name, pose in (('start', start), ('goal', goal)):
            if self._footprint_margin(*pose) < 0.0:
                return PlanResult(
                    False,
                    reason=f'{name} pose is in collision or outside the map',
                    elapsed=time.monotonic() - started_at,
                )

        if (math.hypot(start[0] - goal[0], start[1] - goal[1]) < 1e-6
                and abs(normalize_angle(start[2] - goal[2])) < 1e-6):
            return PlanResult(True, [start], [1], reason="already_at_goal")

        # 두 pose가 이미 한 번에 이어지면 탐색할 것도 없다.
        direct = self._connect(start, goal)
        if direct is not None:
            poses, directions = direct
            steps = np.diff(poses[:, :2], axis=0)
            return PlanResult(
                True, poses, directions,
                reason='single clothoid',
                elapsed=time.monotonic() - started_at,
                cost=float(np.hypot(steps[:, 0], steps[:, 1]).sum()),
            )

        forward_tree = _Tree(
            _Node(start[0], start[1], start[2], 0.0, 1.0, 0.0, 0.0, None),
            1.0,
            _DistanceHeuristic(self.map, goal),
        )
        reverse_tree = _Tree(
            _Node(goal[0], goal[1], goal[2], 0.0, 1.0, 0.0, 0.0, None),
            -1.0,
            _DistanceHeuristic(self.map, start),
        )
        forward_tree.mate = reverse_tree
        reverse_tree.mate = forward_tree
        for tree in (forward_tree, reverse_tree):
            index = self._state_index(*tree.root.pose)
            tree.nodes[index] = tree.root
            heapq.heappush(tree.open, (0.0, 0, index))

        expansions = 0
        deadline = started_at + self.time_limit
        while forward_tree.open and reverse_tree.open:
            if cancel_check is not None and cancel_check():
                return PlanResult(False, reason='planning cancelled', expansions=expansions,
                                  elapsed=time.monotonic() - started_at)
            if expansions >= self.max_expansions:
                return PlanResult(
                    False, reason='expansion limit reached',
                    expansions=expansions,
                    elapsed=time.monotonic() - started_at,
                )
            if time.monotonic() > deadline:
                return PlanResult(
                    False, reason='time limit reached',
                    expansions=expansions,
                    elapsed=time.monotonic() - started_at,
                )

            # 열린 목록의 f값이 작은 쪽을 번갈아 키운다.
            tree = (
                forward_tree
                if forward_tree.open[0][0] <= reverse_tree.open[0][0]
                else reverse_tree
            )
            _, _, index = heapq.heappop(tree.open)
            if index in tree.closed:
                continue
            node = tree.nodes.get(index)
            if node is None:
                continue
            tree.closed.add(index)

            expansions += 1
            joined = self._try_join(tree, node, expansions)
            if joined is not None:
                poses, directions, cost = joined
                return PlanResult(
                    True, poses, directions,
                    reason='clothoid join',
                    expansions=expansions,
                    elapsed=time.monotonic() - started_at,
                    cost=cost,
                )
            self._expand(tree, node)

        return PlanResult(
            False, reason='search space exhausted',
            expansions=expansions,
            elapsed=time.monotonic() - started_at,
        )

    def _sweep(self, node, growth):
        """
        조향 x 기어 후보를 한꺼번에 1스텝 뻗는다.

        DWA가 후보 경로를 한 번에 펼쳐 보는 것과 같은 발상인데, 여기서는
        중간 표본의 발자국 충돌까지 numpy 한 번에 계산한다. 파이썬 반복문으로
        하면 확장 하나에 수십 번씩 지도를 조회하게 되어 너무 느리다.

        (기어, 조향)마다의 도착 pose와 구간 최소 여유를 돌려준다.
        """
        arcs = self._gears * (self.step * growth)
        partial = arcs[:, np.newaxis, np.newaxis] * self._fractions
        curvature = self._curvatures[np.newaxis, :, np.newaxis]
        straight = np.abs(curvature) < 1.0e-6
        divisor = np.where(straight, 1.0, curvature)

        cos_yaw = math.cos(node.yaw)
        sin_yaw = math.sin(node.yaw)
        yaw = node.yaw + curvature * partial
        x = node.x + np.where(
            straight, partial * cos_yaw, (np.sin(yaw) - sin_yaw) / divisor
        )
        y = node.y + np.where(
            straight, partial * sin_yaw, -(np.cos(yaw) - cos_yaw) / divisor
        )

        poses = np.stack(np.broadcast_arrays(x, y, yaw), axis=-1)
        margins = self.map.margins(poses)
        return x[:, :, -1], y[:, :, -1], yaw[:, :, -1], margins.min(axis=2)

    def _expand(self, tree, node):
        """한 노드에서 뻗은 후보들을 비용순으로 트리에 넣는다."""
        end_x, end_y, end_yaw, worst = self._sweep(node, tree.growth)
        for gear_index, direction in enumerate(self._gears):
            arc = float(direction) * self.step * tree.growth
            for steer_index, steer in enumerate(self._steers):
                margin = float(worst[gear_index, steer_index])
                if margin < 0.0:
                    continue  # 구간 어딘가에서 차체가 장애물에 닿는다

                next_x = float(end_x[gear_index, steer_index])
                next_y = float(end_y[gear_index, steer_index])
                next_yaw = float(end_yaw[gear_index, steer_index])
                index = self._state_index(next_x, next_y, next_yaw)
                if index in tree.closed:
                    continue

                cost = node.cost + self._step_cost(
                    node, float(steer), float(direction), margin
                )
                existing = tree.nodes.get(index)
                if existing is not None and existing.cost <= cost:
                    continue

                estimate = tree.heuristic(next_x, next_y)
                if math.isinf(estimate):
                    continue  # 반대편 루트에서 도달할 수 없는 영역

                child = _Node(
                    next_x, next_y, normalize_angle(next_yaw), float(steer),
                    float(direction), arc, cost, node,
                )
                tree.nodes[index] = child
                tree.counter += 1
                heapq.heappush(
                    tree.open,
                    (cost + self.heuristic_weight * estimate,
                     tree.counter, index),
                )

    def _step_cost(self, node, steer, direction, margin):
        """DWA와 같은 항들로 1스텝 비용을 매긴다."""
        cost = self.w_length * self.step
        cost += self.w_steer * abs(steer) * self.step
        cost += self.w_steer_change * abs(steer - node.steer)
        cost += self.w_obstacle * self._obstacle_penalty(margin) * self.step
        if direction < 0.0:
            cost += self.w_reverse * self.step
        if node.parent is not None and direction != node.direction:
            cost += self.w_gear_change
        return cost

    def _try_join(self, tree, node, expansions):
        """방금 꺼낸 노드를 반대 트리와 클로소이드로 이어본다."""
        mate = tree.mate
        candidates = []

        # 1) 반대 트리의 루트로 바로 쏴보는 analytic expansion.
        distance = math.hypot(node.x - mate.root.x, node.y - mate.root.y)
        periodic = expansions % self.connect_interval == 0
        if distance <= self.connect_radius or periodic:
            candidates.append(mate.root)

        # 2) 반대 트리가 이미 와 있는 이웃 상태 칸들.
        base = self._state_index(node.x, node.y, node.yaw)
        span = self.connect_cell_span
        for x_step in range(-span, span + 1):
            for y_step in range(-span, span + 1):
                for yaw_step in range(-span, span + 1):
                    neighbor = mate.nodes.get(
                        (base[0] + x_step, base[1] + y_step,
                         base[2] + yaw_step)
                    )
                    if neighbor is not None:
                        candidates.append(neighbor)
        if len(candidates) > self.connect_candidates:
            candidates.sort(key=lambda item: item.cost)
            candidates = candidates[:self.connect_candidates]

        for other in candidates:
            # head는 항상 출발 트리 쪽, tail은 항상 목적 트리 쪽 노드다.
            head, tail = (
                (node, other) if tree.growth > 0.0 else (other, node)
            )
            bridge = self._connect(head.pose, tail.pose)
            if bridge is not None:
                return self._assemble(head, tail, *bridge)
        return None

    def _connect(self, start, goal):
        """
        두 pose를 클로소이드로 잇고 충돌까지 검사한다.

        전진 연결을 먼저 보고, 안 되면 (후진을 허용할 때) 두 헤딩을 pi만큼
        돌려 후진 연결을 본다. 반환값은 (poses, directions)다.
        """
        attempts = [1.0]
        if self.allow_reverse:
            attempts.append(-1.0)
        for direction in attempts:
            result = self._connect_direction(start, goal, direction)
            if result is not None:
                return result
        return None

    def _connect_direction(
            self, start, goal, direction, max_length=None):
        """충돌 없는 전진 또는 후진 곡선으로 두 차량 위치를 연결합니다."""
        if direction not in {-1.0, 1.0}:
            raise ValueError('direction must be +1.0 or -1.0')
        length_limit = self.connect_max_length
        if max_length is not None:
            length_limit = min(length_limit, float(max_length))

        # 클로소이드 길이는 두 점 사이 직선 거리보다 짧을 수 없다. 탐색 중
        # 대부분의 시도가 여기서 걸러지므로 값비싼 근 찾기를 아낄 수 있다.
        chord = math.hypot(goal[0] - start[0], goal[1] - start[1])
        if chord > length_limit:
            return None

        head = start
        tail = goal
        if direction < 0.0:
            head = (start[0], start[1], start[2] + math.pi)
            tail = (goal[0], goal[1], goal[2] + math.pi)
        clothoid = fit_clothoid(
            head, tail, max_curvature=self.max_curvature
        )
        if clothoid is None or clothoid.length > length_limit:
            return None

        poses = clothoid.sample(self.output_spacing)
        # 적분 잔차를 끝점에 맞춰 선형으로 흡수시킨다.
        residual = np.asarray(tail[:2]) - poses[-1, :2]
        if np.hypot(*residual) > 1.0e-6:
            weights = np.linspace(0.0, 1.0, len(poses))[:, np.newaxis]
            poses[:, :2] += weights * residual
        if direction < 0.0:
            poses[:, 2] -= math.pi  # 후진이면 차체 헤딩은 접선의 반대
        if not self._path_is_free(poses):
            return None
        directions = np.full(len(poses), direction)
        return poses, directions

    def _segment_poses(self, from_pose, steer, arc):
        """한 스텝 구간을 촘촘히 표본화한다(시작점 제외)."""
        count = max(
            2, int(math.ceil(abs(arc) / max(self.output_spacing, 1e-6))) + 1
        )
        poses = []
        for index in range(1, count):
            fraction = arc * index / (count - 1)
            x, y, yaw = self._propagate(*from_pose, steer, fraction)
            poses.append((x, y, normalize_angle(yaw)))
        return poses

    def _chain_to_root(self, node):
        """노드에서 루트까지 부모를 따라 올라간 목록(노드 -> 루트 순)."""
        chain = []
        current = node
        while current is not None:
            chain.append(current)
            current = current.parent
        return chain

    def _assemble(self, head, tail, bridge_poses, bridge_directions):
        """
        출발 트리 구간 + 클로소이드 + 목적 트리 구간을 이어 붙인다.

        ``head``는 출발 트리의 노드, ``tail``은 목적 트리의 노드다. 목적 트리는
        시간을 거꾸로 뻗었으므로, 노드에서 부모로 가는 방향이 실제 진행 방향이
        된다.
        """
        # 출발 트리: 루트 -> head 순으로 되돌려 앞에 붙인다.
        forward_chain = self._chain_to_root(head)[::-1]
        poses = [forward_chain[0].pose]
        directions = [1.0]
        for previous, current in zip(forward_chain, forward_chain[1:]):
            segment = self._segment_poses(
                previous.pose, current.steer, current.arc
            )
            poses.extend(segment)
            directions.extend([current.direction] * len(segment))

        # 클로소이드 다리. 첫 점은 head와 겹치므로 버린다.
        poses.extend(tuple(pose) for pose in bridge_poses[1:])
        directions.extend(bridge_directions[1:].tolist())

        # 목적 트리: tail -> 루트(goal) 순. 마지막 원소가 goal 루트다.
        reverse_chain = self._chain_to_root(tail)
        for current in reverse_chain[:-1]:
            segment = self._segment_poses(
                current.pose, current.steer, -current.arc
            )
            poses.extend(segment)
            directions.extend([current.direction] * len(segment))

        poses = np.asarray(poses, dtype=np.float64)
        # 여기까지 directions[i]는 "pose i에 도착할 때"의 기어다. 경로 추종
        # 쪽이 쓰기 좋게 "pose i에서 떠날 때"의 기어로 한 칸 당긴다.
        directions = np.asarray(directions[1:] + directions[-1:],
                                dtype=np.float64)
        poses[:, 2] = np.arctan2(np.sin(poses[:, 2]), np.cos(poses[:, 2]))
        steps = np.diff(poses[:, :2], axis=0)
        cost = float(np.hypot(steps[:, 0], steps[:, 1]).sum())
        return poses, directions, cost

    def _assemble_single_tree(self, node, bridge_poses, bridge_directions):
        """시작점의 운동 후보 경로와 마지막 클로소이드 연결을 합칩니다."""
        chain = self._chain_to_root(node)[::-1]
        poses = [chain[0].pose]
        directions = [1.0]
        for previous, current in zip(chain, chain[1:]):
            segment = self._segment_poses(
                previous.pose, current.steer, current.arc
            )
            poses.extend(segment)
            directions.extend([current.direction] * len(segment))

        # 연결 경로 첫 표본은 탐색 노드와 중복되므로 한 번만 넣습니다.
        poses.extend(tuple(pose) for pose in bridge_poses[1:])
        directions.extend(bridge_directions[1:].tolist())
        poses = np.asarray(poses, dtype=np.float64)
        directions = np.asarray(
            directions[1:] + directions[-1:], dtype=np.float64
        )
        poses[:, 2] = np.arctan2(np.sin(poses[:, 2]), np.cos(poses[:, 2]))
        return poses, directions
