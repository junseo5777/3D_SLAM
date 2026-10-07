"""주차 계획·추종·최종 검사·한 번의 재시도를 관리하는 상태 기계입니다."""

import math
from dataclasses import dataclass, field
import numpy as np
from .path_tracking_dwa import PathTrackingDWA, poses_to_local, poses_to_map, lookahead_index
from .reverse_ssp import ReverseSSP
from .vehicle import footprint_clearance


def normalize(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def goal_errors(pose, goal):
    dx, dy = np.asarray(pose)[:2] - np.asarray(goal)[:2]
    c, s = math.cos(goal[2]), math.sin(goal[2])
    return (c * dx + s * dy, -s * dx + c * dy, normalize(pose[2] - goal[2]))


@dataclass
class Command:
    speed: float = 0.0
    steer: float = 0.0
    reason: str = ''
    trajectory: np.ndarray = field(default_factory=lambda: np.empty((0, 3)))


class ParkingSession:
    def __init__(self, vehicle, forward_speed=0.20, reverse_speed=0.15,
                 lookahead=0.7, endpoint_distance=0.10, gear_dwell=2.0,
                 final_dwell=2.0, longitudinal_tolerance=0.15, lateral_tolerance=0.10,
                 yaw_tolerance=math.radians(8.0), enable_retry=False, path_error_limit=0.75):
        if min(forward_speed, reverse_speed, lookahead, endpoint_distance, gear_dwell,
               final_dwell, longitudinal_tolerance, lateral_tolerance,
               yaw_tolerance, path_error_limit) <= 0:
            raise ValueError('controller values must be positive')
        self.vehicle = vehicle
        self.dwa = PathTrackingDWA(vehicle)
        self.ssp = ReverseSSP(vehicle)
        self.ssp.effective_speed = reverse_speed
        self.forward_speed, self.reverse_speed = forward_speed, reverse_speed
        self.lookahead, self.endpoint_distance = lookahead, endpoint_distance
        self.gear_dwell, self.final_dwell = gear_dwell, final_dwell
        self.longitudinal_tolerance, self.lateral_tolerance = longitudinal_tolerance, lateral_tolerance
        self.yaw_tolerance, self.enable_retry = yaw_tolerance, enable_retry
        self.path_error_limit = path_error_limit
        self.goal = None
        self.state = 'WAITING'
        self.reason = 'waiting_for_goal'
        self.retry_count = 0
        self.path = self.directions = None
        self.segment = self.progress = 0
        self.segments = []
        self.stopped_at = None
        self.last_steer = 0.0

    def request(self, goal):
        goal = np.asarray(goal, dtype=float).reshape(3)
        if not np.isfinite(goal).all():
            raise ValueError('goal must be finite')
        self.goal = goal.copy()
        self.retry_count = 0  # 새로운 목표를 받을 때만 재시도 횟수를 초기화합니다.
        self.replan()

    def replan(self):
        if self.goal is None:
            raise ValueError('no parking goal')
        self.path = self.directions = None
        self.state = 'RETRY_PLAN' if self.retry_count else 'PLAN'
        self.reason = 'planning_requested'
        self.last_steer = 0.0
        self.stopped_at = None

    def cancel(self):
        self.goal = None
        self.path = self.directions = None
        self.state, self.reason = 'WAITING', 'cancelled'

    def halt(self, reason):
        self.path = self.directions = None
        self.state, self.reason = 'STOP', reason

    def accept(self, result, now):
        if not result.success:
            self.halt('planning_failed:' + result.reason)
            return
        path = np.asarray(result.poses, dtype=float).reshape(-1, 3)
        directions = np.asarray(result.directions, dtype=np.int8).reshape(-1)
        if (not len(path) or len(path) != len(directions) or not np.isfinite(path).all()
                or not np.isin(directions, [-1, 1]).all()):
            self.halt('invalid_planner_result')
            return
        self.path, self.directions = path, directions
        self.progress = self.segment = 0
        self.segments = []
        start = 0
        # 방향 값 i는 표본 i에서 i+1로 출발하는 구간의 기어입니다.
        for i in range(1, len(path) - 1):
            if directions[i] != directions[i - 1]:
                self.segments.append((start, i, int(directions[start])))
                start = i
        self.segments.append((start, len(path) - 1, int(directions[start])))
        self.state = 'FINAL_CHECK' if len(path) == 1 else 'TRACKING'
        self.reason = result.reason
        self.stopped_at = now if len(path) == 1 else None

    def _final_check(self, pose, now):
        if self.stopped_at is None:
            self.stopped_at = now
        if now - self.stopped_at < self.final_dwell:
            return Command(reason='final_stop_dwell')
        longitudinal, lateral, yaw = goal_errors(pose, self.goal)
        if (abs(longitudinal) <= self.longitudinal_tolerance
                and abs(lateral) <= self.lateral_tolerance and abs(yaw) <= self.yaw_tolerance):
            self.state, self.reason = 'PARKING_DONE', 'goal_tolerances_met'
        elif self.enable_retry and self.retry_count == 0:
            self.retry_count = 1
            self.replan()
        else:
            self.halt(f'final_error:longitudinal={longitudinal:.3f},lateral={lateral:.3f},yaw={yaw:.3f}')
        return Command(reason=self.reason)

    def tick(self, pose, obstacles, now, reverse_observable=False):
        pose = np.asarray(pose, dtype=float).reshape(3)
        obstacles = np.asarray(obstacles, dtype=float).reshape(-1, 2)
        if not np.isfinite(pose).all() or not np.isfinite(obstacles).all():
            self.halt('invalid_control_input')
            return Command(reason=self.reason)
        if self.state == 'FINAL_CHECK':
            return self._final_check(pose, now)
        if self.state == 'GEAR_CHANGE':
            if now - self.stopped_at >= self.gear_dwell:
                self.segment += 1
                self.progress = self.segments[self.segment][0]
                self.state = 'TRACKING'
                self.stopped_at = None
                self.last_steer = 0.0
            return Command(reason='gear_change_stop')
        if self.state != 'TRACKING' or self.path is None:
            return Command(reason=self.reason)
        start, end, gear = self.segments[self.segment]
        first = max(start, self.progress - 2)
        last = min(end, self.progress + 30)
        candidates = self.path[first:last + 1]
        yaw_error = np.arctan2(np.sin(candidates[:, 2] - pose[2]),
                               np.cos(candidates[:, 2] - pose[2]))
        scores = np.linalg.norm(candidates[:, :2] - pose[:2], axis=1) + 0.20 * np.abs(yaw_error)
        nearest = first + int(np.argmin(scores))
        path_error = float(np.linalg.norm(self.path[nearest, :2] - pose[:2]))
        if path_error > self.path_error_limit:
            self.halt('path_error_exceeded')
            return Command(reason=self.reason)
        self.progress = max(self.progress, nearest)
        endpoint_error = float(np.linalg.norm(self.path[end, :2] - pose[:2]))
        tangent = gear * np.array([math.cos(self.path[end, 2]), math.sin(self.path[end, 2])])
        passed = (self.progress >= max(start, end - 2)
                  and np.dot(pose[:2] - self.path[end, :2], tangent) > 0)
        if endpoint_error <= self.endpoint_distance or passed:
            self.state = ('GEAR_CHANGE' if self.segment + 1 < len(self.segments) else 'FINAL_CHECK')
            self.stopped_at = now
            return Command(reason='segment_endpoint_stop')
        if gear > 0:
            target_index = lookahead_index(self.path, self.progress, end, self.lookahead)
            target = poses_to_local(self.path[target_index:target_index + 1], pose)[0]
            result = self.dwa.dwa(target, obstacles, 1, self.last_steer)
            if not result.success:
                self.halt('forward_collision_blocked')
                return Command(reason=self.reason)
            steer, trajectory = result.steer, result.trajectory
            speed = self.forward_speed
        else:
            result = self.ssp.track(self.path[start:end + 1], pose)
            steer, trajectory = result.steer, self.ssp.predict(result.steer)
            if not reverse_observable:
                return Command(reason='reverse_observation_required', trajectory=trajectory)
            if footprint_clearance(trajectory, obstacles, self.vehicle) < 0:
                self.halt('reverse_collision_blocked')
                return Command(reason=self.reason, trajectory=trajectory)
            speed = -self.reverse_speed
        if footprint_clearance([[0., 0., 0.]], obstacles, self.vehicle) < 0:
            self.halt('current_footprint_collision')
            return Command(reason=self.reason)
        self.last_steer = float(steer)
        # 끝점에 가까워질수록 물리 속도를 연속적으로 줄입니다.
        scale = max(0.20, min(1.0, endpoint_error / 0.6))
        self.reason = 'forward_dwa' if gear > 0 else 'reverse_ssp'
        return Command(speed * scale, float(steer), self.reason, trajectory)
