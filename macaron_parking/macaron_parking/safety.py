"""NDT 정상 상태와 운전자 모드를 확인하여 불확실한 입력에서 출력을 차단합니다."""

import math


class LocalizationGate:
    def __init__(self, pose_timeout=0.5, status_timeout=0.8, cloud_timeout=0.5,
                 confirmations=3, max_fitness=0.4, min_overlap=0.6):
        if min(pose_timeout, status_timeout, cloud_timeout) <= 0 or confirmations < 1:
            raise ValueError('gate timeouts/count must be positive')
        self.pose_timeout = pose_timeout
        self.status_timeout = status_timeout
        self.cloud_timeout = cloud_timeout
        self.confirmations = confirmations
        self.max_fitness = max_fitness
        self.min_overlap = min_overlap
        self.pose = None
        self.cloud = None
        self.status_received = None
        self.good_count = 0
        self.reason = 'waiting_for_ndt'

    def observe_status(self, text, received):
        parts = text.split(';')
        data = dict(part.split('=', 1) for part in parts[1:] if '=' in part)
        flags = [part for part in parts[1:] if '=' not in part]
        # NDT는 초기 위치·복구 후보에도 tracking을 사용하므로 세부 상태를 확인합니다.
        try:
            fitness = float(data['fitness'])
            overlap = float(data['overlap'])
            good = (parts[0] == 'tracking' and not flags
                    and 'relaxation_level' not in data and 'recovery_level' not in data
                    and 'candidate_confirmations' not in data
                    and math.isfinite(fitness) and 0 <= fitness <= self.max_fitness
                    and math.isfinite(overlap) and self.min_overlap <= overlap <= 1)
        except (KeyError, ValueError):
            good = False
        if (self.status_received is None or received < self.status_received
                or received - self.status_received > self.status_timeout):
            self.good_count = 0
        self.status_received = received
        self.good_count = self.good_count + 1 if good else 0
        self.reason = 'strict_tracking' if good else text

    def observe_pose(self, stamp, received):
        if self.pose is not None and stamp <= self.pose[0]:
            self.good_count = 0
        self.pose = (stamp, received)

    def observe_cloud(self, stamp, received):
        self.cloud = (stamp, received)

    @staticmethod
    def fresh(item, ros_now, wall_now, timeout):
        return (item is not None and all(math.isfinite(v) for v in item)
                and -0.1 <= ros_now - item[0] <= timeout
                and 0 <= wall_now - item[1] <= timeout)

    def failure(self, ros_now, wall_now, require_cloud=True):
        if self.good_count < self.confirmations:
            return 'ndt_not_ready:' + self.reason
        if (self.status_received is None
                or not 0 <= wall_now - self.status_received <= self.status_timeout):
            return 'ndt_status_stale'
        if not self.fresh(self.pose, ros_now, wall_now, self.pose_timeout):
            return 'localization_pose_stale'
        if require_cloud and not self.fresh(self.cloud, ros_now, wall_now, self.cloud_timeout):
            return 'live_cloud_stale'
        return ''


class ModeGate:
    COMMANDS = {(49.5, 99.0): 'MANUAL', (-500.0, -1000.0): 'AUTO',
                (500.0, 1000.0): 'ESTOP', (-49.5, -99.0): 'RELEASE'}

    def __init__(self, timeout=0.5):
        self.mode = 'MANUAL'
        self.estop = False
        self.received = None
        self.timeout = timeout

    def observe(self, speed, steer, received):
        command = self.COMMANDS.get((speed, steer))
        self.received = received
        if command in ('MANUAL', 'AUTO'):
            self.mode = command
            # 기존 명령 선택부는 AUTO 진입과 모드 전환 때도 긴급정지 상태를 유지합니다.
        elif command == 'ESTOP':
            self.estop = True
        elif command == 'RELEASE':
            self.estop = False

    def failure(self, now):
        if self.received is None or not 0 <= now - self.received <= self.timeout:
            return 'operator_command_stale'
        if self.estop:
            return 'operator_estop'
        if self.mode != 'AUTO':
            return 'manual_mode'
        return ''
