"""확인 모드의 입력·재계획·출력 차단을 ROS 통신과 차량 없이 검사합니다."""

import math
from concurrent.futures import Future
from types import SimpleNamespace
import numpy as np
import pytest
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from macaron_parking.cloud_map import CloudMap
from macaron_parking.configuration import DEFAULTS
from macaron_parking.controller import ParkingSession
from macaron_parking.hybrid_astar import HybridAStar
from macaron_parking.parking_node import ParkingNode
from macaron_parking.adapter_node import MacaronAdapter
from macaron_parking.safety import LocalizationGate, ModeGate
from macaron_parking.vehicle import Vehicle


class PlanningExecutor:
    """탐색 결과 전달 시점을 제어하여 클릭 변경과 이전 결과의 경쟁을 검사합니다."""

    def __init__(self, deferred=False):
        self.deferred = deferred
        self.calls = []

    def submit(self, function, *args):
        future = Future()
        self.calls.append((function, args, future))
        if not self.deferred:
            future.set_result(function(*args))
        return future

    def complete(self, index):
        function, args, future = self.calls[index]
        future.set_result(function(*args))


def pose_message(initial, x, y=0., yaw=0.):
    msg = PoseWithCovarianceStamped() if initial else PoseStamped()
    msg.header.frame_id, msg.header.stamp = 'map', Time(sec=10)
    pose = msg.pose.pose if initial else msg.pose
    pose.position.x, pose.position.y = float(x), float(y)
    pose.orientation.z, pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    return msg


def preview_node(obstacles=(), deferred=False):
    """메시지 변환과 실제 탐색은 유지하며 ROS 노드 생성·통신만 생략합니다."""
    node = object.__new__(ParkingNode)
    node.cfg = dict(DEFAULTS, planning_only=True)
    node.vehicle = Vehicle()
    floor = [[x, y, -.9] for x in np.arange(-5, 6, .4) for y in np.arange(-5, 6, .4)]
    points = np.vstack((floor, np.asarray(obstacles).reshape(-1, 3)))
    node.map = CloudMap(points, node.vehicle, ground_mode='flat', ground_z=-.9,
                        bounds=(-5, 5, -5, 5))
    node.planner = HybridAStar(node.map)
    node.session = ParkingSession(node.vehicle)
    node.gate, node.mode = LocalizationGate(), ModeGate()
    node.pose = node.preview_start = None
    node.base_z = 0.
    node.obstacles = np.empty((0, 2))
    node.cloud_error = 'waiting_for_cloud'
    node.executor_pool = PlanningExecutor(deferred)
    node.future = None
    node.plan_cancel = None
    node.generation = node.plan_id = 0
    node.last_path_id = node.last_status = None
    node.outputs = {}
    for name in ('path', 'directions', 'command', 'status', 'local_path', 'footprint'):
        messages = node.outputs[name] = []
        setattr(node, name + '_pub', SimpleNamespace(publish=messages.append))
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(
        nanoseconds=10_000_000_000, to_msg=lambda: Time(sec=10)))
    node.get_logger = lambda: SimpleNamespace(info=lambda msg: None, error=lambda msg: None,
                                              warning=lambda msg: None)
    return node


@pytest.mark.parametrize('goal_first', [False, True])
def test_manual_start_goal_plan_without_localization_or_live_cloud(goal_first):
    node = preview_node()
    first = lambda: node.on_goal(pose_message(False, 2.))
    second = lambda: node.on_preview_start(pose_message(True, 0.))
    if not goal_first:
        first, second = second, first
    first()
    node.tick()
    assert not node.executor_pool.calls
    second()
    node.tick()
    node.tick()
    assert node.session.state == 'PREVIEW_READY'
    path = node.outputs['path'][-1]
    assert path.header.frame_id == 'map' and len(path.poses) > 1
    assert path.poses[0].pose.position.x == pytest.approx(0.)
    assert path.poses[-1].pose.position.x == pytest.approx(2.)
    assert path.poses[0].pose.position.z == pytest.approx(-.76)
    assert all(msg.drive.speed == 0 and msg.drive.steering_angle == 0 for msg in node.outputs['command'])
    node.on_ndt(SimpleNamespace(data='relocalizing_continuously'))
    node.on_pose(pose_message(False, 4., 4.))
    for _ in range(5):
        node.tick()
    assert node.session.state == 'PREVIEW_READY'
    assert np.allclose(node.preview_start, [0, 0, 0])
    assert node.outputs['path'][-1] is path


@pytest.mark.parametrize('change', ['start', 'goal', 'replan'])
def test_new_click_discards_pending_result_and_plans_again(change):
    node = preview_node(deferred=True)
    node.on_preview_start(pose_message(True, 0.))
    node.on_goal(pose_message(False, 2.))
    node.tick()
    previous_cancel = node.plan_cancel
    assert not previous_cancel.is_set()
    if change == 'start':
        node.on_preview_start(pose_message(True, .4))
    elif change == 'goal':
        node.on_goal(pose_message(False, 3.))
    else:
        node.on_replan(None, SimpleNamespace())
    assert previous_cancel.is_set()
    node.executor_pool.complete(0)
    assert not node.executor_pool.calls[0][2].result().success
    node.tick()
    assert node.session.path is None
    assert len(node.executor_pool.calls) == 2
    assert not node.plan_cancel.is_set()
    node.executor_pool.complete(1)
    node.tick()
    assert node.session.state == 'PREVIEW_READY'
    assert node.session.path[0, 0] == pytest.approx(.4 if change == 'start' else 0.)
    assert node.session.path[-1, 0] == pytest.approx(3. if change == 'goal' else 2.)


def test_cancel_interrupts_pending_search_and_does_not_restart():
    node = preview_node(deferred=True)
    node.on_preview_start(pose_message(True, 0.))
    node.on_goal(pose_message(False, 2.))
    node.tick()
    node.on_cancel(None, SimpleNamespace())
    assert node.plan_cancel.is_set()
    node.executor_pool.complete(0)
    node.tick()
    assert node.session.goal is None and node.session.path is None
    assert not node.outputs['path'][-1].poses
    assert len(node.executor_pool.calls) == 1
    assert all(msg.drive.speed == 0 for msg in node.outputs['command'])


@pytest.mark.parametrize('obstacle_x', [.5, 2.5])
def test_preview_still_rejects_colliding_start_or_goal(obstacle_x):
    node = preview_node([[obstacle_x, 0, -.65]])
    node.on_preview_start(pose_message(True, 0.))
    node.on_goal(pose_message(False, 2.))
    node.tick()
    node.tick()
    assert node.session.state == 'STOP'
    assert node.session.reason.startswith('planning_failed:')
    assert not node.outputs['path'][-1].poses
    assert all(msg.drive.speed == 0 for msg in node.outputs['command'])


def test_preview_cancel_removes_path_and_does_not_restart():
    node = preview_node()
    node.on_preview_start(pose_message(True, 0.))
    node.on_goal(pose_message(False, 2.))
    node.tick()
    node.tick()
    node.on_cancel(None, SimpleNamespace())
    node.tick()
    assert node.session.goal is None and not node.outputs['path'][-1].poses
    assert len(node.executor_pool.calls) == 1


def test_normal_mode_still_requires_healthy_localization():
    node = preview_node()
    node.cfg['planning_only'] = False
    node.on_preview_start(pose_message(True, 0.))
    assert node.preview_start is None and node.pose is None
    node.on_goal(pose_message(False, 2.))
    node.tick()
    assert not node.executor_pool.calls
    assert not node.outputs['path'][-1].poses
    assert 'ndt_not_ready' in node.outputs['status'][-1].data
    assert node.outputs['command'][-1].drive.speed == 0


@pytest.mark.parametrize('kind', [ParkingNode, MacaronAdapter])
def test_preview_cannot_enable_vehicle_output_even_when_explicitly_requested(monkeypatch, kind):
    from rclpy.node import Node
    config = dict(DEFAULTS, planning_only=True, enable_drive=True)
    topics = []
    monkeypatch.setattr(Node, '__init__', lambda *args, **kwargs: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, key, value, **kwargs:
                        SimpleNamespace(value=config[key]))
    monkeypatch.setattr(Node, 'create_publisher', lambda self, message_type, topic, qos:
                        topics.append(topic))
    with pytest.raises(ValueError, match='실차 출력'):
        kind()
    assert '/erp_writes' not in topics and '/erp_write' not in topics
