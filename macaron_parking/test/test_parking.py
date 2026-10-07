"""합성 입력으로 검사하며 ROS 통신과 차량 장치를 사용하지 않습니다."""

import math
import struct
from types import SimpleNamespace
import numpy as np
import pytest
from macaron_parking.vehicle import Vehicle, footprint_clearance
from macaron_parking.cloud_map import CloudMap, load_pcd, live_obstacles
from macaron_parking.hybrid_astar import HybridAStar, PlanResult
from macaron_parking.retry import plan_retry
from macaron_parking.controller import ParkingSession, goal_errors
from macaron_parking.safety import LocalizationGate, ModeGate
from macaron_parking.adapter import MacaronCalibration, adapter_failure


def flat_map(obstacles=()):
    floor = np.array([[x, y, -0.9] for x in np.arange(-5, 6, .4) for y in np.arange(-5, 6, .4)])
    points = np.vstack((floor, np.asarray(obstacles).reshape(-1, 3)))
    return CloudMap(points, Vehicle(), ground_mode='flat', ground_z=-.9, bounds=(-5, 5, -5, 5))


def good_gate(now=10.):
    gate = LocalizationGate()
    for t in (now - .4, now - .2, now):
        gate.observe_status('tracking;fitness=0.1;overlap=0.9;iterations=4', t)
    gate.observe_pose(now, now)
    gate.observe_cloud(now, now)
    return gate


def calibration():
    # 검사 전용 합성 보정값이며 실제 차량의 속도 실측값이 아닙니다.
    return MacaronCalibration([0., .2, .4], [0., 35., 60.], [0., 30., 55.],
                              forward_input_sign=-1., steering_input_sign=-1., confirmed=True)


def test_shared_vehicle_dimensions():
    vehicle = Vehicle()
    assert vehicle.front_extent == pytest.approx(1.06)
    assert vehicle.front_overhang == pytest.approx(.31)
    assert vehicle.base_height == .14 and vehicle.height == .50
    assert vehicle.wheel_base == .75 and vehicle.width == .68
    assert vehicle.min_radius == pytest.approx(.75 / math.tan(math.radians(21)))


@pytest.mark.parametrize('binary', [False, True])
def test_pcd_field_order_and_finite_filter(tmp_path, binary):
    path = tmp_path / 'map.pcd'
    header = ('VERSION .7\nFIELDS intensity z x y\nSIZE 4 4 4 4\nTYPE F F F F\n'
              'COUNT 1 1 1 1\nWIDTH 3\nHEIGHT 1\nPOINTS 3\nDATA ')
    rows = [(5., 0., 1., 2.), (9., 3., 4., 5.), (0., 0., math.nan, 0.)]
    payload = (b''.join(struct.pack('<ffff', *row) for row in rows) if binary else
               '\n'.join(' '.join(map(str, row)) for row in rows).encode())
    path.write_bytes((header + ('binary\n' if binary else 'ascii\n')).encode() + payload)
    assert np.allclose(load_pcd(path), [[1, 2, 0], [4, 5, 3]])


def test_pcd_truncation_rejected(tmp_path):
    p = tmp_path / 'bad.pcd'
    p.write_bytes(b'FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nWIDTH 1\nPOINTS 1\nDATA binary\n')
    with pytest.raises(ValueError, match='payload'):
        load_pcd(p)


@pytest.mark.parametrize('mode', ['ascii', 'binary'])
def test_pcd_rejects_multivalued_coordinate_fields(tmp_path, mode):
    path = tmp_path / 'invalid_count.pcd'
    path.write_bytes(('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                      'COUNT 2 1 1\nPOINTS 1\nDATA ' + mode + '\n').encode())
    with pytest.raises(ValueError, match='scalar'):
        load_pcd(path)


def test_map_filters_floor_and_overhead_but_not_collision_height():
    assert flat_map([[.7, 0, -.9], [.7, 0, 1.5]]).clearance([0, 0, 0]) >= 0
    assert flat_map([[.7, 0, -.65]]).clearance([0, 0, 0]) < 0
    assert flat_map([[.7, 0, -.65]]).clearance([0, 0, math.pi / 2]) >= 0


def test_map_boundary_and_missing_ground_are_blocked():
    assert flat_map().clearance([4.8, 0, 0]) < 0
    cloud = CloudMap([[0, 0, 0], [1, 1, 0]], Vehicle(), min_ground_points=5, bounds=(-5, 5, -5, 5))
    assert cloud.clearance([0, 0, 0]) < 0


def test_local_ground_uses_relative_height():
    ground = [[x, y, 4.0] for x in np.arange(-3, 4, .2) for y in np.arange(-3, 4, .2)]
    cloud = CloudMap(ground + [[.8, 0, 4.3]], Vehicle(), bounds=(-3, 3, -3, 3))
    assert cloud.ground_at(0, 0) == pytest.approx(4)
    assert cloud.clearance([0, 0, 0]) < 0


@pytest.mark.parametrize('ground_mode', ['flat', 'local'])
@pytest.mark.parametrize('margin', [0., .12])
def test_batch_collision_preserves_individual_checks(ground_mode, margin):
    # 경사진 지면, 차체·여유 영역 장애물, 상부 점과 지도 밖 후보를 함께 검사합니다.
    rng = np.random.default_rng(42)
    floor_xy = np.array([[x, y] for x in np.arange(-5, 5.1, .2)
                        for y in np.arange(-5, 5.1, .2)])
    floor = np.column_stack((floor_xy, -.9 + .015 * floor_xy[:, 0]))
    obstacle_xy = rng.uniform(-4, 4, (250, 2))
    obstacles = np.column_stack((obstacle_xy, -.9 + .015 * obstacle_xy[:, 0]
                                 + rng.uniform(.08, 1., len(obstacle_xy))))
    vehicle = Vehicle(margin=margin)
    cloud = CloudMap(np.vstack((floor, obstacles)), vehicle, ground_mode=ground_mode,
                     ground_z=-.9, bounds=(-5, 5, -5, 5))
    poses = rng.uniform([-5.2, -5.2, -math.pi], [5.2, 5.2, math.pi], (2, 170, 3))
    poses[0, 0] = [math.nan, 0, 0]
    poses[0, 1] = [0, math.inf, 0]
    expected = np.array([cloud.clearance(p) for p in poses.reshape(-1, 3)]).reshape(2, 170)
    actual = cloud.margins(poses)
    np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-12)
    np.testing.assert_array_equal(actual >= 0, expected >= 0)


def test_batch_collision_blocks_unknown_ground_and_handles_empty_candidates():
    cloud = CloudMap([[0, 0, 0], [1, 1, 0]], Vehicle(), min_ground_points=5,
                     bounds=(-5, 5, -5, 5))
    assert np.isneginf(cloud.margins([[0, 0, 0], [4.8, 0, 0]])).all()
    assert cloud.margins(np.empty((2, 0, 3))).shape == (2, 0)
    assert np.isposinf(flat_map().margins([[0, 0, 0], [0, 0, math.pi / 2]])).all()


def test_live_ground_height_and_body_filter():
    points = [[2, 0, -.14], [2, 0, .06], [2, 0, 2], [.5, 0, .06]]
    assert np.allclose(live_obstacles(points, Vehicle()), [[2, 0]])


@pytest.mark.parametrize('goal,gear', [((2, 0, 0), 1), ((-2, 0, 0), -1)])
def test_hybrid_forward_and_reverse_use_macaron_geometry(goal, gear):
    cloud = flat_map()
    planner = HybridAStar(cloud)
    result = planner.plan((0, 0, 0), goal)
    assert result.success, result.reason
    assert cloud.path_is_free(result.poses)
    assert np.all(result.directions == gear)
    assert np.allclose(result.poses[-1], goal, atol=1e-4)
    assert planner.max_curvature == pytest.approx(math.tan(math.radians(21)) / .75)


def test_planner_rejects_blocked_goal_and_already_at_goal():
    planner = HybridAStar(flat_map([[2.5, 0, -.65]]))
    assert not planner.plan((0, 0, 0), (2, 0, 0)).success
    same = HybridAStar(flat_map()).plan((0, 0, 0), (0, 0, 0))
    assert same.success and len(same.poses) == 1


@pytest.mark.parametrize('retry', [False, True])
def test_planning_cancellation_interrupts_search_before_time_limit(retry):
    planner = HybridAStar(flat_map())
    planner.connect_max_length = .01  # 바로 연결되지 않도록 하여 실제 탐색 중 취소를 검사합니다.
    checks = 0

    def cancel_check():
        nonlocal checks
        checks += 1
        return checks >= 4

    if retry:
        result = plan_retry(planner, (0, 0, 0), (-.3, 0, 0), cancel_check=cancel_check)
    else:
        result = planner.plan((-3, 0, 0), (3, 0, 0), cancel_check=cancel_check)
    assert not result.success and 'planning cancelled' in result.reason
    assert 0 < result.expansions < planner.max_expansions
    assert result.elapsed < planner.time_limit


def test_retry_requires_forward_distance_then_reverse_finish():
    planner = HybridAStar(flat_map())
    result = plan_retry(planner, (0, 0, 0), (-.3, 0, 0), min_forward=.4)
    assert result.success, result.reason
    assert result.directions[0] == 1 and result.directions[-1] == -1
    cusp = int(np.flatnonzero(result.directions < 0)[0])
    forward_length = np.linalg.norm(np.diff(result.poses[:cusp + 1, :2], axis=0), axis=1).sum()
    assert forward_length >= .4 - 1e-6
    assert planner.map.path_is_free(result.poses)


@pytest.mark.parametrize('status', [
    'waiting_for_initial_pose', 'relocalizing;reason=poor_match',
    'tracking;fitness=.1;overlap=.9;verifying_candidate',
    'tracking;fitness=.1;overlap=.9;initial_refinement_complete',
    'tracking;fitness=.1;overlap=.9;recovery_level=0',
    'tracking;fitness=nan;overlap=.9', 'tracking;fitness=.1;overlap=.2',
    'poor_match;fitness=.1;overlap=.9',
])
def test_gate_does_not_confuse_accepted_candidates_with_tracking(status):
    gate = good_gate()
    gate.observe_status(status, 10.01)
    assert gate.failure(10.01, 10.01)


def test_gate_warmup_freshness_reset_and_cloud_timeout():
    gate = good_gate()
    assert not gate.failure(10, 10)
    assert gate.failure(10.6, 10.6)
    gate.observe_pose(10, 10.1)  # 위치 메시지 시각이 증가하지 않으면 정상 확인 횟수 초기화
    assert gate.failure(10.1, 10.1).startswith('ndt_not_ready')
    gate = LocalizationGate()
    gate.observe_status('tracking;fitness=.1;overlap=.9', 10.)
    gate.observe_pose(10., 10.)
    gate.observe_cloud(10., 10.)
    assert gate.failure(10., 10.)  # 과거 상태 한 개만으로는 정상으로 인정하지 않습니다.


def test_adapter_units_signs_stop_and_independent_timeout():
    cal = calibration()
    assert cal.convert(.2, math.radians(10)) == pytest.approx((-35, -10))
    assert cal.convert(-.2, math.radians(-10)) == pytest.approx((30, 10))
    assert cal.convert(0, 1) == (0, 0)
    with pytest.raises(ValueError):
        MacaronCalibration().convert(.2, 0)
    mode = ModeGate()
    mode.observe(-500, -1000, 10.)
    command = (.2, 0., 10., 10.)
    assert not adapter_failure(cal, good_gate(), mode, command, ('TRACKING', 10., 10.), 10., 10.)
    assert adapter_failure(cal, good_gate(), mode, command, ('TRACKING', 10., 10.), 10.4, 10.4) == 'parking_command_stale'
    assert adapter_failure(cal, good_gate(), mode, command, ('TRACKING', 10., 9.9), 10., 10.) == 'parking_command_status_mismatch'
    mode.observe(500, 1000, 10.01)
    assert adapter_failure(cal, good_gate(), mode, command, ('TRACKING', 10., 10.), 10.01, 10.01) == 'operator_estop'


def test_controller_gears_dwell_and_reverse_guard():
    session = ParkingSession(Vehicle(), gear_dwell=1., final_dwell=1.)
    session.request((-1, 0, 0))
    result = PlanResult(True, [[0, 0, 0], [.5, 0, 0], [-1, 0, 0]], [1, -1, -1])
    session.accept(result, 0.)
    assert session.tick([0, 0, 0], [], 0.).speed > 0
    assert session.tick([.5, 0, 0], [], .1).speed == 0
    assert session.state == 'GEAR_CHANGE'
    assert session.tick([.5, 0, 0], [], .5).speed == 0
    session.tick([.5, 0, 0], [], 1.2)
    assert session.tick([.5, 0, 0], [], 1.3).speed == 0
    reverse = session.tick([.5, 0, 0], [], 1.4, reverse_observable=True)
    assert reverse.speed < 0
    assert abs(reverse.steer) <= Vehicle().max_steer


def test_one_retry_budget_survives_new_plan_and_second_failure_stops():
    session = ParkingSession(Vehicle(), final_dwell=1., enable_retry=True)
    session.request([0, 0, 0])
    session.state, session.stopped_at = 'FINAL_CHECK', 0.
    session.tick([.3, .2, .2], [], 1.1)
    assert session.state == 'RETRY_PLAN' and session.retry_count == 1
    session.accept(PlanResult(True, [[.3, .2, .2], [0, 0, 0]], [1, -1]), 1.2)
    assert session.retry_count == 1
    session.state, session.stopped_at = 'FINAL_CHECK', 2.
    session.tick([.3, .2, .2], [], 3.1)
    assert session.state == 'STOP' and session.retry_count == 1


def test_goal_frame_errors_and_whole_trajectory_collision():
    longitudinal, lateral, yaw = goal_errors([1, 2.3, .1 + math.pi/2], [1, 2, math.pi/2])
    assert (longitudinal, lateral, yaw) == pytest.approx((.3, 0, .1))
    # 시작점·끝점만 확인하면 중간 궤적의 장애물을 놓칠 수 있습니다.
    assert footprint_clearance([[0, 0, 0], [4, 0, 0]], [[2, 0]], Vehicle()) > 0
    assert footprint_clearance([[0, 0, 0], [1.5, 0, 0], [4, 0, 0]], [[2, 0]], Vehicle()) < 0


def test_adapter_node_default_creates_no_vehicle_publisher(monkeypatch):
    from rclpy.node import Node
    from macaron_parking.adapter_node import MacaronAdapter
    from macaron_parking.configuration import DEFAULTS
    topics = []
    monkeypatch.setattr(Node, '__init__', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'declare_parameter', lambda self, key, value, **kwargs: SimpleNamespace(value=DEFAULTS[key]))
    monkeypatch.setattr(Node, 'create_publisher', lambda self, kind, topic, qos:
                        (topics.append(topic) or SimpleNamespace(publish=lambda msg: None)))
    monkeypatch.setattr(Node, 'create_subscription', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'create_timer', lambda *a, **kw: None)
    monkeypatch.setattr(Node, 'get_logger', lambda *a: SimpleNamespace(info=lambda msg: None))
    MacaronAdapter()
    assert '/erp_writes' not in topics and '/erp_write' not in topics
    assert '/parking/adapter_preview' in topics
