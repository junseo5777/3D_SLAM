"""NDT 차량 위치를 받아 PCD 경로와 DWA·SSP의 물리 속도·조향을 계산합니다."""

from concurrent.futures import ThreadPoolExecutor
import math
import time
from threading import Event
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist, Point
from nav_msgs.msg import Path
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String, Int8MultiArray
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformListener
from visualization_msgs.msg import Marker
from .cloud_map import CloudMap, load_pcd, live_obstacles
from .configuration import declare, vehicle_from, gate_from, configure_planner
from .controller import ParkingSession
from .hybrid_astar import HybridAStar
from .retry import plan_retry
from .safety import ModeGate
from .ros_helpers import stamp_seconds, rotation_matrix, planar_pose, path_message, direction_message
from .path_tracking_dwa import poses_to_map


class ParkingNode(Node):
    def __init__(self):
        super().__init__('parking_node')
        self.cfg = c = declare(self)
        self.vehicle = vehicle_from(c)
        if c['control_rate'] <= 0 or c['min_cloud_points'] < 1:
            raise ValueError('invalid control rate/cloud size')
        self.get_logger().info('PCD를 읽고 주차 충돌 검사 자료를 준비합니다.')
        self.map = CloudMap(load_pcd(c['map_path']), self.vehicle,
                            c['ground_mode'], c['ground_z'], c['ground_radius'],
                            c['ground_percentile'], c['min_ground_points'], c['obstacle_min_height'],
                            prefilter=c['map_prefilter'])
        self.get_logger().info(f'주차 지도 준비 완료: 원본 {len(self.map.points)}점 → '
                               f'충돌 후보 {len(self.map.collision_points)}점, '
                               f'바닥 제외 {self.map.removed_below}점, '
                               f'상부 제외 {self.map.removed_above}점, '
                               f'전처리 {self.map.prefilter_elapsed:.2f}초')
        self.planner = HybridAStar(self.map)
        configure_planner(self.planner, c)
        self.session = ParkingSession(
            self.vehicle, c['forward_speed_mps'], c['reverse_speed_mps'], c['lookahead'],
            c['endpoint_distance'], c['gear_dwell'], c['final_dwell'], c['longitudinal_tolerance'],
            c['lateral_tolerance'], math.radians(c['yaw_tolerance_deg']), c['enable_retry'], c['path_error_limit'])
        self.gate, self.mode = gate_from(c), ModeGate(c['mode_timeout'])
        self.pose = None
        self.preview_start = None
        self.base_z = 0.0
        self.obstacles = np.empty((0, 2))
        self.cloud_error = 'waiting_for_cloud'
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.executor_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='parking_plan')
        self.future = None
        self.plan_cancel = None
        self.generation = 0
        self.plan_id = 0
        self.last_path_id = None
        self.last_status = None
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.path_pub = self.create_publisher(Path, '/parking/path', latched)
        self.directions_pub = self.create_publisher(Int8MultiArray, '/parking/directions', latched)
        self.map_pub = self.create_publisher(PointCloud2, '/parking/map', latched)
        self.collision_map_pub = self.create_publisher(PointCloud2, '/parking/collision_map', latched)
        self.local_path_pub = self.create_publisher(Path, '/parking/local_path', 1)
        self.footprint_pub = self.create_publisher(Marker, '/parking/footprint', 1)
        self.command_pub = self.create_publisher(AckermannDriveStamped, '/parking/desired_command', 1)
        self.status_pub = self.create_publisher(String, '/parking/status', 1)
        if c['planning_only']:
            # 경로 확인 모드는 RViz 클릭값만 사용하며 센서·NDT 정상 여부를 요구하지 않습니다.
            self.create_subscription(PoseWithCovarianceStamped, c['initial_pose_topic'],
                                     self.on_preview_start, 10)
        else:
            self.create_subscription(PoseStamped, c['pose_topic'], self.on_pose, 10)
            # 과거에 저장된 NDT 상태를 새 정상 신호로 세지 않도록 현재 상태만 구독합니다.
            self.create_subscription(String, c['ndt_status_topic'], self.on_ndt, 1)
            self.create_subscription(PointCloud2, c['cloud_topic'], self.on_cloud, qos_profile_sensor_data)
            self.create_subscription(Twist, c['mode_topic'], self.on_mode, 10)
        self.create_subscription(PoseStamped, c['goal_topic'], self.on_goal, 10)
        self.create_service(Trigger, '/parking/cancel', self.on_cancel)
        self.create_service(Trigger, '/parking/replan', self.on_replan)
        self.create_timer(1.0 / c['control_rate'], self.tick)
        header = Header(frame_id=c['map_frame'], stamp=self.get_clock().now().to_msg())
        self.map_pub.publish(point_cloud2.create_cloud_xyz32(header, self.map.points.tolist()))
        self.collision_map_pub.publish(point_cloud2.create_cloud_xyz32(header,
                                                                      self.map.collision_points.tolist()))
        self.publish_path()
        self.get_logger().info('PCD parking ready; output is physical intent only. '
                               f'Length={self.vehicle.length}, width={self.vehicle.width}, '
                               f'rear={self.vehicle.rear_overhang}, front={self.vehicle.front_extent}.')
        if c['planning_only']:
            self.get_logger().info('경로 확인 모드: RViz 초기 자세와 목표 자세를 지정하세요. '
                                   '경로만 표시하며 속도·조향 명령은 항상 0입니다.')

    def on_preview_start(self, msg):
        """경로 확인 모드의 시작 자세를 저장하고 기존 목표가 있으면 다시 계획합니다."""
        if not self.cfg['planning_only']:
            return
        self.cancel_pending_plan()
        try:
            if msg.header.frame_id.lstrip('/') != self.cfg['map_frame'].lstrip('/'):
                raise ValueError('시작 자세는 지도 좌표계로 지정해야 합니다')
            start = planar_pose(msg.pose.pose)
            self.preview_start = start.copy()
            self.pose = start.copy()
            ground = self.map.ground_at(start[0], start[1])
            self.base_z = (ground + self.vehicle.base_height if math.isfinite(ground)
                           else msg.pose.pose.position.z)
            if self.session.goal is not None:
                self.session.replan()
            self.generation += 1
            self.publish_path()
        except ValueError as exc:
            self.preview_start = None
            self.pose = None
            self.session.halt('invalid_preview_start:' + str(exc))
            self.generation += 1
            self.publish_path()
            self.get_logger().error(str(exc))

    def on_pose(self, msg):
        if self.cfg['planning_only']:
            return
        try:
            if msg.header.frame_id.lstrip('/') != self.cfg['map_frame'].lstrip('/'):
                raise ValueError('localization_pose_wrong_frame')
            self.pose = planar_pose(msg.pose)
            self.base_z = msg.pose.position.z
            self.gate.observe_pose(stamp_seconds(msg.header.stamp), time.monotonic())
        except ValueError as exc:
            self.gate.pose = None
            self.gate.good_count = 0
            self.gate.reason = str(exc)

    def on_ndt(self, msg):
        self.gate.observe_status(msg.data, time.monotonic())

    def on_mode(self, msg):
        self.mode.observe(msg.linear.x, msg.angular.z, time.monotonic())

    def on_cloud(self, msg):
        try:
            points = np.asarray(list(point_cloud2.read_points(msg, field_names=('x', 'y', 'z'),
                                                             skip_nans=True)), dtype=float).reshape(-1, 3)
            points = points[np.isfinite(points).all(axis=1)]
            if len(points) < self.cfg['min_cloud_points']:
                raise ValueError('too_few_live_points')
            frame = msg.header.frame_id.lstrip('/')
            if not frame:
                raise ValueError('empty_cloud_frame')
            if frame != self.cfg['base_frame'].lstrip('/'):
                transform = self.buffer.lookup_transform(self.cfg['base_frame'], frame,
                                                         Time.from_msg(msg.header.stamp)).transform
                translation = [transform.translation.x, transform.translation.y, transform.translation.z]
                points = points @ rotation_matrix(transform.rotation).T + translation
            self.obstacles = live_obstacles(points, self.vehicle, self.cfg['obstacle_min_height'],
                                            self.cfg['obstacle_max_range'])
            self.gate.observe_cloud(stamp_seconds(msg.header.stamp), time.monotonic())
            self.cloud_error = ''
        except Exception as exc:
            self.gate.cloud = None
            self.cloud_error = str(exc)

    def on_goal(self, msg):
        self.cancel_pending_plan()
        try:
            if msg.header.frame_id.lstrip('/') != self.cfg['map_frame'].lstrip('/'):
                raise ValueError('goal must be in map frame')
            goal = planar_pose(msg.pose)
            self.session.request(goal)
            self.generation += 1
            self.publish_path()
        except ValueError as exc:
            self.session.halt('invalid_goal:' + str(exc))
            self.generation += 1
            self.publish_path()
            self.get_logger().error(str(exc))

    def on_cancel(self, request, response):
        self.cancel_pending_plan()
        self.session.cancel()
        self.generation += 1
        self.publish_path()
        response.success, response.message = True, 'parking cancelled; zero intent'
        return response

    def on_replan(self, request, response):
        self.cancel_pending_plan()
        if self.session.goal is None:
            response.success, response.message = False, 'no parking goal'
        else:
            self.session.replan()
            self.generation += 1
            self.publish_path()
            response.success, response.message = True, 'replan requested; retry budget preserved'
        return response

    def cancel_pending_plan(self):
        """새 입력·취소·위치 추정 실패 시 이전 탐색이 시간 한도까지 계속 돌지 않게 합니다."""
        if self.plan_cancel is not None:
            self.plan_cancel.set()

    def publish_path(self):
        self.plan_id += 1
        path = self.session.path if self.session.path is not None else np.empty((0, 3))
        directions = self.session.directions if self.session.directions is not None else []
        self.path_pub.publish(path_message(path, self.cfg['map_frame'], self.get_clock().now().to_msg(), self.base_z))
        self.directions_pub.publish(direction_message(directions, self.plan_id))
        self.last_path_id = id(self.session.path)

    def publish_intent(self, speed, steer, reason, trajectory=None):
        now = self.get_clock().now().to_msg()
        command = AckermannDriveStamped()
        command.header.frame_id, command.header.stamp = self.cfg['base_frame'], now
        command.drive.speed, command.drive.steering_angle = float(speed), float(steer)
        self.command_pub.publish(command)
        text = (f'state={self.session.state};plan_id={self.plan_id};retry_count={self.session.retry_count};'
                f'command_stamp_ns={now.sec * 1000000000 + now.nanosec};'
                f'reason={reason};speed_mps={speed:.3f};steer_rad={steer:.4f}')
        self.status_pub.publish(String(data=text))
        key = (self.session.state, reason)
        if key != self.last_status:
            self.get_logger().info(text)
            self.last_status = key
        if trajectory is not None and len(trajectory) and self.pose is not None:
            local = poses_to_map(trajectory, self.pose)
        else:
            local = np.empty((0, 3))
        self.local_path_pub.publish(path_message(local, self.cfg['map_frame'], now, self.base_z))
        if self.pose is not None:
            marker = Marker()
            marker.header.frame_id, marker.header.stamp = self.cfg['map_frame'], now
            marker.ns, marker.id, marker.type, marker.action = 'vehicle', 0, Marker.LINE_STRIP, Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.03
            marker.color.r, marker.color.g, marker.color.a = 1.0, 0.8, 1.0
            corners = self.vehicle.corners(self.pose)
            marker.points = [Point(x=float(p[0]), y=float(p[1]), z=float(self.base_z))
                             for p in np.vstack((corners, corners[0]))]
            self.footprint_pub.publish(marker)

    def tick(self):
        wall = time.monotonic()
        if self.cfg['planning_only']:
            self.tick_preview(wall)
            return
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        failure = self.gate.failure(ros_now, wall, require_cloud=False)
        if not failure and (self.cfg['require_auto_mode'] or self.cfg['enable_drive']):
            failure = self.mode.failure(wall)
        if self.future is not None and self.future.done():
            generation, future = self.future_generation, self.future
            self.future = None
            if generation == self.generation and not failure:
                try:
                    result = future.result()
                    if (np.linalg.norm(self.pose[:2] - self.future_start[:2]) > 0.15
                            or abs(math.atan2(math.sin(self.pose[2] - self.future_start[2]),
                                              math.cos(self.pose[2] - self.future_start[2]))) > math.radians(5)):
                        self.session.replan()
                    else:
                        self.session.accept(result, wall)
                except Exception as exc:
                    self.session.halt('planning_exception:' + str(exc))
                self.publish_path()
        if failure:
            if (self.session.state in ('TRACKING', 'GEAR_CHANGE', 'FINAL_CHECK')
                    or (self.future is not None and self.session.state in ('PLAN', 'RETRY_PLAN'))):
                self.session.halt(failure)
                self.cancel_pending_plan()
                self.generation += 1
                self.publish_path()
            self.publish_intent(0., 0., failure)
            return
        if self.session.state in ('PLAN', 'RETRY_PLAN'):
            if self.future is None:
                start, goal = self.pose.copy(), self.session.goal.copy()
                self.future_start = start.copy()
                self.future_generation = self.generation
                self.plan_cancel = Event()
                if self.session.state == 'RETRY_PLAN':
                    self.future = self.executor_pool.submit(plan_retry, self.planner, start, goal,
                                                           self.cfg['retry_min_forward'], self.cfg['retry_max_reverse'],
                                                           self.plan_cancel.is_set)
                else:
                    self.future = self.executor_pool.submit(self.planner.plan, start, goal, self.plan_cancel.is_set)
            self.publish_intent(0., 0., 'planning')
            return
        failure = self.gate.failure(ros_now, wall, require_cloud=True)
        if failure:
            if self.session.state in ('TRACKING', 'GEAR_CHANGE', 'FINAL_CHECK'):
                self.session.halt(failure + ':' + self.cloud_error)
                self.publish_path()
            self.publish_intent(0., 0., failure)
            return
        command = self.session.tick(self.pose, self.obstacles, wall, self.cfg['reverse_observable'])
        if id(self.session.path) != self.last_path_id:
            self.publish_path()
        self.publish_intent(command.speed, command.steer, command.reason, command.trajectory)

    def tick_preview(self, wall):
        """수동 시작·목표 자세로 PCD 경로를 생성하고 결과를 유지하며 주행은 하지 않습니다."""
        if self.future is not None and self.future.done():
            generation, future = self.future_generation, self.future
            self.future = None
            # 클릭을 바꾼 뒤 도착한 이전 탐색 결과는 새 경로를 덮어쓰지 않습니다.
            if generation == self.generation:
                try:
                    result = future.result()
                    self.session.accept(result, wall)
                    if self.session.state != 'STOP':
                        self.session.state = 'PREVIEW_READY'
                        self.session.reason = 'planning_preview_ready'
                        self.get_logger().info(f'경로 생성 성공: 길이 {result.length:.2f}m, '
                                               f'표본 {len(result.poses)}개, 탐색 시간 {result.elapsed:.2f}초')
                    else:
                        self.get_logger().warning(f'경로 생성 실패: {self.session.reason}; '
                                                  f'탐색 시간 {result.elapsed:.2f}초, '
                                                  f'확장한 상태 {result.expansions}개')
                except Exception as exc:
                    self.session.halt('planning_exception:' + str(exc))
                self.publish_path()
        if self.session.goal is None:
            reason = 'waiting_for_preview_goal' if self.preview_start is not None else 'waiting_for_preview_start'
        elif self.preview_start is None:
            reason = 'waiting_for_preview_start'
        elif self.session.state in ('PLAN', 'RETRY_PLAN'):
            if self.future is None:
                self.future_start = self.preview_start.copy()
                self.future_generation = self.generation
                self.plan_cancel = Event()
                self.future = self.executor_pool.submit(self.planner.plan, self.future_start.copy(),
                                                       self.session.goal.copy(), self.plan_cancel.is_set)
            reason = 'planning_preview'
        else:
            reason = self.session.reason
        # NDT·센서·JOY 입력과 관계없이 확인 모드에서는 물리 명령을 항상 0으로 유지합니다.
        self.publish_intent(0., 0., reason)

    def close(self):
        self.cancel_pending_plan()
        try:
            # Ctrl+C로 ROS 문맥이 종료된 뒤에는 메시지를 발행할 수 없습니다.
            if self.context.ok():
                self.publish_intent(0., 0., 'shutdown')
        finally:
            self.executor_pool.shutdown(wait=True, cancel_futures=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ParkingNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        rclpy.try_shutdown()
