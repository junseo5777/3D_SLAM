"""입력 지연 검사와 출력 변환을 담당하며 기존 명령 선택·펌웨어는 별도로 둡니다."""

import math
import time
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseStamped, Twist, TwistStamped
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import String
from .configuration import declare, gate_from, vehicle_from
from .safety import ModeGate
from .adapter import MacaronCalibration, adapter_failure
from .ros_helpers import stamp_seconds, planar_pose


class MacaronAdapter(Node):
    def __init__(self):
        super().__init__('macaron_adapter')
        self.cfg = c = declare(self)
        self.vehicle = vehicle_from(c)
        self.calibration = MacaronCalibration(
            c['speed_knots_mps'], c['forward_pwm_knots'], c['reverse_pwm_knots'],
            c['forward_input_sign'], c['steering_input_sign'], c['output_calibrated'],
            c['max_pwm'], c['max_steer_deg'])
        self.gate, self.mode = gate_from(c), ModeGate(c['mode_timeout'])
        self.command = self.parking_status = None
        self.last_reason = None
        self.output = None
        # 미리보기 모드에서는 /erp_writes 발행자를 생성하지 않습니다.
        if c['enable_drive']:
            if self.get_parameter('use_sim_time').value:
                raise ValueError('real vehicle output is disabled with use_sim_time')
            failure = self.calibration.readiness_failure()
            if failure:
                raise ValueError('enable_drive refused: ' + failure)
            if self.count_publishers('/erp_writes') > 0:
                raise ValueError('another AUTO publisher already owns /erp_writes')
            self.output = self.create_publisher(Twist, '/erp_writes', 1)
        self.preview = self.create_publisher(TwistStamped, '/parking/adapter_preview', 1)
        self.status = self.create_publisher(String, '/parking/adapter_status', 1)
        self.create_subscription(AckermannDriveStamped, '/parking/desired_command', self.on_command, 1)
        self.create_subscription(String, '/parking/status', self.on_parking_status, 1)
        self.create_subscription(String, c['ndt_status_topic'], self.on_ndt, 1)
        self.create_subscription(PoseStamped, c['pose_topic'], self.on_pose, 10)
        self.create_subscription(PointCloud2, c['cloud_topic'], self.on_cloud, qos_profile_sensor_data)
        self.create_subscription(Twist, c['mode_topic'], self.on_mode, 10)
        self.create_timer(0.05, self.tick)
        self.get_logger().info('macaron Adapter: ' + ('vehicle output enabled' if self.output is not None
                                                     else 'PREVIEW; no /erp_writes publisher'))

    def on_ndt(self, msg):
        self.gate.observe_status(msg.data, time.monotonic())

    def on_pose(self, msg):
        try:
            if msg.header.frame_id.lstrip('/') != self.cfg['map_frame'].lstrip('/'):
                raise ValueError('wrong pose frame')
            planar_pose(msg.pose)
            self.gate.observe_pose(stamp_seconds(msg.header.stamp), time.monotonic())
        except ValueError:
            self.gate.pose = None
            self.gate.good_count = 0

    def on_cloud(self, msg):
        if msg.width * msg.height < self.cfg['min_cloud_points'] or not msg.header.frame_id:
            self.gate.cloud = None
        else:
            self.gate.observe_cloud(stamp_seconds(msg.header.stamp), time.monotonic())

    def on_mode(self, msg):
        self.mode.observe(msg.linear.x, msg.angular.z, time.monotonic())

    def on_command(self, msg):
        speed, steer = msg.drive.speed, msg.drive.steering_angle
        if (msg.header.frame_id.lstrip('/') != self.cfg['base_frame'].lstrip('/')
                or not math.isfinite(speed) or not math.isfinite(steer)
                or abs(steer) > self.vehicle.max_steer + 1e-6):
            self.command = None
        else:
            self.command = (speed, steer, stamp_seconds(msg.header.stamp), time.monotonic())

    def on_parking_status(self, msg):
        values = dict(p.split('=', 1) for p in msg.data.split(';') if '=' in p)
        try:
            stamp = int(values['command_stamp_ns']) * 1e-9
            self.parking_status = (values.get('state', ''), time.monotonic(), stamp)
        except (KeyError, ValueError):
            self.parking_status = None

    def tick(self):
        wall = time.monotonic()
        ros_now = self.get_clock().now().nanoseconds * 1e-9
        reason = ('planning_only_no_vehicle_output' if self.cfg['planning_only'] else
                  adapter_failure(self.calibration, self.gate, self.mode,
                                  self.command, self.parking_status, ros_now, wall,
                                  self.cfg['command_timeout'], self.cfg['command_timeout'],
                                  self.cfg['reverse_observable']))
        speed = steer = 0.0
        if not reason:
            try:
                speed, steer = self.calibration.convert(*self.command[:2])
            except ValueError as exc:
                reason = str(exc)
        if self.output is not None and self.count_publishers('/erp_writes') > 1:
            reason, speed, steer = 'multiple_auto_publishers', 0.0, 0.0
        message = Twist()
        message.linear.x, message.angular.z = float(speed), float(steer)
        preview = TwistStamped()
        preview.header.frame_id = self.cfg['base_frame']
        preview.header.stamp = self.get_clock().now().to_msg()
        preview.twist = message
        self.preview.publish(preview)
        if self.output is not None:
            # 기존 명령 선택부의 속도 필터가 정지값에 수렴하도록 0을 계속 발행합니다.
            self.output.publish(message)
        status = reason or ('AUTO_COMMAND' if self.output is not None else 'PREVIEW_COMMAND')
        self.status.publish(String(data=f'state={status};enabled={self.output is not None};'
                                       f'pwm_input={speed:.3f};steer_input_deg={steer:.3f}'))
        if status != self.last_reason:
            self.get_logger().info(status)
            self.last_reason = status

    def publish_stop(self):
        if self.output is not None and self.context.ok():
            self.output.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = MacaronAdapter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.publish_stop()
            node.destroy_node()
        rclpy.try_shutdown()
