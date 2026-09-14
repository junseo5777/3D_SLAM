"""명시적인 보정값을 사용하는 FAST-LIO 매핑·위치추정 공통 파이프라인이다."""
import math
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 지도 파일명을 바꿀 때는 이 한 줄만 수정한다.
MAP_FILENAME = 'cart_test_01.pcd'
MAP_DIRECTORY = Path.home() / 'ros2_ws/src/3d_slam/maps'


def cart_transform(calibration, cart):
    """IMU→라이다와 카트→라이다를 조합해 IMU→카트 변환을 계산한다."""
    xyz = [float(v) for v in cart['lidar_xyz']]
    angles = [float(v) for v in cart['lidar_rpy_degrees']]
    if len(xyz) != 3 or len(angles) != 3 or not all(math.isfinite(v) for v in xyz + angles):
        raise ValueError('cart.yaml requires finite xyz[3] and rpy_degrees[3]')
    roll, pitch, yaw = map(math.radians, angles)
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                             math.sin(pitch), math.cos(yaw), math.sin(yaw))
    base_r_lidar = [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr],
                    [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr], [-sp, cp*sr, cp*cr]]
    imu_r_lidar = [calibration['rotation'][i:i+3] for i in (0, 3, 6)]
    r = [[sum(imu_r_lidar[i][k] * base_r_lidar[j][k] for k in range(3))
          for j in range(3)] for i in range(3)]
    t = [calibration['translation'][i] - sum(r[i][k] * xyz[k] for k in range(3))
         for i in range(3)]
    return [float(v) for i in range(3) for v in [*r[i], t[i]]] + [0.0, 0.0, 0.0, 1.0]


def base_mount_node(matrix):
    """기존 IMU 트리에 base_link를 자식으로 연결해 TF 부모 중복을 피한다."""
    r = [matrix[i:i+3] for i in (0, 4, 8)]
    pitch = math.atan2(-r[2][0], math.hypot(r[0][0], r[1][0]))
    if math.hypot(r[0][0], r[1][0]) > 1e-6:
        roll, yaw = math.atan2(r[2][1], r[2][2]), math.atan2(r[1][0], r[0][0])
    else:
        roll, yaw = 0.0, math.atan2(-r[0][1], r[1][1])
    arguments = []
    for name, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'),
                           [matrix[3], matrix[7], matrix[11], roll, pitch, yaw]):
        arguments.extend([f'--{name}', str(value)])
    arguments.extend(['--frame-id', 'imu_link', '--child-frame-id', 'base_link'])
    return Node(package='tf2_ros', executable='static_transform_publisher',
                name='cart_mount', arguments=arguments, output='screen')


def launch_pipeline(context):
    def arg(name):
        return LaunchConfiguration(name).perform(context)
    def flag(name):
        value = arg(name).lower()
        if value not in ('true', 'false', '1', '0'):
            raise RuntimeError(f'{name} must be true or false')
        return value in ('true', '1')
    share = get_package_share_directory('velodyne_slam')
    mode = arg('mode')
    if mode not in ('mapping', 'localization'):
        raise RuntimeError('mode must be mapping or localization')
    with open(arg('extrinsics_file'), encoding='utf-8') as stream:
        calibration = yaml.safe_load(stream)
    calibrated = calibration.get('calibrated') is True
    if not calibrated and not (flag('allow_uncalibrated')
                               or calibration.get('allow_uncalibrated') is True):
        raise RuntimeError('LiDAR/IMU mounting is not calibrated. Fill lidar_imu_calibration.yaml. '
                           'Set allow_uncalibrated: true in that file only for diagnostic experiments; '
                           'resulting poses/maps are unvalidated.')
    translation = calibration['translation']
    rotation = calibration['rotation']
    offset = float(calibration['time_offset_lidar_to_imu'])
    if len(translation) != 3 or len(rotation) != 9 or not all(
            math.isfinite(float(v)) for v in [*translation, *rotation, offset]):
        raise RuntimeError('Calibration requires finite translation[3], rotation[9], and time offset')
    r = [rotation[i:i+3] for i in (0, 3, 6)]
    for i in range(3):
        for j in range(3):
            if abs(sum(r[i][k]*r[j][k] for k in range(3)) - (1 if i == j else 0)) > 1e-4:
                raise RuntimeError('Calibration rotation must be orthonormal')
    det = (r[0][0]*(r[1][1]*r[2][2]-r[1][2]*r[2][1])
           - r[0][1]*(r[1][0]*r[2][2]-r[1][2]*r[2][0])
           + r[0][2]*(r[1][0]*r[2][1]-r[1][1]*r[2][0]))
    if abs(det - 1) > 1e-4:
        raise RuntimeError('Calibration rotation determinant must be +1')
    with open(arg('cart_params_file'), encoding='utf-8') as stream:
        imu_to_base = cart_transform(calibration, yaml.safe_load(stream))
    map_path = str(Path(arg('map_path')).expanduser().resolve())
    if mode == 'localization' and not Path(map_path).is_file():
        raise RuntimeError(f'Localization requires an existing PCD map: {map_path}')
    sim = flag('use_sim_time')
    actions = []
    if not calibrated:
        actions.append(LogInfo(msg='UNCALIBRATED DIAGNOSTIC RUN: poses and maps are not validated.'))
    pitch = math.atan2(-r[2][0], math.hypot(r[0][0], r[1][0]))
    if math.hypot(r[0][0], r[1][0]) > 1e-6:
        roll, yaw = math.atan2(r[2][1], r[2][2]), math.atan2(r[1][0], r[0][0])
    else:
        roll, yaw = 0.0, math.atan2(-r[0][1], r[1][1])
    mount_arguments = []
    for key, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), [*translation, roll, pitch, yaw]):
        mount_arguments.extend([f'--{key}', str(value)])
    mount_arguments.extend(['--frame-id', 'imu_link', '--child-frame-id', 'velodyne'])
    actions.append(Node(package='tf2_ros', executable='static_transform_publisher',
                        name='lidar_imu_mount', arguments=mount_arguments, output='screen'))
    actions.append(base_mount_node(imu_to_base))
    if flag('start_sensors') and not sim:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', 'sensors.launch.py')),
            launch_arguments={name: arg(name) for name in ('lidar_params_file', 'imu_params_file', 'input_topic')}.items()))
    lio = Node(package='fast_lio', executable='fastlio_mapping', name='fast_lio', output='screen',
        parameters=[arg('fast_lio_params_file'), {
            'use_sim_time': sim, 'world_frame': 'map' if mode == 'mapping' else 'odom',
            'body_frame': 'imu_link', 'common.lid_topic': arg('input_topic'),
            'common.imu_topic': arg('imu_topic'),
            'common.time_offset_lidar_to_imu': offset,
            'mapping.extrinsic_T': [float(v) for v in translation],
            'mapping.extrinsic_R': [float(v) for v in rotation]}],
        remappings=[('/Odometry', '/fast_lio/odometry'),
                    ('/cloud_registered', '/fast_lio/cloud_world'),
                    ('/cloud_registered_body', '/fast_lio/cloud_body'),
                    ('/cloud_effected', '/fast_lio/effective_points'),
                    ('/Laser_map', '/fast_lio/local_map'), ('/path', '/fast_lio/path'),
                    ('map_save', '/fast_lio/unused_map_save')])
    if mode == 'mapping':
        backend = Node(package='velodyne_slam', executable='map_builder_node', name='map_builder', output='screen',
            parameters=[os.path.join(share, 'config', 'map_builder.yaml'), {
                'map_path': map_path, 'use_sim_time': sim, 'imu_frame': 'imu_link',
                'lidar_translation_in_imu': [float(v) for v in translation]}])
    else:
        backend = Node(package='velodyne_slam', executable='ndt_localization_node', name='ndt_localization', output='screen',
            parameters=[arg('ndt_params_file'), {'map_path': map_path, 'use_sim_time': sim,
                'base_transform_in_imu': imu_to_base,
                'lidar_rotation_in_imu': [float(v) for v in rotation],
                'lidar_translation_in_imu': [float(v) for v in translation]}])
    for node in (lio, backend):
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node,
            on_exit=[EmitEvent(event=Shutdown(reason='A pipeline node exited'))])))
    actions.extend([lio, backend])
    return actions


def generate_launch_description():
    share = get_package_share_directory('velodyne_slam')
    defaults = {
        'mode': 'mapping', 'start_sensors': 'true', 'use_sim_time': 'false',
        'allow_uncalibrated': 'true', 'input_topic': '/velodyne_points', 'imu_topic': '/imu/data',
        'map_path': str(MAP_DIRECTORY / MAP_FILENAME),
        'fast_lio_params_file': os.path.join(share, 'config', 'fast_lio.yaml'),
        'ndt_params_file': os.path.join(share, 'config', 'ndt.yaml'),
        'extrinsics_file': os.path.join(share, 'config', 'lidar_imu_calibration.yaml'),
        'cart_params_file': os.path.join(share, 'config', 'cart.yaml'),
        'lidar_params_file': os.path.join(share, 'config', 'lidar.yaml'),
        'imu_params_file': os.path.join(share, 'config', 'imu.yaml'),
    }
    return LaunchDescription([*(DeclareLaunchArgument(k, default_value=v) for k, v in defaults.items()),
                              OpaqueFunction(function=launch_pipeline)])
