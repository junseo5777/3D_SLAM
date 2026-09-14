"""SLAM 알고리즘과 별도로 VLP-16 및 CV7-AHRS 드라이버를 실행한다."""

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def mounting_transforms(context):
    """명시적으로 활성화한 경우에만 실측한 센서 장착 변환을 발행한다."""
    with open(LaunchConfiguration('mounts_file').perform(context), encoding='utf-8') as stream:
        mounts = yaml.safe_load(stream)
    if mounts.get('calibrated') is not True:
        raise RuntimeError('Fill sensor_mounts.yaml with measured transforms and set calibrated: true before enabling publish_sensor_tf.')
    nodes = []
    for index, transform in enumerate(mounts['transforms']):
        xyz = transform['xyz']
        rpy_degrees = transform['rpy_degrees']
        if len(xyz) != 3 or len(rpy_degrees) != 3 or not all(
                math.isfinite(float(value)) for value in [*xyz, *rpy_degrees]):
            raise ValueError('Mount xyz/rpy_degrees must each contain three finite values.')
        rpy = [math.radians(float(value)) for value in rpy_degrees]
        arguments = []
        for name, value in zip(('x', 'y', 'z', 'roll', 'pitch', 'yaw'), [*xyz, *rpy]):
            arguments.extend([f'--{name}', str(value)])
        arguments.extend(['--frame-id', transform['parent'], '--child-frame-id', transform['child']])
        nodes.append(Node(
            package='tf2_ros', executable='static_transform_publisher',
            name=f'sensor_mount_{index}', arguments=arguments, output='screen'))
    return nodes


def sensor_nodes(context):
    """설정 파일의 각도를 도에서 라디안으로 바꾼 뒤 센서 노드를 만든다."""
    lidar_params_file = LaunchConfiguration('lidar_params_file').perform(context)
    with open(lidar_params_file, encoding='utf-8') as stream:
        lidar_config = yaml.safe_load(stream)
    driver_params = dict(lidar_config['velodyne_driver_node']['ros__parameters'])
    transform_params = dict(lidar_config['velodyne_transform_node']['ros__parameters'])
    direction_degree = float(transform_params.pop('view_direction_degree'))
    width_degree = float(transform_params.pop('view_width_degree'))
    if not math.isfinite(direction_degree) or not math.isfinite(width_degree):
        raise ValueError('LiDAR view angles must be finite degree values.')
    if width_degree <= 0.0 or width_degree > 360.0:
        raise ValueError('LiDAR view_width_degree must be greater than 0 and at most 360.')
    transform_params['view_direction'] = math.radians(direction_degree)
    transform_params['view_width'] = math.radians(width_degree)

    imu_share = get_package_share_directory('microstrain_inertial_driver')
    cloud_share = get_package_share_directory('velodyne_pointcloud')
    with open(os.path.join(imu_share, 'microstrain_inertial_driver_common',
                           'config', 'params.yml'), encoding='utf-8') as stream:
        imu_defaults = yaml.safe_load(stream)
    return [
        Node(
            package='velodyne_driver', executable='velodyne_driver_node',
            name='velodyne_driver_node', output='screen',
            condition=IfCondition(LaunchConfiguration('lidar_enabled')),
            parameters=[driver_params, {'use_sim_time': False}]),
        Node(
            package='velodyne_pointcloud', executable='velodyne_transform_node',
            name='velodyne_transform_node', output='screen',
            condition=IfCondition(LaunchConfiguration('lidar_enabled')),
            parameters=[transform_params, {
                'calibration': ParameterValue(
                    LaunchConfiguration('calibration_file'), value_type=str),
                'use_sim_time': False}],
            remappings=[('velodyne_points', LaunchConfiguration('input_topic'))]),
        Node(
            package='microstrain_inertial_driver',
            executable='microstrain_inertial_driver_node',
            name='microstrain_inertial_driver', output='screen',
            condition=IfCondition(LaunchConfiguration('imu_enabled')),
            parameters=[imu_defaults, LaunchConfiguration('imu_params_file'),
                        {'use_sim_time': False}]),
    ]


def generate_launch_description():
    share = get_package_share_directory('velodyne_slam')
    cloud_share = get_package_share_directory('velodyne_pointcloud')
    return LaunchDescription([
        DeclareLaunchArgument('lidar_params_file', default_value=os.path.join(share, 'config', 'lidar.yaml')),
        DeclareLaunchArgument('imu_params_file', default_value=os.path.join(share, 'config', 'imu.yaml')),
        DeclareLaunchArgument('calibration_file', default_value=os.path.join(cloud_share, 'params', 'VLP16db.yaml')),
        DeclareLaunchArgument('mounts_file', default_value=os.path.join(share, 'config', 'sensor_mounts.yaml')),
        DeclareLaunchArgument('publish_sensor_tf', default_value='false'),
        DeclareLaunchArgument('lidar_enabled', default_value='true'),
        DeclareLaunchArgument('imu_enabled', default_value='true'),
        DeclareLaunchArgument('input_topic', default_value='/velodyne_points'),
        OpaqueFunction(function=mounting_transforms,
                       condition=IfCondition(LaunchConfiguration('publish_sensor_tf'))),
        OpaqueFunction(function=sensor_nodes),
    ])
