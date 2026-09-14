"""센서 드라이버와 이전 매핑 전처리 기본 노드를 실행한다."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('velodyne_slam'), 'config', 'mapping.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=config),
        DeclareLaunchArgument('input_topic', default_value='/velodyne_points'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('map_path', default_value=''),
        # 기본값은 실제 센서를 실행하며 bag 재생 시에는 센서를 자동으로 끈다.
        DeclareLaunchArgument('start_sensors', default_value='true'),
        DeclareLaunchArgument('lidar_enabled', default_value='true'),
        DeclareLaunchArgument('imu_enabled', default_value='true'),
        DeclareLaunchArgument('publish_sensor_tf', default_value='false'),
        DeclareLaunchArgument('lidar_params_file', default_value=os.path.join(
            get_package_share_directory('velodyne_slam'), 'config', 'lidar.yaml')),
        DeclareLaunchArgument('imu_params_file', default_value=os.path.join(
            get_package_share_directory('velodyne_slam'), 'config', 'imu.yaml')),
        DeclareLaunchArgument('mounts_file', default_value=os.path.join(
            get_package_share_directory('velodyne_slam'), 'config', 'sensor_mounts.yaml')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('velodyne_slam'), 'launch', 'sensors.launch.py')),
            condition=IfCondition(PythonExpression([
                "'", LaunchConfiguration('start_sensors'), "'.lower() in ('true', '1') and '",
                LaunchConfiguration('use_sim_time'), "'.lower() not in ('true', '1')"])),
            launch_arguments={
                name: LaunchConfiguration(name) for name in (
                    'input_topic', 'lidar_enabled', 'imu_enabled', 'publish_sensor_tf',
                    'lidar_params_file', 'imu_params_file', 'mounts_file')
            }.items()),
        Node(
            package='velodyne_slam', executable='slam_node',
            name='velodyne_slam', output='screen',
            parameters=[LaunchConfiguration('params_file'), {
                'mode': 'mapping',
                'input_topic': ParameterValue(
                    LaunchConfiguration('input_topic'), value_type=str),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
                'map_path': ParameterValue(
                    LaunchConfiguration('map_path'), value_type=str),
            }],
        ),
    ])
