"""FAST-LIO2와 위치추정 전용 후처리 노드로 현재 위치를 계산한다."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    return LaunchDescription([IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('velodyne_slam'), 'launch', 'lio.launch.py')),
        launch_arguments={'mode': 'localization'}.items())])
