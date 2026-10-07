"""RViz 시작·목표 클릭으로 PCD 주차 경로만 확인하는 launch입니다."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('macaron_parking'))
    candidates = [parent / 'src/3d_slam/maps/library.pcd'
                  for parent in Path(__file__).resolve().parents]
    default_map = next((str(path) for path in candidates if path.is_file()), '')
    return LaunchDescription([
        DeclareLaunchArgument('map_path', default_value=default_map,
                              description='충돌 검사와 RViz 표시에 사용할 저장 PCD 지도'),
        DeclareLaunchArgument('vehicle_params', default_value=str(share / 'config/vehicle.yaml'),
                              description='차량 제원과 충돌 여유 설정 파일'),
        DeclareLaunchArgument('parking_params', default_value=str(share / 'config/parking.yaml'),
                              description='지면 필터·Hybrid A* 탐색 설정 파일'),
        DeclareLaunchArgument('planning_only', default_value='true', choices=['true'],
                              description='이 실행에서는 경로 확인 모드를 항상 사용'),
        DeclareLaunchArgument('enable_drive', default_value='false', choices=['false'],
                              description='이 실행에서는 차량 출력을 항상 차단'),
        DeclareLaunchArgument('initial_pose_topic', default_value='/initialpose',
                              description='RViz 시작 자세 입력 토픽'),
        DeclareLaunchArgument('goal_topic', default_value='/goal_pose',
                              description='RViz 목표 자세 입력 토픽'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='지도·경로·시작 자세 도구가 설정된 RViz를 함께 실행'),
        DeclareLaunchArgument('rviz_config', default_value=str(share / 'rviz/parking_preview.rviz'),
                              description='경로 확인용 RViz 화면 설정 파일'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / 'launch/parking.launch.py')),
            launch_arguments={
                'map_path': LaunchConfiguration('map_path'),
                'vehicle_params': LaunchConfiguration('vehicle_params'),
                'parking_params': LaunchConfiguration('parking_params'),
                'planning_only': LaunchConfiguration('planning_only'),
                'enable_drive': LaunchConfiguration('enable_drive'),
                'initial_pose_topic': LaunchConfiguration('initial_pose_topic'),
                'goal_topic': LaunchConfiguration('goal_topic'),
            }.items()),
        Node(package='rviz2', executable='rviz2', name='parking_preview_rviz',
             arguments=['-d', LaunchConfiguration('rviz_config')], output='screen',
             condition=IfCondition(LaunchConfiguration('rviz'))),
    ])
