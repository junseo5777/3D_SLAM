"""주차 계산을 실행하며 기존 위치 추정과 차량 출력은 별도로 연결합니다."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('macaron_parking'))
    candidates = [parent / 'src/3d_slam/maps/library.pcd'
                  for parent in Path(__file__).resolve().parents]
    default_map = next((str(p) for p in candidates if p.is_file()), '')
    declarations = [
        DeclareLaunchArgument('map_path', default_value=default_map,
                              description='NDT와 주차 계획이 함께 사용할 PCD 지도 경로'),
        DeclareLaunchArgument('vehicle_params', default_value=str(share / 'config/vehicle.yaml'),
                              description='차량 제원과 충돌 여유를 설정한 YAML 파일'),
        DeclareLaunchArgument('parking_params', default_value=str(share / 'config/parking.yaml'),
                              description='위치 추정 검사·경로 계획·추종을 설정한 YAML 파일'),
        DeclareLaunchArgument('adapter_params', default_value=str(share / 'config/adapter.yaml'),
                              description='실측 출력 보정값과 명령 지연 한도를 설정한 YAML 파일'),
        DeclareLaunchArgument('goal_topic', default_value='/parking/goal',
                              description='지도 기준 주차 목표를 받을 토픽'),
        DeclareLaunchArgument('initial_pose_topic', default_value='/initialpose',
                              description='경로 확인 모드에서 RViz 시작 자세를 받을 토픽'),
        DeclareLaunchArgument('planning_only', default_value='false',
                              description='NDT 정합을 기다리지 않고 RViz 시작·목표로 경로만 생성'),
        DeclareLaunchArgument('enable_drive', default_value='false',
                              description='실측 보정 완료 후 기존 AUTO 토픽으로 차량 명령 발행'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='시뮬레이션 시각 사용 여부; true이면 실차 출력 차단'),
    ]
    shared = [LaunchConfiguration('vehicle_params'), LaunchConfiguration('parking_params'),
              LaunchConfiguration('adapter_params'), {
                  'enable_drive': ParameterValue(LaunchConfiguration('enable_drive'), value_type=bool),
                  'planning_only': ParameterValue(LaunchConfiguration('planning_only'), value_type=bool),
                  'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
              }]
    return LaunchDescription(declarations + [
        Node(package='macaron_parking', executable='parking_node', name='parking_node',
             parameters=shared + [{'map_path': LaunchConfiguration('map_path'),
                                   'goal_topic': LaunchConfiguration('goal_topic'),
                                   'initial_pose_topic': LaunchConfiguration('initial_pose_topic')}], output='screen'),
        Node(package='macaron_parking', executable='macaron_adapter', name='macaron_adapter',
             parameters=shared, output='screen', condition=UnlessCondition(LaunchConfiguration('planning_only'))),
    ])
