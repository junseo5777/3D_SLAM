"""기존 velodyne_slam 위치 추정 launch를 그대로 포함하여 주차와 연결합니다."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory('macaron_parking'))
    slam = Path(get_package_share_directory('velodyne_slam'))
    candidates = [parent / 'src/3d_slam/maps/library.pcd'
                  for parent in Path(__file__).resolve().parents]
    default_map = next((str(p) for p in candidates if p.is_file()), '')
    # 원본 launch의 입력도 새 패키지에서 한국어로 설명하며 기본값은 동일하게 유지합니다.
    original_arguments = {
        'allow_uncalibrated': ('true', '기존 위치 추정에서 미보정 센서 설치값의 진단 실행 허용'),
        'input_topic': ('/velodyne_points', '기존 FAST-LIO와 LiDAR 드라이버의 점군 토픽'),
        'imu_topic': ('/imu/data', '기존 FAST-LIO가 사용할 IMU 입력 토픽'),
        'fast_lio_params_file': (str(slam / 'config/fast_lio.yaml'), '기존 FAST-LIO 설정 파일'),
        'ndt_params_file': (str(slam / 'config/ndt.yaml'), '기존 NDT 위치 추정 설정 파일'),
        'extrinsics_file': (str(slam / 'config/lidar_imu_calibration.yaml'), '기존 LiDAR·IMU 설치 변환과 시각 보정 파일'),
        'cart_params_file': (str(slam / 'config/cart.yaml'), '기존 LiDAR 설치 위치와 차량 기준 좌표 설정 파일'),
        'lidar_params_file': (str(slam / 'config/lidar.yaml'), '기존 VLP-16 드라이버 설정 파일'),
        'imu_params_file': (str(slam / 'config/imu.yaml'), '기존 MicroStrain IMU 드라이버 설정 파일'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='localization', choices=['localization'],
                              description='주차에 필요한 기존 NDT 위치 추정 모드로 실행'),
        DeclareLaunchArgument('map_path', default_value=default_map,
                              description='NDT와 주차 계획이 함께 사용할 PCD 지도 경로'),
        DeclareLaunchArgument('start_sensors', default_value='true',
                              description='기존 launch에서 LiDAR·IMU 드라이버를 함께 실행할지 여부'),
        DeclareLaunchArgument('use_sim_time', default_value='false',
                              description='시뮬레이션 시각 사용 여부; true이면 실차 출력 차단'),
        DeclareLaunchArgument('enable_drive', default_value='false',
                              description='실측 보정 완료 후 기존 AUTO 토픽으로 차량 명령 발행'),
        DeclareLaunchArgument('goal_topic', default_value='/parking/goal',
                              description='지도 기준 주차 목표를 받을 토픽'),
        DeclareLaunchArgument('initial_pose_topic', default_value='/initialpose',
                              description='경로 확인 모드에서 RViz 시작 자세를 받을 토픽'),
        DeclareLaunchArgument('planning_only', default_value='false',
                              description='NDT 정합을 기다리지 않고 RViz 시작·목표로 경로만 생성'),
        DeclareLaunchArgument('vehicle_params', default_value=str(share / 'config/vehicle.yaml'),
                              description='차량 제원과 충돌 여유를 설정한 YAML 파일'),
        DeclareLaunchArgument('parking_params', default_value=str(share / 'config/parking.yaml'),
                              description='위치 추정 검사·경로 계획·추종을 설정한 YAML 파일'),
        DeclareLaunchArgument('adapter_params', default_value=str(share / 'config/adapter.yaml'),
                              description='실측 출력 보정값과 명령 지연 한도를 설정한 YAML 파일'),
        *[DeclareLaunchArgument(name, default_value=value, description=description)
          for name, (value, description) in original_arguments.items()],
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(slam / 'launch/lio.launch.py')),
                                 launch_arguments={name: LaunchConfiguration(name) for name in
                                                   ('mode', 'map_path', 'start_sensors', 'use_sim_time',
                                                    *original_arguments)}.items()),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share / 'launch/parking.launch.py')),
                                 launch_arguments={name: LaunchConfiguration(name) for name in
                                                   ('map_path', 'use_sim_time', 'enable_drive', 'goal_topic',
                                                    'initial_pose_topic', 'planning_only',
                                                    'vehicle_params', 'parking_params', 'adapter_params')}.items()),
    ])
