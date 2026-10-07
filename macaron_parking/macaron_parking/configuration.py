"""계획·추종·출력 변환이 공유하는 파라미터와 한국어 설명."""

import math
from .vehicle import Vehicle
from .safety import LocalizationGate

# 모든 거리·높이는 m, 속도는 m/s이며 각도 단위는 이름과 설명에 표시합니다.
DEFAULTS = {
    'wheel_base': 0.75,  # 앞뒤 바퀴 축 사이 거리(m)
    'vehicle_width': 0.68,  # 차체 전체 폭(m)
    'vehicle_length': 1.3,  # 차체 전체 길이(m)
    'rear_overhang': 0.24,  # 후륜축에서 뒤 범퍼까지 거리(m)
    'vehicle_height': 0.5,  # 지면에서 차량 최고점까지 높이(m)
    'base_height': 0.14,  # 지면에서 후륜축 중심까지 높이(m)
    'max_steer_deg': 21.0,  # 계획과 추종에 공통 적용하는 조향 한계(도)
    'collision_margin': 0.12,  # 차체 바깥에 추가하는 충돌 여유(m)
    'map_frame': 'map',  # 저장 지도와 목표 위치의 기준 좌표계
    'base_frame': 'base_link',  # 후륜축 중심의 차량 좌표계
    'pose_topic': '/localization/pose',  # NDT가 발행하는 차량 위치 입력 토픽
    'ndt_status_topic': '/localization/status',  # NDT 정합 상태 입력 토픽
    'cloud_topic': '/velodyne_points',  # 현재 장애물 검사용 실시간 점군 입력 토픽
    'goal_topic': '/parking/goal',  # 지도 기준 주차 목표 입력 토픽
    'initial_pose_topic': '/initialpose',  # 경로 확인 모드에서 RViz 시작 자세를 받을 토픽
    'mode_topic': '/cmd_vel',  # 기존 JOY 수동·자동·긴급정지 명령 입력 토픽
    'map_path': '',  # NDT와 동일한 저장 PCD 지도 경로
    'control_rate': 10.0,  # 주차 추종 계산 주기(Hz)
    'enable_drive': False,  # 기존 AUTO 토픽으로 차량 명령을 발행할지 여부
    'planning_only': False,  # RViz 시작·목표 자세로 경로만 생성하며 주행 출력은 차단
    'require_auto_mode': False,  # 계산 단계에서도 JOY의 AUTO 상태를 요구할지 여부
    'pose_timeout': 0.5,  # 차량 위치 입력의 최대 지연(초)
    'ndt_status_timeout': 0.8,  # NDT 상태 수신의 최대 지연(초)
    'cloud_timeout': 0.5,  # 실시간 점군 입력의 최대 지연(초)
    'ndt_confirmations': 3,  # 주행 허용 전 필요한 연속 정상 NDT 결과 수
    'ndt_max_fitness': 0.4,  # 정상 NDT로 인정할 최대 평균 제곱 정합 오차(m²)
    'ndt_min_overlap': 0.6,  # 정상 NDT로 인정할 최소 점군 겹침 비율(0~1)
    'mode_timeout': 0.5,  # JOY 모드 명령의 최대 수신 지연(초)
    'command_timeout': 0.3,  # 주차 명령과 상태의 최대 수신 지연(초)
    'ground_mode': 'local',  # 지면 높이 방식: local은 주변 점 추정, flat은 고정 높이
    'ground_z': -0.88,  # flat 방식에서만 사용하는 지도 기준 지면 높이(m)
    'ground_radius': 1.0,  # 주변 지면 높이를 찾는 수평 검색 반경(m)
    'ground_percentile': 15.0,  # 주변 점 높이의 지면 추정 분위수(%)
    'min_ground_points': 5,  # 주변 지면 높이를 추정할 최소 점 개수
    'obstacle_min_height': 0.08,  # 지면 대비 이 높이 이상의 점을 장애물로 사용(m)
    'obstacle_max_range': 4.0,  # 실시간 장애물 검색의 최대 수평 거리(m)
    'min_cloud_points': 20,  # 유효 점군으로 인정할 최소 원시 점 개수
    'forward_speed_mps': 0.2,  # 전진 추종의 기준 물리 속도(m/s)
    'reverse_speed_mps': 0.15,  # 후진 추종 속도의 크기(m/s)
    'lookahead': 0.7,  # 전진 경로에서 목표점을 앞서 보는 거리(m)
    'endpoint_distance': 0.1,  # 구간 끝에서 정지를 시작할 위치 오차(m)
    'gear_dwell': 2.0,  # 전후진 전환 전에 0 명령을 유지하는 시간(초)
    'final_dwell': 2.0,  # 최종 오차 검사 전에 0 명령을 유지하는 시간(초)
    'longitudinal_tolerance': 0.15,  # 주차 완료에 허용할 목표 방향 종방향 오차(m)
    'lateral_tolerance': 0.1,  # 주차 완료에 허용할 목표 방향 횡방향 오차(m)
    'yaw_tolerance_deg': 8.0,  # 주차 완료에 허용할 차체 방향 오차(도)
    'path_error_limit': 0.75,  # 추종을 중단할 경로 이탈 거리(m)
    'reverse_observable': False,  # 실제 후방 장애물 관측이 확보된 경우에만 후진 허용
    'enable_retry': False,  # 최종 주차 오차가 클 때 한 번만 재주차할지 여부
    'retry_min_forward': 0.4,  # 재주차에서 후진 전에 반드시 전진할 최소 거리(m)
    'retry_max_reverse': 3.0,  # 재주차에서 허용할 총 후진 거리(m)
    'planner_time_limit': 10.0,  # 한 번의 경로 탐색 시간 한도(초)
    'planner_max_expansions': 40000,  # 한 번의 경로 탐색에서 확장할 최대 상태 수
    'primitive_length': 0.25,  # 차량 운동 후보 한 단계의 이동 거리(m)
    'collision_substeps': 5,  # 운동 후보 한 단계 안에서 검사할 충돌 표본 수
    'xy_resolution': 0.15,  # 탐색 상태를 묶는 위치 격자 간격(m)
    'yaw_resolution_deg': 5.0,  # 탐색 상태를 묶는 방향 격자 간격(도)
    'path_spacing': 0.05,  # 최종 경로의 위치 표본 간격(m)
    'distance_cost': 1.0,  # 주행 거리 비용 가중치
    'steering_cost': 0.35,  # 조향 크기 비용 가중치
    'steering_change_cost': 0.5,  # 조향 변화량 비용 가중치
    'reverse_cost': 1.7,  # 후진 거리의 추가 비용 가중치
    'gear_change_cost': 2.0,  # 전후진 전환 비용 가중치
    'obstacle_cost': 1.5,  # 장애물 근접 비용 가중치
    'output_calibrated': False,  # 실측 속도·PWM 관계와 실제 주행·조향 부호의 확인 여부
    'speed_knots_mps': [0.0],  # 실측 속도 보정표의 오름차순 속도 목록(m/s), 첫 값은 0
    'forward_pwm_knots': [0.0],  # 각 실측 속도에 대응하는 전진 PWM 크기 목록, 첫 값은 0
    'reverse_pwm_knots': [0.0],  # 각 실측 속도에 대응하는 후진 PWM 크기 목록, 첫 값은 0
    'forward_input_sign': -1.0,  # 기존 AUTO 입력에서 실제 전진을 만드는 부호(+1 또는 -1)
    'steering_input_sign': 0.0,  # 좌조향 양수 계산값을 기존 조향 입력으로 바꾸는 부호, 0은 미확인
    'max_pwm': 150.0,  # 기존 Arduino 제한 이하로 적용할 PWM 크기 상한(최대 150)
}

# ROS 파라미터 조회에서도 동일한 한국어 설명을 제공합니다.
DESCRIPTIONS = {
    'wheel_base': '앞뒤 바퀴 축 사이 거리(m)',
    'vehicle_width': '차체 전체 폭(m)',
    'vehicle_length': '차체 전체 길이(m)',
    'rear_overhang': '후륜축에서 뒤 범퍼까지 거리(m)',
    'vehicle_height': '지면에서 차량 최고점까지 높이(m)',
    'base_height': '지면에서 후륜축 중심까지 높이(m)',
    'max_steer_deg': '계획과 추종에 공통 적용하는 조향 한계(도)',
    'collision_margin': '차체 바깥에 추가하는 충돌 여유(m)',
    'map_frame': '저장 지도와 목표 위치의 기준 좌표계',
    'base_frame': '후륜축 중심의 차량 좌표계',
    'pose_topic': 'NDT가 발행하는 차량 위치 입력 토픽',
    'ndt_status_topic': 'NDT 정합 상태 입력 토픽',
    'cloud_topic': '현재 장애물 검사용 실시간 점군 입력 토픽',
    'goal_topic': '지도 기준 주차 목표 입력 토픽',
    'initial_pose_topic': '경로 확인 모드에서 RViz 시작 자세를 받을 토픽',
    'mode_topic': '기존 JOY 수동·자동·긴급정지 명령 입력 토픽',
    'map_path': 'NDT와 동일한 저장 PCD 지도 경로',
    'control_rate': '주차 추종 계산 주기(Hz)',
    'enable_drive': '기존 AUTO 토픽으로 차량 명령을 발행할지 여부',
    'planning_only': 'RViz 시작·목표 자세로 경로만 생성하며 주행 출력은 차단',
    'require_auto_mode': '계산 단계에서도 JOY의 AUTO 상태를 요구할지 여부',
    'pose_timeout': '차량 위치 입력의 최대 지연(초)',
    'ndt_status_timeout': 'NDT 상태 수신의 최대 지연(초)',
    'cloud_timeout': '실시간 점군 입력의 최대 지연(초)',
    'ndt_confirmations': '주행 허용 전 필요한 연속 정상 NDT 결과 수',
    'ndt_max_fitness': '정상 NDT로 인정할 최대 평균 제곱 정합 오차(m²)',
    'ndt_min_overlap': '정상 NDT로 인정할 최소 점군 겹침 비율(0~1)',
    'mode_timeout': 'JOY 모드 명령의 최대 수신 지연(초)',
    'command_timeout': '주차 명령과 상태의 최대 수신 지연(초)',
    'ground_mode': '지면 높이 방식: local은 주변 점 추정, flat은 고정 높이',
    'ground_z': 'flat 방식에서만 사용하는 지도 기준 지면 높이(m)',
    'ground_radius': '주변 지면 높이를 찾는 수평 검색 반경(m)',
    'ground_percentile': '주변 점 높이의 지면 추정 분위수(%)',
    'min_ground_points': '주변 지면 높이를 추정할 최소 점 개수',
    'obstacle_min_height': '지면 대비 이 높이 이상의 점을 장애물로 사용(m)',
    'obstacle_max_range': '실시간 장애물 검색의 최대 수평 거리(m)',
    'min_cloud_points': '유효 점군으로 인정할 최소 원시 점 개수',
    'forward_speed_mps': '전진 추종의 기준 물리 속도(m/s)',
    'reverse_speed_mps': '후진 추종 속도의 크기(m/s)',
    'lookahead': '전진 경로에서 목표점을 앞서 보는 거리(m)',
    'endpoint_distance': '구간 끝에서 정지를 시작할 위치 오차(m)',
    'gear_dwell': '전후진 전환 전에 0 명령을 유지하는 시간(초)',
    'final_dwell': '최종 오차 검사 전에 0 명령을 유지하는 시간(초)',
    'longitudinal_tolerance': '주차 완료에 허용할 목표 방향 종방향 오차(m)',
    'lateral_tolerance': '주차 완료에 허용할 목표 방향 횡방향 오차(m)',
    'yaw_tolerance_deg': '주차 완료에 허용할 차체 방향 오차(도)',
    'path_error_limit': '추종을 중단할 경로 이탈 거리(m)',
    'reverse_observable': '실제 후방 장애물 관측이 확보된 경우에만 후진 허용',
    'enable_retry': '최종 주차 오차가 클 때 한 번만 재주차할지 여부',
    'retry_min_forward': '재주차에서 후진 전에 반드시 전진할 최소 거리(m)',
    'retry_max_reverse': '재주차에서 허용할 총 후진 거리(m)',
    'planner_time_limit': '한 번의 경로 탐색 시간 한도(초)',
    'planner_max_expansions': '한 번의 경로 탐색에서 확장할 최대 상태 수',
    'primitive_length': '차량 운동 후보 한 단계의 이동 거리(m)',
    'collision_substeps': '운동 후보 한 단계 안에서 검사할 충돌 표본 수',
    'xy_resolution': '탐색 상태를 묶는 위치 격자 간격(m)',
    'yaw_resolution_deg': '탐색 상태를 묶는 방향 격자 간격(도)',
    'path_spacing': '최종 경로의 위치 표본 간격(m)',
    'distance_cost': '주행 거리 비용 가중치',
    'steering_cost': '조향 크기 비용 가중치',
    'steering_change_cost': '조향 변화량 비용 가중치',
    'reverse_cost': '후진 거리의 추가 비용 가중치',
    'gear_change_cost': '전후진 전환 비용 가중치',
    'obstacle_cost': '장애물 근접 비용 가중치',
    'output_calibrated': '실측 속도·PWM 관계와 실제 주행·조향 부호의 확인 여부',
    'speed_knots_mps': '실측 속도 보정표의 오름차순 속도 목록(m/s), 첫 값은 0',
    'forward_pwm_knots': '각 실측 속도에 대응하는 전진 PWM 크기 목록, 첫 값은 0',
    'reverse_pwm_knots': '각 실측 속도에 대응하는 후진 PWM 크기 목록, 첫 값은 0',
    'forward_input_sign': '기존 AUTO 입력에서 실제 전진을 만드는 부호(+1 또는 -1)',
    'steering_input_sign': '좌조향 양수 계산값을 기존 조향 입력으로 바꾸는 부호, 0은 미확인',
    'max_pwm': '기존 Arduino 제한 이하로 적용할 PWM 크기 상한(최대 150)',
}


def declare(node):
    from rcl_interfaces.msg import ParameterDescriptor
    config = {key: node.declare_parameter(
        key, value, descriptor=ParameterDescriptor(description=DESCRIPTIONS[key])).value
        for key, value in DEFAULTS.items()}
    if config['planning_only'] and config['enable_drive']:
        raise ValueError('경로 확인 모드에서는 실차 출력을 활성화할 수 없습니다')
    return config


def vehicle_from(config):
    return Vehicle(config['wheel_base'], config['vehicle_width'], config['vehicle_length'],
                   config['rear_overhang'], config['vehicle_height'], config['base_height'],
                   math.radians(config['max_steer_deg']), config['collision_margin'])


def gate_from(config):
    return LocalizationGate(config['pose_timeout'], config['ndt_status_timeout'],
                            config['cloud_timeout'], config['ndt_confirmations'],
                            config['ndt_max_fitness'], config['ndt_min_overlap'])


def configure_planner(planner, config):
    names = {
        'planner_time_limit': 'time_limit', 'planner_max_expansions': 'max_expansions',
        'primitive_length': 'step', 'collision_substeps': 'substeps',
        'xy_resolution': 'xy_resolution', 'path_spacing': 'output_spacing',
        'distance_cost': 'w_length', 'steering_cost': 'w_steer',
        'steering_change_cost': 'w_steer_change', 'reverse_cost': 'w_reverse',
        'gear_change_cost': 'w_gear_change', 'obstacle_cost': 'w_obstacle',
    }
    for source, target in names.items():
        value = config[source]
        if not math.isfinite(value) or value <= 0:
            raise ValueError(source + ' must be positive')
        setattr(planner, target, value)
    planner.yaw_resolution = math.radians(config['yaw_resolution_deg'])
    if planner.yaw_resolution <= 0:
        raise ValueError('yaw_resolution_deg must be positive')
