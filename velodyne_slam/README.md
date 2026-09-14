# VLP-16 + CV7: FAST-LIO2 매핑 / NDT 위치추정

ROS 2 Humble. 디렉터리는 `3d_slam`, ROS 패키지 이름은 `velodyne_slam`이다.

## 구현한 기능

- `mapping.launch.py`: 라이다·IMU → FAST-LIO2 → voxel 지도 누적 → 종료 시 PCD 자동 저장.
- `localization.launch.py`: FAST-LIO2의 deskew 점군·odometry + 저장 PCD → PCL NDT → 지도 기준 위치 보정.
- `sensors.launch.py`: 센서만 실행. 기존 인터페이스 유지.
- `preprocessing.launch.py`: 이전 XYZI 전처리 기능. SLAM 입력으로 사용하지 않는다.
- NDT는 정합 수렴, fitness, overlap, 예측 대비 보정량을 검사한다. 초기 위치를 받기 전에는
  전역 위치를 발행하지 않는다. 추적이 끊기면 최근 위치에서 10초간 기존 조건으로 재탐색하고,
  이후에는 조건과 탐색 범위를 단계적으로 완화하면서 성공할 때까지 계속 시도한다.
- 루프 클로저, pose graph 최적화, 지도 전체 범위의 전역 재위치추정은 구현하지 않았다.

## 센서에 맞춘 설정

| 항목 | 설정 |
|---|---|
| 라이다 | VLP-16, 16채널, 600 rpm / 약 9.9 Hz |
| 라이다 통신 | `192.168.1.201:2368`, PC 유선 `192.168.1.100/24` |
| 점군 | `/velodyne_points`, `x/y/z/intensity/ring/time`, Reliable 구독 |
| 점별 time | 초 단위 → FAST-LIO `preprocess.timestamp_unit: 0` |
| 스캔 timestamp | 드라이버 `timestamp_first_packet: true` |
| IMU | MicroStrain 3DM-CV7-AHRS, 일련번호 `6286.167183`, USB by-id / 115200 baud |
| LIO IMU 입력 | `/imu/data`, 약 100 Hz, m/s² 및 rad/s, ENU 변환된 센서 축 |
| 내부 필터 출력 | `/ekf/imu/data`, 약 25 Hz. FAST-LIO 입력으로 사용하지 않음 |
| 시간 기준 | 호스트 시각. 하드웨어 동기화 검증 결과를 의미하지 않음 |

FAST-LIO는 각속도·가속도를 사용하며 CV7의 절대 yaw quaternion에 의존하지 않는다.
점군 전처리에서 비유한 점/잘못된 ring/time을 제거하고 시간순으로 정렬한다.
현재 FAST-LIO 설정에서 point_filter_num=2는 연산량 절감을 위한 점 선택 간격이며,
라이다 자체가 32채널로 바뀌는 것은 아니다.

설정 파일에서 사람이 입력하는 각도 파라미터는 모두 도 단위를 사용한다. 라이다 사용 영역은
`lidar.yaml`의 `view_direction_degree`, `view_width_degree`에서 설정하고 NDT 회전 허용값은
`ndt.yaml`의 이름이 `_degree`로 끝나는 항목에서 설정한다. 차량 장착 TF의 roll·pitch·yaw는
`sensor_mounts.yaml`의 `rpy_degrees`에 입력한다. launch와 NDT 노드가 내부 라디안 값으로 변환한다.

## 장착 보정: 실제 매핑 전에 필요

`config/lidar_imu_calibration.yaml`의 값은 아직 미측정이다.

```text
p_imu = rotation * p_lidar + translation
```

- translation: IMU 원점에서 본 라이다 원점, m.
- rotation: 라이다 좌표를 `/imu/data`의 출력 축으로 옮기는 3×3 회전행렬, row-major.
- IMU의 native NED 축과 ROS로 변환된 축을 혼동하지 않는다.
- time_offset_lidar_to_imu: FAST-LIO가 IMU timestamp에서 빼는 보정량, 초.
- 센서가 강체로 고정되어 있어야 한다. USB/IP 조회나 정지 중 중력값만으로 전체 외부 보정을 알아낼 수 없다.

측정/캘리브레이션 후 값을 입력하고 `calibrated: true`로 설정한다. 차량 장착 전 시험에서는
`lidar_imu_calibration.yaml`의 `allow_uncalibrated: true`로 진단 실행을 허용한다. 그 결과를
검증된 지도/위치로 사용해서는 안 된다. 온라인 외부 보정 추정도 기본 비활성화했다.

FAST-LIO 설정은 `config/fast_lio.yaml`, 센서 설정은 `config/lidar.yaml`, `config/imu.yaml`이다.
noise covariance는 시작용 값이며 실측 노이즈 동정값이 아니다.
Mapping 시작 후 `mapping.imu_init_duration`(현재 3초) 동안 IMU 평균으로 중력 방향과
자이로 bias를 초기화한다. 터미널에 `IMU Initial Done`이 출력될 때까지 센서를 움직이지 않는다.

## 빌드

반드시 workspace 루트에서 실행한다. `src`에서 빌드하면 `src/install`에 별도 결과가 생겨
`scin`이 불러오는 환경과 달라진다.

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
MAKEFLAGS=-j1 colcon build --base-paths src/3d_slam src/fast_lio src/velodyne src/microstrain_inertial --packages-up-to velodyne_slam --symlink-install --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
```

필수 시스템 의존성은 현재 환경에 설치되어 있다. 새 환경에서는 위 대상의 package.xml을
기준으로 rosdep을 사용한다. 합성 검증 스크립트에는 numpy와 psutil도 필요하다.

## 센서만 실행

```bash
ros2 launch velodyne_slam sensors.launch.py
# 라이다만: imu_enabled:=false
# IMU만: lidar_enabled:=false
```

현재 유선 IP가 없다면 먼저 터미널에서 다음을 실행한다. 이미 같은 주소가 있으면 생략한다.

```bash
sudo ip addr add 192.168.1.100/24 dev enx00e04c680761
ping -c 3 192.168.1.201
```

이 IP는 임시 설정이다. launch가 네트워크 주소를 변경하지 않는다.

## 매핑

현재 지도 경로는 launch 코드에
`/home/junseo5777/ros2_ws/src/3d_slam/maps/cart_test_01.pcd`로 설정되어 있다.
실행 옵션 대신 코드의 기본값을 수정해서 사용한다.

```bash
ros2 launch velodyne_slam mapping.launch.py
```

`Ctrl+C`로 mapping launch를 종료하면 현재 지도를 자동 저장한다. 같은 파일이 이미 있으면
새 지도로 자동 교체한다. 파일명은 `launch/lio.launch.py` 상단의 `MAP_FILENAME` 한 줄에서 바꾼다.
중간에 지도를 저장하고 싶을 때는 다음 서비스를 사용할 수 있다.

지도 누적기는 현재 라이다 원점에서 새 측정점까지의 광선 경로를 빈 공간으로 처리한다.
원점은 FAST-LIO의 IMU 위치에 IMU 자세로 회전시킨 장착 translation을 더해서 계산한다.
장착 translation은 `lidar_imu_calibration.yaml`에서 FAST-LIO와 지도 누적기에 함께 전달된다.
이는 스캔 시점의 단일 원점이며, 스캔 중 포인트별 센서 이동 궤적까지 복원하는 방식은 아니다.
기존 voxel보다 먼 배경이 `clearing_miss_threshold` 프레임 연속 관측되면 중간 voxel을 삭제해
움직이는 물체의 잔상을 줄인다. 끝점 앞 `clearing_endpoint_margin` 구간은 벽 표면 보호를 위해
지우지 않는다. 하늘이나 열린 공간처럼 더 먼 반사점이 전혀 없는 방향은 이 방식만으로 지울 수 없다.
연속 여부는 지도 누적기가 처리한 스캔 순서로 판정한다. 중간 스캔에서 해당 voxel을 통과한
광선이 없거나 대응 odometry가 없으면 다음 빈 공간 관측은 1회부터 다시 시작한다.
같은 voxel에 점이 다시 관측되면 즉시 횟수를 초기화한다. 광선 간추림(`clearing_ray_stride`)으로
관측이 끊길 수 있으므로 5회 설정이 반드시 0.5초 내 삭제를 의미하지는 않는다.

```bash
ros2 service call /save_map std_srvs/srv/Trigger '{}'
```

수동 저장 성공 후 추가 포인트가 없다면 종료 시 중복 저장하지 않는다.
지도는 최초 IMU 자세·위치로 초기화된 `map` 좌표계의 XYZI PCD다.
지도 재사용 시 같은 센서 장착 보정값을 사용한다.

## 위치추정

```bash
ros2 launch velodyne_slam localization.launch.py
```

RViz Fixed Frame을 `map`으로 설정하고 `/pointcloud_map`을 Reliable / Transient Local로
표시한다. `2D Pose Estimate`로 지도상 초기 위치와 방향을 `/initialpose`에 지정한다.
클릭 위치는 카트 뒷바퀴 중심(`base_link`)의 지도상 위치이며 화살표는 카트 전방을 뜻한다.
roll·pitch 기울기는 FAST-LIO 측정을 유지한다. 높이가 초기 지도 원점과 다르면
`/initialpose`의 z도 적절히 지정해야 한다.
수동 초기 입력에서는 클릭 위치 기준 X/Y 각각 ±1 m와 yaw ±15도 후보를 탐색한다. 범위는
`config/ndt.yaml`의 `initial_search_xy`, `initial_search_yaw_degree`에서 바꿀 수 있다.
초기 위치를 받으면 NDT 결과를 기다리지 않고 해당 위치를 `/localization/pose`에 즉시 발행한다.
초기 NDT가 실패해도 표시한 위치를 유지하며 계속 재시도한다. 3회 실패할 때마다 겹침률·fitness·
반복 횟수·주변 탐색 범위를 한 단계씩 완화하고, 6단계 한계에 도달한 뒤에도 그 조건으로 계속
시도한다. 초기·복구 후보는 FAST-LIO 이동을 고려한 지도 보정값이 3회 연속 일관될 때 적용한다.
후보 사이 위치 차이는 0.3 m, 회전 차이는 5도 이내여야 하며, 실패·불일치·3초 초과 간격에서는
검증을 다시 시작한다. 검증 중에는 초기 표시 위치와 FAST-LIO 이동으로 위치 발행을 유지한다.
지도 보정은 오도메트리 수신마다 최대 0.5 m/s·15도/s로 반영한다. 작은 오차는 즉시 따라잡고
큰 오차는 나누어 반영하며 실제 이동 속도는 제한하지 않는다. 초기 잔여 오차가 0.1 m·2도
이내이면 정상 추적으로 전환한다. 관련 값은 `ndt.yaml`에서 조정한다.
지도 전체를 탐색해 최초 위치를 찾는 전역 탐색은 수행하지 않는다.

`/localization/status`의 `tracking`을 확인한다. 추적 단절 후에는
`relocalizing_continuously`와 현재 `recovery_level`이 표시된다. 첫 10초가 지나면 5초마다
최대 6단계까지 조건을 완화한다. X/Y 탐색 범위는 ±6 m, yaw는 ±45도, 지역 지도 반경은
60 m까지 커진다. 최대 단계에서도 위치추정을 중단하지 않는다. 복구 후에는
`tightening_after_recovery`가 표시되고 다음 단계의 더 엄격한 기준과 더 작은 지역 지도에서도
연속 3회 정합에 성공해야 한 단계씩 조건을 복원한다. 실패하면 연속 성공 횟수를 초기화한다.
모든 단계가 복원되면 `strict_tracking_restored`가 표시된다. 상태와 TF timestamp를 함께
확인하며 이전에 캐시된 TF만으로 추적 성공을 판단하지 않는다.

NDT는 별도 작업 스레드에서 실행하며 ROS 콜백은 오도메트리와 위치·TF 발행을 계속 처리한다.
대기 점군은 최신 한 장만 유지한다. 예상 위치에서 먼저 정합하고 실패하면 가까운 후보부터
확인한다. 한 작업은 기본 최대 5개 후보와 150 ms의 추가 탐색 예산을 사용하며, 남은 후보는
다음 최신 스캔에서 이어 확인한다. 조건 복원 검사는 별도로 최대 1회 수행한다. 진행 중인
PCL 정합 한 번은 중간에 취소하지 않으므로 150 ms는 전체 작업의 엄격한 시간 상한은 아니다.
측정 시각에서 2초가 지난 결과나 초기 위치 재입력·오도메트리 시간 초기화 이전 결과는 폐기한다.
NDT 결과 시각의 오래된 Pose를 재발행하지 않고 최신 오도메트리에 보정을 적용한다.

## TF와 토픽

| 모드 | TF |
|---|---|
| Mapping | FAST-LIO `map → imu_link`, 보정값 기반 `imu_link → velodyne` |
| Localization | NDT `map → odom`, FAST-LIO `odom → imu_link`, 보정값 기반 `imu_link → velodyne` |

카트 기준점은 뒷바퀴 축의 좌우 중심인 `base_link`다. `config/cart.yaml`에서 카트 기준
라이다 위치를 `[0.70, 0.0, 0.75]` m로 설정한다. 라이다는 카트와 같은 방향을 향한다고
가정하며 회전은 `lidar_rpy_degrees`에서 도 단위로 입력한다. 기존 라이다–IMU 보정값과
조합한 `imu_link → base_link`를 매핑·위치추정 launch에서 발행한다. 따라서 IMU 아래에
`velodyne`와 `base_link`가 각각 연결되며 TF 부모가 중복되지 않는다.
LIO launch가 센서·카트 장착 TF를 발행하므로 sensors.launch의 별도 장착 TF 발행을
동시에 켜지 않는다. mapping과 localization도 동시에 실행하지 않는다.
FAST-LIO 내부 상태와 지도 좌표계는 기존 IMU 기준을 유지한다. 기존 PCD는 재작성하지 않으며,
예전 지도 원점과 뒷바퀴 중심 높이가 다르면 초기 위치의 z도 그 지도 좌표에 맞춰야 한다.

| 토픽 | 내용 |
|---|---|
| `/fast_lio/odometry` | LIO 위치·자세, 현재 스캔 종료 시각 |
| `/fast_lio/cloud_body` | IMU 프레임의 deskew 점군, NDT 입력 |
| `/fast_lio/cloud_world` | mapping에서는 map, localization에서는 odom 프레임 점군 |
| `/pointcloud_map` | 누적/저장 지도, Reliable + Transient Local |
| `/mapping/status` | 지도 누적 및 저장 상태 |
| `/localization/pose` | 지도 기준 뒷바퀴 중심(base_link) 위치와 카트 전방 방향; 초기·복구 검증 중에는 임시 예측 포함 |
| `/localization/lidar_pose` | 동일한 지도 보정을 적용한 라이다 원점 위치·방향 |
| `/localization/imu_pose` | 같은 지도 보정을 적용한 IMU 위치·방향 |
| `/localization/status` | 정합 상태, fitness, overlap, 반복 수, 처리 시간 |

NDT의 보정은 `map → odom`에만 적용하며 FAST-LIO 내부 상태에 중복 주입하지 않는다.
센서가 별도로 실행 중이면 `start_sensors:=false`를 사용한다.
bag 재생은 `use_sim_time:=true start_sensors:=false`와 `ros2 bag play ... --clock`을 사용한다.
FAST-LIO 점군 구독은 Reliable이므로 bag의 발행 QoS도 Reliable로 맞춘다.
bag을 뒤로 seek하거나 시간을 초기화하면 전체 LIO/지도 노드를 재시작한다.

## 노트북 연산량 설정

- FAST-LIO: 16채널/10 Hz, point_filter_num=2, surface/map voxel=0.3 m, EKF 최대 3회 반복.
- NDT: 최대 2 Hz, 입력 voxel=0.5 m, 반경 30 m 부분 지도, resolution=1 m, 최대 30회 반복.
- 전역 지도: voxel=0.2 m, 최대 100만 voxel. 용량 제한 도달은 상태 토픽에 표시한다.
- NDT 정밀도가 더 필요하면 `ndt_resolution`을 줄이고 `transformation_epsilon`을 조절하되
  지도 밀도와 처리 시간을 함께 검증한다. resolution=1 m로 모든 환경에서 cm급 정확도를 보장하지 않는다.

작은 합성 실내 지도에서 약 9.8 Hz LIO, 프로세스 합계 peak RSS 약 210~230 MB,
평균 CPU 약 0.3코어가 관찰됐다. 입력 5,760점/scan 및 지도 약 1,947점인 정지 실험이며,
실제 29,184 슬롯/scan, 넓은 지도, 움직이는 장비의 성능을 대신하지 않는다.

## 검증과 현재 제한

- 실제 IMU 재측정: 약 100 Hz / 필터 약 25 Hz, 수신 timestamp 중복·역행 없음.
- 이번 재연결에서는 유선 IPv4가 없어 실제 라이다 토픽은 수신하지 못했다.
  이전 세션에서는 VLP-16 패킷과 약 9.9 Hz ROS 점군을 확인했다.
- 합성 입력으로 실제 FAST-LIO 매핑 → PCD 저장 → FAST-LIO+NDT localization을 실행했다.
- NDT 알려진 변환 정합, 지도 밖 초기값 거부, 빈 점군 거부 및 VLP-16 시간/비유한 점 처리 회귀 테스트를 제공한다.
- 실제 이동 매핑, 외부 보정, 시간 보정, 큰 지도 성능 및 실제 위치 정확도는 미검증이다.

```bash
cd ~/ros2_ws
colcon test --base-paths src/3d_slam src/fast_lio --packages-select velodyne_slam fast_lio
colcon test-result --verbose
source install/setup.bash
ROS_DOMAIN_ID=91 python3 src/3d_slam/test/validate_pipeline.py
```

원시 검증 수치는 `docs/`의 JSON을 참고한다.

## 소스 이력

FAST-LIO2 ROS 2 이식본: https://github.com/Ericsii/FAST_LIO_ROS2
commit `2fffc570a25d0df172720bac034fbdb6a13d2162`, ikd-Tree submodule
`e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4`.
로컬 변경: Livox CustomMsg 빌드 선택화(기본 OFF), VLP 입력 유효성/시간 정렬,
Reliable 점군 및 SensorData IMU QoS, world/body 프레임 인자, odometry covariance 발행 순서,
시간 역행 시 병렬 버퍼 정리, VLP 입력 회귀 테스트. upstream 소스와 라이선스를 유지한다.
NDT는 현재 설치된 PCL 구현을 사용한다.
