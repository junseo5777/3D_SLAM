# `velodyne_slam` 구현 기록

이 패키지는 Velodyne VLP-16과 MicroStrain 3DM-CV7-AHRS를 사용한 FAST-LIO2 mapping 및 PCL NDT localization backend입니다. 초기 전처리 scaffold에서 출발해 실제 지도 누적, PCD 저장, 차량 기준 위치 출력과 추적 복구 구조를 구현했습니다.

## 패키지의 역할

| 구성 | 역할 |
|---|---|
| `lio.launch.py` | 센서 장착값과 FAST-LIO2를 mapping/localization 모드에 맞게 연결 |
| `map_builder_node.cpp` | world-frame 점군을 voxel map으로 누적하고 PCD 저장 |
| `ndt_localization_node.cpp` | 저장 PCD와 현재 body cloud를 비동기 NDT로 정합 |
| `ndt_matcher.cpp` | PCL NDT 실행과 fitness·overlap 계산 |
| `mapping.launch.py` | FAST-LIO2와 map builder로 구성한 mapping 진입점 |
| `localization.launch.py` | FAST-LIO2와 NDT로 구성한 localization 진입점 |

## 센서 입력과 좌표계

| 항목 | 현재 구성 |
|---|---|
| LiDAR | VLP-16, 16채널, 600 rpm, 약 9.9 Hz |
| Point cloud | `/velodyne_points`, `x/y/z/intensity/ring/time` |
| IMU | 3DM-CV7-AHRS `/imu/data`, 약 100 Hz |
| Mapping TF | FAST-LIO2 `map → imu_link` |
| Localization TF | NDT `map → odom`, FAST-LIO2 `odom → imu_link` |
| Vehicle pose | `/localization/pose`, 카트 뒷바퀴 축 중심 `base_link` |

LiDAR와 IMU의 장착 관계는 `lidar_imu_calibration.yaml`, 카트와 LiDAR의 관계는 `cart.yaml`에서 관리합니다. 수동 축 검사와 실제 장착 높이를 반영해 LiDAR–IMU 회전을 Y축 180°, translation을 `[0, 0, 0.05]` m로 구성했습니다.

FAST-LIO2는 mapping에서 `map`, localization에서 `odom`을 world frame으로 사용합니다. NDT가 계산한 전역 보정은 `map → odom`에만 적용하며 FAST-LIO2 내부 상태에는 다시 넣지 않습니다.

## Mapping 설계

Map builder는 `/fast_lio/cloud_world`의 각 점을 0.2 m voxel에 누적하고 voxel마다 대표점 하나를 유지합니다. 최대 100만 voxel까지 저장하며 변경된 지도는 `/pointcloud_map`으로 발행합니다.

동적 잔상을 줄이기 위해 각 scan에서 다음 순서로 free-space를 판정합니다.

1. scan timestamp와 0.03초 이내인 FAST-LIO odometry를 찾습니다.
2. IMU pose와 장착 translation으로 world-frame LiDAR 원점을 계산합니다.
3. 원점에서 hit point 앞 0.4 m까지 광선을 따라갑니다.
4. 광선이 통과했지만 현재 scan에서 hit되지 않은 기존 voxel을 miss로 기록합니다.
5. 처리 scan 기준 5회 연속 miss인 voxel만 삭제합니다.
6. 재관측, 광선 미통과, 대응 odometry 부재가 발생하면 연속 횟수를 초기화합니다.

광선은 4개 중 하나를 검사하도록 간추려 동적 잔상 제거와 연산량을 함께 관리했습니다.

지도 저장은 임시 PCD를 먼저 쓴 뒤 rename하는 방식으로 기존 파일을 교체합니다. 빈 지도 저장을 거부하고, 수동 저장 뒤 변화가 없으면 종료 시 같은 내용을 다시 저장하지 않습니다.

## Localization 설계

### 정상 추적

현재 설정은 NDT를 최대 약 5 Hz로 실행합니다. 0.5 m로 다운샘플링한 body cloud와 예상 위치 주변 30 m의 부분 지도를 사용하며, NDT resolution은 1.0 m입니다. 결과가 수렴했더라도 다음 조건을 모두 통과해야 채택합니다.

- fitness ≤ 0.4
- overlap ≥ 0.6
- initial guess 대비 translation ≤ 2.0 m
- initial guess 대비 rotation ≤ 28.65°
- 최대 반복 횟수에 도달하지 않고 pose 행렬이 유한할 것

fitness는 최근접점 거리 기반 점수, overlap은 1 m 이내에 지도점이 있는 source 점 비율로 계산해 정합 품질 검사에 함께 사용합니다.

### 초기 자세

시작 직후에는 `waiting_for_initial_pose` 상태입니다. `/initialpose`는 지도상의 `base_link` 위치와 카트 전방을 뜻합니다. 입력 즉시 클릭한 위치를 표시하고, 클릭 주변 ±1 m·±15°에서 NDT 후보를 찾습니다.

초기 후보는 이동을 보상한 correction이 0.3 m·5° 안에서 3회 연속 일치할 때 채택합니다. 실패가 반복되면 fitness, overlap, 반복 횟수와 탐색 범위를 최대 6단계까지 완화합니다. 확정 pose가 목표 correction의 0.1 m·2° 안에 들어오면 정상 tracking으로 전환합니다.

### 추적 단절과 복구

정상 NDT가 5회 연속 실패하거나 마지막 성공 후 2초가 지나면 최근 `map → odom`을 유지하며 재탐색합니다.

| 추적 단절 후 시간 | 단계 | X/Y 탐색 | Yaw 탐색 | 판정 조건 |
|---:|---:|---:|---:|---|
| 0–3초 | 0-A | ±0.25 m | ±2.5° | strict 유지 |
| 3–8초 | 0-B | ±0.5 m | ±5° | strict 유지 |
| 8–11초 | 1 | ±1.417 m | ±11.667° | 1/6 완화 |
| 11–14초 | 2 | ±2.333 m | ±18.333° | 2/6 완화 |
| 14–17초 | 3 | ±3.25 m | ±25° | 3/6 완화 |
| 17–20초 | 4 | ±4.167 m | ±31.667° | 4/6 완화 |
| 20–23초 | 5 | ±5.083 m | ±38.333° | 5/6 완화 |
| 23초 이후 | 6 | ±6 m | ±45° | 최종 완화 유지 |

8초 이후에는 fitness 상한, overlap 하한, 보정 허용량, 반복 횟수와 지역 지도 반경도 같은 비율로 완화합니다. 한 작업은 중심 guess를 포함해 최대 5개 후보와 150 ms의 추가 탐색 예산을 사용합니다. 남은 후보 cursor는 다음 최신 scan으로 넘깁니다.

복구 결과도 바로 적용하지 않고 3회 연속 일관성을 확인합니다. 완화 단계에서 복구했다면 다음 단계의 더 엄격한 조건을 다시 3회 통과할 때마다 recovery level을 하나 낮춥니다.

### 출력 연속성

NDT는 `std::async` 작업에서 실행하고 callback thread는 odometry, TF와 pose 발행을 계속합니다. 처리 대기열 대신 최신 body cloud 한 장만 유지합니다. 오래된 작업은 generation과 timestamp로 폐기합니다.

확정된 `map → odom` correction은 최대 0.5 m/s와 15°/s로 접근시킵니다. 이 제한은 NDT가 만든 전역 보정에만 적용되며 FAST-LIO가 측정한 실제 이동은 즉시 pose에 반영됩니다.

## 예외 처리

### Launch와 설정

- mapping/localization mode와 boolean launch 인자의 형식을 검사합니다.
- localization 시작 전에 지정한 PCD 파일이 실제로 존재하는지 확인합니다.
- 외부 보정 translation 3개, rotation 9개와 time offset이 모두 유한한지 확인합니다.
- 회전행렬의 행이 서로 직교하고 determinant가 +1인지 검사합니다.
- 센서 view 각도와 NDT 수치 파라미터의 유한성, 양수 범위 및 상·하한 관계를 시작 시 확인합니다.
- 공통 pipeline의 주요 child process가 종료되면 launch도 함께 종료해 일부 노드만 남는 상태를 방지합니다.

### Point cloud와 odometry

- VLP-16 point의 NaN·Inf, 범위를 벗어난 ring과 음수 time을 제거합니다.
- point time을 stable sort하여 deskew 입력 순서를 일정하게 유지합니다.
- odometry와 cloud frame이 예상한 `map`·`odom`·`imu_link` 관계인지 확인합니다.
- cloud와 odometry timestamp 차이가 0.03초 이내일 때만 하나의 NDT 또는 map update 입력으로 묶습니다.
- 중복되거나 역행한 timestamp를 거부하고, odometry 시간이 초기화되면 buffer와 tracking state도 함께 초기화합니다.
- 현재 시각보다 미래이거나 허용 지연 시간을 넘긴 cloud는 처리하지 않습니다.

### Map builder

- 유한하지 않거나 voxel index 범위를 과도하게 벗어나는 점을 건너뜁니다.
- LiDAR 원점을 계산할 때 quaternion, translation과 변환 결과를 검사합니다.
- scan에 대응하는 odometry가 없으면 해당 scan의 free-space 삭제를 수행하지 않습니다.
- 최대 voxel 수에 도달하면 새 voxel 추가를 멈추고 상태 토픽으로 알립니다.
- 빈 지도 저장을 거부하고 PCD 쓰기 또는 rename 실패를 `/mapping/status`로 전달합니다.
- 기존 지도를 교체할 때 임시 파일에 먼저 기록한 뒤 rename합니다.

### NDT localization

- PCD를 읽은 뒤 NaN·Inf를 제거하고 최소 point 수를 만족하는지 확인합니다.
- quaternion, initial guess와 NDT 결과 행렬이 모두 유한한지 검사합니다.
- source/target point 부족, 비수렴, 최대 반복 도달 결과를 거부합니다.
- 수렴 결과도 fitness, overlap, translation과 rotation 보정량을 모두 통과해야 accepted 상태가 됩니다.
- 초기 자세나 복구 후보는 한 번의 결과로 확정하지 않고 차량 이동을 보상한 correction이 3회 연속 일관될 때 적용합니다.
- 후보가 실패하거나 서로 다르거나 timestamp 간격이 3초를 넘으면 확인 횟수를 초기화합니다.

### 비동기 작업

- NDT 작업 중에는 callback thread가 최신 odometry와 pose를 계속 발행합니다.
- 처리 대기열은 누적하지 않고 최신 body cloud 한 장만 보관합니다.
- 초기 자세 재입력이나 시간 초기화 이전에 시작된 작업은 generation 번호로 구분해 폐기합니다.
- 측정 시각보다 2초 이상 오래됐거나 미래 시각인 NDT 결과는 pose에 적용하지 않습니다.
- 작업 thread의 예외를 잡아 `registration_exception` 상태로 전달하고 후보 확인 상태를 초기화합니다.

## 주요 파일

```text
config/lidar_imu_calibration.yaml  # LiDAR–IMU 장착 관계
config/cart.yaml                   # base_link–LiDAR 장착 관계
config/map_builder.yaml            # voxel map과 free-space 설정
config/ndt.yaml                    # NDT 판정·탐색·복구 설정
launch/lio.launch.py               # mapping/localization 공통 구성
src/map_builder_node.cpp           # 지도 누적과 PCD 저장
src/ndt_localization_node.cpp      # 비동기 NDT와 상태 머신
src/ndt_matcher.cpp                # NDT 실행과 품질 계산
```
