# macaron_parking

기존 프로젝트를 수정하지 않고 연결하는 ROS 2 Humble 주차 패키지입니다.

```text
기존 fast_lio + velodyne_slam(NDT)
    → /localization/pose (map 기준 후륜축 중심 base_link)
    → PCD Hybrid A* → 경로 + 전진/후진 방향
    → 전진 DWA / 후진 SSP
    → /parking/desired_command (속도 m/s, 조향 rad)
    → 출력 변환 → /erp_writes (PWM, 조향 도)
    → 기존 total_node.py → /erp_write → micro-ROS Arduino
JOY → 기존 teleop_joy.py → /cmd_vel → 기존 total_node.py
```

`fast_lio`, `3d_slam`(패키지 이름 `velodyne_slam`), `2026_KMU`, `macaron8_HL_1`을
수정하지 않습니다. 새 패키지는 기존 localization을 구독하고, 기존 출력부의
AUTO 입력에만 연결합니다. 기본 실행에서는 **/erp_writes 발행자 자체를 만들지 않습니다**.

## 차량과 알고리즘

`config/vehicle.yaml`을 모든 계산부가 공유합니다.

| 제원 | 값 |
|---|---:|
| 축거 / 차폭 | 0.75 / 0.68 m (macaron 코드) |
| 전장 / 뒤 오버행 | 1.30 / 0.24 m (사용자 실측) |
| 앞 오버행 / 후륜축부터 앞 범퍼 | 0.31 / 1.06 m |
| 차 높이 / 지면에서 base_link 높이 | 0.50 / 0.14 m |
| 공통 조향 한계 | ±21° (기존 DWA/SSP), Arduino ±23°는 그대로 |

KMU의 Hybrid A* 탐색·자전거 모델·클로소이드 연결·비용과 DWA/SSP 수식을 복사해
새 패키지 안에서 사용합니다. 원본 패키지 import에 의존하지 않습니다.
competition START/A/B, AMCL, PGM, 자동 복귀, 위치별 후진 허용, 대회 recovery는 제외했습니다.
Hybrid A*는 처음부터 두 방향을 허용하며 탐색은 weighted bidirectional A*입니다.
PCD 교체에 따라 휴리스틱은 Euclidean distance로 분리했습니다. 최적 경로 보장은 없습니다.
시간/확장 한도에 도달하면 실패하고 정지합니다.

PCD는 ASCII와 binary를 지원합니다(현재 map_builder가 저장하는 binary 포함).
`binary_compressed`는 지원하지 않습니다. KD-tree는 XY 공간 검색을 하고,
각 후보 주변의 실제 XYZ 점을 지면 상대 높이로 필터링한 뒤 전체 사각 footprint와 검사합니다.
상태는 `(x,y,yaw)`이며 primitive 중간과 연결 경로도 검사합니다.
local ground는 주변 점의 낮은 높이 분위수 근사입니다. 지면이 관측되지 않은 영역은
막고, 경사/낮은 구조물/다층 환경에는 실제 지도로 별도 검증해야 합니다.
flat 모드는 지도별로 측정한 `ground_z`를 사용합니다. map의 z=0을 지면으로 가정하지 않습니다.
PCD bounding box 밖은 막지만, bounding box 안의 모든 미관측 공간을 자유 공간으로
확정할 수는 없습니다. 저장 복셀 0.2m와 장애물 표본 밀도를 고려해 collision_margin을 검증하세요.

실시간 `/velodyne_points`는 메시지 시각의 **전체 3D TF**로 base_link에 변환합니다.
지면 기준 높이(`point.z + 0.14`)로 지면/상부 점을 제거하고 차체 내부 반사를 제외합니다.
DWA는 KMU의 조향 후보/비용을 유지하며, 중간·끝점뿐 아니라 궤적 전체의 충돌을 검사합니다.
속도는 물리 단위 설정값입니다. SSP에도 별도 궤적 충돌 정지 검사를 적용합니다.
현재 기존 LiDAR 설정은 후방 120°와 0.9m 이내 점을 제거합니다.
`reverse_observable: false`가 기본이며 후진 SSP 미리보기는 표시하지만 후진 속도는 0입니다.
실제 후방 관측이 확보되고 검증된 경우에만 새 설정에서 true로 바꿉니다.

## 빌드와 기본 실행

### RViz 시작·목표 입력으로 경로 생성만 확인

```bash
source ~/ros2_ws/install/setup.bash
ros2 launch macaron_parking planning_preview.launch.py
```

이 명령은 PCD 지도와 경로 표시가 설정된 RViz를 함께 실행합니다. 센서와 NDT 입력 없이도
동작하며 차량 출력 변환 노드·JOY·Arduino 연결은 실행하지 않습니다.

1. `2D Pose Estimate`로 시작 후륜축 중심과 차체 방향을 지정합니다(`/initialpose`).
2. `2D Goal Pose`로 목표 후륜축 중심과 차체 방향을 지정합니다(`/goal_pose`).
3. 생성된 경로는 `/parking/path`에 표시되며 시작·목표를 다시 지정하면 재계획합니다.

탐색 중 시작·목표를 바꾸면 이전 탐색을 중단하고 최신 입력으로 다시 계산합니다.
충돌 검사는 여러 후보의 점군 검색과 거리 계산을 묶어 수행하며, 높이·차체·안전 여유
판정 기준은 유지합니다. 노란 차체 표시는 `collision_margin`을 포함한 영역입니다.

이 모드는 `planning_only: true`이며 NDT 초기 정합·실시간 점군·JOY AUTO를 기다리지 않습니다.
클릭한 시작 자세는 NDT 결과로 덮어쓰지 않습니다. 성공 시 상태는 `PREVIEW_READY`로 유지하고,
속도·조향은 항상 0입니다. `enable_drive: true`와 함께 사용하면 실행을 거부합니다.
PCD 충돌 검사는 그대로 적용하므로 차체가 장애물·지도 바깥에 있거나 가능한 경로가 없으면
계획이 실패하며 `/parking/status`와 터미널에 이유를 표시합니다.
`time limit reached`는 지정된 시간 안에 경로를 찾지 못했다는 뜻이며, 경로가 없다는
증명은 아닙니다. 기본 시간 한도는 `config/parking.yaml`의 `planner_time_limit: 10.0`입니다.
시작점 자체는 자유 공간이어도 차체 방향과 앞뒤 범퍼·안전 여유까지 고려하면 충돌할 수 있습니다.

이미 RViz가 켜져 있다면 `rviz:=false`를 사용합니다. 해당 RViz에서 Fixed Frame은 `map`,
시작 도구 토픽은 `/initialpose`, 목표 도구 토픽은 `/goal_pose`로 지정합니다.
지도 `/parking/map`과 경로 `/parking/path`는 Reliable·Transient Local로 표시합니다.
다른 지도는 `map_path:=/절대경로/지도.pcd`로 지정합니다.

기존 위치 추정과 함께 확인하려면 아래처럼 경로 확인 모드만 선택할 수도 있습니다.

```bash
ros2 launch macaron_parking localization_parking.launch.py \
  planning_only:=true goal_topic:=/goal_pose
```

일반 주차 실행은 아래와 같으며 `planning_only: false`가 기본입니다. 일반 모드의
현재 위치 입력·NDT 정상 상태 검사·실시간 장애물 검사 조건은 유지됩니다.

```bash
cd ~/ros2_ws
colcon build --packages-select macaron_parking --symlink-install
source install/setup.bash

# 이미 기존 localization을 실행한 경우: 주차 계산과 Adapter 미리보기만 실행
ros2 launch macaron_parking parking.launch.py

# 기존 localization launch까지 포함; 센서 드라이버는 원본 start_sensors 동작을 사용
ros2 launch macaron_parking localization_parking.launch.py \
  map_path:=/home/junseo5777/ros2_ws/src/3d_slam/maps/library.pcd
```

ROS domain 설정은 기존 Localization과 같아야 합니다. `/initialpose`는 기존 NDT 초기
위치 지정용이며 `/parking/goal`과 다릅니다. RViz의 2D Goal Pose 도구 Topic을
`/parking/goal`로 지정하거나 `goal_topic:=/goal_pose` 인자로 실행하세요.
목표는 map frame의 차량 몸체 yaw입니다. 후진 진행 방향을 yaw에 대신 넣지 않습니다.

```bash
ros2 topic pub --once /parking/goal geometry_msgs/msg/PoseStamped \
  '{header: {frame_id: map}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}'
ros2 service call /parking/cancel std_srvs/srv/Trigger '{}'
ros2 service call /parking/replan std_srvs/srv/Trigger '{}'
```

지도 기본 경로는 workspace에서 `src/3d_slam/maps/library.pcd`를 찾습니다.
다른 배치에서는 map_path를 지정하세요. NDT와 planner에는 반드시 동일한 PCD를 사용합니다.
별도 `vehicle_params`, `parking_params`, `adapter_params` 파일 경로도 launch 인자로 받습니다.
설정은 시작 시 읽습니다. 변경 후 노드를 재시작하세요.
모든 설정 파라미터에 한국어 주석이 있으며 ROS 파라미터 설명과 launch 인자 설명도
한국어로 제공합니다. 파라미터·토픽 이름은 기존 인터페이스에 맞춰 유지합니다.

아두이노만 연결된 현재 상태에서도 아래 **ROS/장치 없는** 명령으로 PCD 계획을 확인할 수 있습니다.
start/goal의 세 번째 입력은 degree이고 JSON 출력 yaw는 rad입니다.

```bash
ros2 run macaron_parking parking_plan \
  --map ~/ros2_ws/src/3d_slam/maps/library.pcd \
  --start=0,0,0 --goal=2,0,0 --output=/tmp/parking_path.json
```

이 예시 좌표가 실제 지도에서 충돌이면 계획은 실패합니다. 실제 자유 공간 좌표를 지정하세요.
수동으로 지도 지면 높이를 측정한 경우 `--ground-z=-0.88`처럼 명시할 수 있습니다.
`--params <새 YAML>`은 반복 가능하며, `--retry`는 제한된 재주차 계획만 계산합니다.

## 인터페이스와 상태

| 토픽 | 형식 / 의미 |
|---|---|
| `/parking/goal` | PoseStamped, map의 후륜축 목표 |
| `/parking/path` | nav_msgs/Path, map의 차체 pose |
| `/parking/directions` | Int8MultiArray, +1/-1; layout.label에 plan_id |
| `/parking/map` | PointCloud2, 읽은 원본 PCD |
| `/parking/local_path`, `/parking/footprint` | RViz 추종/차체 표시 |
| `/parking/desired_command` | AckermannDriveStamped, speed=m/s, steering_angle=rad |
| `/parking/status` | String, state/plan_id/retry_count/reason |
| `/parking/adapter_preview` | TwistStamped, 기존 AUTO 입력으로 변환한 PWM/degree 또는 차단 시 0 |
| `/parking/adapter_status` | String, 출력 차단 이유 |

Path와 direction은 같은 노드 안에서 원자적으로 controller에 전달합니다.
외부 시각화에서 방향은 점 i에서 i+1로 떠나는 기어이며 마지막은 직전 기어를 반복합니다.
ROS Path와 directions는 분리 토픽이므로 외부 controller를 추가할 경우 plan_id를 포함한
단일 메시지로 묶거나 엄격한 동기화를 추가해야 합니다.

```text
PLAN → TRACKING → FINAL_CHECK → PARKING_DONE
                 ↕ 기어 전환 정지
최종 오차 실패 → (enable_retry && retry_count==0) RETRY_PLAN → TRACKING → FINAL_CHECK
두 번째 실패 / 계획 실패 / 장애물 / Localization 손실 → STOP
```

`enable_retry: false`가 기본입니다. 정상 주차 검증 후 true로 설정하면 **한 번만**
현재 Pose에서 다시 계획합니다. 최소 0.40m 전진 후 마지막 후진을 요구하고,
이를 만들 수 없으면 정지합니다. 최종 검사는 목표 yaw 기준 종/횡/yaw 오차입니다.
기어 전환과 최종 검사는 0 명령을 2초 유지합니다. 실제 정지 확인용 encoder가 없는
기존 출력부이므로 이 dwell을 실제 정지 증명으로 해석하지 마세요.

## NDT와 출력 안전 경계

TF가 있다는 이유로 운전을 허용하지 않습니다. 일반 `tracking` 상태에서
fitness/overlap 기준을 통과한 새 상태 3개, 최신 Pose, 최신 live cloud가 필요합니다.
초기/복구 후보의 `tracking;...;verifying_candidate`, `refining_initial_pose`,
`recovered`, `tightening_after_recovery`도 차단합니다. Adapter가 독립적으로 동일
Localization gate, 명령/상태 timeout, JOY AUTO/E-STOP 상태를 검사합니다.
추종 중 실패는 STOP으로 유지되며, 건강한 입력 복원 후 새 목표 또는 replan으로 재개합니다.
기존 NDT String에는 측정 시각이 없어 status 최신성은 수신 시각 기준입니다.
엄밀한 정합 측정 age를 외부에서 확인하는 인터페이스는 원본 변경 없이 제공할 수 없습니다.

차량 연결은 `enable_drive:=true`로 별도 선택합니다. 그 전에 새 `config/adapter.yaml`에
실측 속도↔PWM 테이블, 물리 좌조향 부호를 넣고 `output_calibrated: true`로 설정해야 합니다.
예시 PWM-속도 관계를 임의로 만들지 않았으므로 초기 설정으로 실차 활성화하면 거부됩니다.
AUTO 발행자는 이 Adapter 하나만 사용하고 기존 macaron state/mission 노드는 같이 실행하지 않습니다.
JOY/선택 노드만 원본 그대로 실행하는 선택적 wrapper:

```bash
ros2 launch macaron_parking existing_output.launch.py
```

ROS Joy driver와 해당 Arduino에 맞는 micro-ROS Agent는 기존 방식으로 별도 실행합니다.
이 패키지는 serial 포트를 열거나 Arduino 펌웨어를 업로드하지 않습니다.
기본 launch는 이 output wrapper도 실행하지 않습니다.

ZIP에서 확인한 출력 계약은 `/erp_writes` Twist의 PWM/degree → 기존 mux의 속도 부호 반전
→ `/erp_write` Twist → Arduino입니다. 펌웨어 함수 goForward 기준 forward_input_sign=-1이지만,
업로드된 펌웨어와 실제 모터/조향 방향을 확인해야 합니다. Arduino는 정수 PWM/각도를 사용하며,
기존 AUTO 선택 노드는 양수 13~21° 입력을 1.5° 줄입니다. 이 비대칭 보정도 그대로 유지됩니다.

**원본 mux/Arduino에는 명령 단절 정지 watchdog이 없고, 음수 입력에서 0으로 정지할 때도
감속 필터가 남습니다.** Adapter가 살아 있는 동안 20Hz로 0을 보내 차단하지만 Adapter/PC/
serial 자체의 단절까지 정지를 보장하지는 않습니다. AUTO E-STOP도 다음 AUTO callback에서
실제 0을 발행합니다. 기존 출력층을 그대로 유지한 구현의 한계이며 실차 검증의 선행 조건입니다.

## 하드웨어 없는 검사

```bash
source /opt/ros/humble/setup.bash
PYTHONPATH=src/macaron_parking:$PYTHONPATH python3 -m pytest src/macaron_parking/test -q
```

테스트는 ROS spin, serial 또는 차량 토픽 발행 없이 PCD·충돌·알고리즘·상태·변환을 검사합니다.
추가 원본 설명과 출처는 NOTICE에 기록되어 있습니다.
