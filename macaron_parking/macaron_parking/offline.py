"""ROS 노드와 차량 장치 없이 PCD 경로를 계산하는 명령입니다."""

import argparse
import json
import math
from pathlib import Path
import yaml
from .configuration import DEFAULTS, vehicle_from, configure_planner
from .cloud_map import CloudMap, load_pcd
from .hybrid_astar import HybridAStar
from .retry import plan_retry


def pose_value(text):
    values = [float(v) for v in text.split(',')]
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise argparse.ArgumentTypeError('위치는 x,y,yaw(도) 형식으로 입력하세요')
    return values[0], values[1], math.radians(values[2])


def main(argv=None):
    parser = argparse.ArgumentParser(description='ROS·차량 출력 없이 PCD로 주차 경로 계산')
    parser.add_argument('--map', required=True, help='저장된 PCD 지도 경로')
    parser.add_argument('--start', required=True, type=pose_value,
                        help='현재 후륜축 위치 x,y,yaw(도)')
    parser.add_argument('--goal', required=True, type=pose_value,
                        help='목표 후륜축 위치 x,y,yaw(도)')
    parser.add_argument('--params', action='append', default=[],
                        help='덮어쓸 새 패키지 YAML 설정; 여러 번 지정 가능')
    parser.add_argument('--ground-z', type=float,
                        help='지도에서 실측한 고정 지면 높이(m); 지정 시 평지 방식 사용')
    parser.add_argument('--retry', action='store_true',
                        help='전진 후 후진으로 마무리하는 재주차 경로 계산')
    parser.add_argument('--output', help='결과 JSON 저장 경로; 생략 시 화면에 출력')
    args = parser.parse_args(argv)
    config = dict(DEFAULTS)
    for filename in args.params:
        data = yaml.safe_load(Path(filename).read_text())
        config.update(data.get('/**', data).get('ros__parameters', data))
    if args.ground_z is not None:
        config['ground_mode'], config['ground_z'] = 'flat', args.ground_z
    vehicle = vehicle_from(config)
    cloud_map = CloudMap(load_pcd(args.map), vehicle, config['ground_mode'], config['ground_z'],
                         config['ground_radius'], config['ground_percentile'], config['min_ground_points'],
                         config['obstacle_min_height'])
    planner = HybridAStar(cloud_map)
    configure_planner(planner, config)
    result = (plan_retry(planner, args.start, args.goal, config['retry_min_forward'], config['retry_max_reverse'])
              if args.retry else planner.plan(args.start, args.goal))
    output = dict(success=result.success, reason=result.reason, elapsed=result.elapsed,
                  expansions=result.expansions, length_m=result.length,
                  frame_id=config['map_frame'], poses_xy_yaw_rad=result.poses.tolist(),
                  directions=result.directions.astype(int).tolist())
    text = json.dumps(output, ensure_ascii=False, indent=2)
    if args.output:
        Path(args.output).write_text(text + '\n')
        print(f'success={result.success};reason={result.reason};length={result.length:.3f}m;output={args.output}')
    else:
        print(text)
    return 0 if result.success else 2


if __name__ == '__main__':
    raise SystemExit(main())
