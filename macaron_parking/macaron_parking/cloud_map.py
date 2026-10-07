"""PCD의 XYZ 점을 직접 사용하여 후보 차량 위치의 충돌을 검사합니다."""

import math
import time
from itertools import chain
from pathlib import Path
import numpy as np
from scipy.ndimage import minimum_filter, maximum_filter
from scipy.spatial import cKDTree
from .vehicle import footprint_clearance


def load_pcd(filename):
    """필드 순서와 COUNT를 반영하여 ASCII·바이너리 PCD를 읽습니다."""
    with Path(filename).expanduser().open('rb') as stream:
        header = {}
        while True:
            raw = stream.readline()
            if not raw:
                raise ValueError('PCD has no DATA header')
            line = raw.decode('ascii').strip()
            if not line or line.startswith('#'):
                continue
            key, *values = line.split()
            header[key.upper()] = values
            if key.upper() == 'DATA':
                break
        fields = header.get('FIELDS', header.get('FIELD', []))
        sizes = list(map(int, header.get('SIZE', [])))
        types = header.get('TYPE', [])
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError('inconsistent PCD field metadata')
        if (any(n <= 0 for n in counts) or not all(k in fields for k in ('x', 'y', 'z'))
                or any(counts[fields.index(k)] != 1 for k in ('x', 'y', 'z'))):
            raise ValueError('PCD requires scalar x,y,z fields')
        n = (int(header['POINTS'][0]) if 'POINTS' in header else
             int(header['WIDTH'][0]) * int(header.get('HEIGHT', ['1'])[0]))
        if n < 1:
            raise ValueError('PCD is empty')
        mode = header['DATA'][0].lower()
        if mode == 'ascii':
            rows = np.loadtxt(stream, ndmin=2)
            if rows.shape != (n, sum(counts)):
                raise ValueError('PCD ASCII data does not match POINTS/COUNT')
            offsets = np.cumsum([0] + counts[:-1])
            xyz = rows[:, [offsets[fields.index(k)] for k in ('x', 'y', 'z')]]
        elif mode == 'binary':
            names = []
            for i, (field, kind, size, count) in enumerate(zip(fields, types, sizes, counts)):
                code = {'F': 'f', 'I': 'i', 'U': 'u'}.get(kind.upper())
                if code is None or size not in (1, 2, 4, 8) or (code == 'f' and size not in (4, 8)):
                    raise ValueError('unsupported PCD field type')
                name = field if field in ('x', 'y', 'z') else f'field_{i}'
                names.append((name, '<' + code + str(size), (count,)) if count != 1
                             else (name, '<' + code + str(size)))
            dtype = np.dtype(names)
            payload = stream.read()
            if len(payload) != n * dtype.itemsize:
                raise ValueError('PCD binary payload length mismatch')
            rows = np.frombuffer(payload, dtype=dtype, count=n)
            if any(rows[k].ndim != 1 for k in ('x', 'y', 'z')):
                raise ValueError('PCD coordinates must have COUNT 1')
            xyz = np.column_stack([rows[k] for k in ('x', 'y', 'z')])
        else:
            raise ValueError('supported PCD formats: ascii, binary; binary_compressed is unsupported')
    xyz = np.asarray(xyz, dtype=float)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    if not len(xyz):
        raise ValueError('PCD has no finite XYZ points')
    return xyz


class CloudMap:
    def __init__(self, points, vehicle, ground_mode='local', ground_z=-0.88,
                 ground_radius=1.0, ground_percentile=15.0, min_ground_points=5,
                 obstacle_min_height=0.08, safe_distance=0.30, bounds=None, prefilter=True):
        self.points = np.asarray(points, dtype=float).reshape(-1, 3)
        self.points = self.points[np.isfinite(self.points).all(axis=1)]
        if not len(self.points):
            raise ValueError('collision map needs finite XYZ points')
        if ground_mode not in ('flat', 'local'):
            raise ValueError('ground_mode must be flat or local')
        if (ground_radius <= 0 or min_ground_points < 1 or not 0 <= ground_percentile <= 100
                or not math.isfinite(ground_z) or not 0 <= obstacle_min_height < vehicle.height
                or safe_distance < 0):
            raise ValueError('invalid ground/collision settings')
        self.vehicle = vehicle
        # 원본 점은 지면 추정·지도 범위·시각화용으로 보관하며 파일에는 쓰지 않습니다.
        self._ground_tree = cKDTree(self.points[:, :2])
        self.ground_mode = ground_mode
        self.ground_z = ground_z
        self.ground_radius = ground_radius
        self.ground_percentile = ground_percentile
        self.min_ground_points = min_ground_points
        self.obstacle_min_height = obstacle_min_height
        self.safe_distance = safe_distance
        self.bounds = (tuple(bounds) if bounds is not None else
                       (*self.points[:, 0].take([np.argmin(self.points[:, 0]),
                                                 np.argmax(self.points[:, 0])]),
                        *self.points[:, 1].take([np.argmin(self.points[:, 1]),
                                                 np.argmax(self.points[:, 1])])))
        if (len(self.bounds) != 4 or not np.isfinite(self.bounds).all()
                or self.bounds[0] >= self.bounds[1] or self.bounds[2] >= self.bounds[3]):
            raise ValueError('map bounds must be [xmin,xmax,ymin,ymax]')
        self._ground_cache = {}
        self._ground_resolution = 0.10  # 기존 지면 추정의 위치 격자 간격(m)
        self._tile_cells = 32  # 빈 지도 영역에 큰 배열을 만들지 않는 타일 한 변의 격자 수
        self._ground_tiles = None
        self.prefilter_enabled = bool(prefilter)
        self.removed_below = self.removed_above = 0
        started = time.monotonic()
        self.collision_points = self._prefilter_points() if prefilter else self.points
        self.prefilter_elapsed = time.monotonic() - started
        # Hybrid A*는 바닥·상부 점을 제외한 별도 검색 구조만 조회합니다.
        self.tree = cKDTree(self.collision_points[:, :2])

    def _search_radius(self):
        return (math.hypot(self.vehicle.length / 2, self.vehicle.width / 2)
                + self.vehicle.margin + self.safe_distance)

    def _prefilter_points(self):
        """충돌에 사용될 수 없는 점을 지도 로딩 시 한 번 제외합니다."""
        z = self.points[:, 2]
        if self.ground_mode == 'flat':
            height = z - self.ground_z
            below = height < self.obstacle_min_height
            above = height > self.vehicle.height + 0.05
        else:
            resolution, cells = self._ground_resolution, self._tile_cells
            point_keys = np.rint(self.points[:, :2] / resolution).astype(np.int64)
            point_tiles, labels = np.unique(point_keys // cells, axis=0, return_inverse=True)
            offset = abs(self.vehicle.front_extent - self.vehicle.rear_overhang) / 2
            # 검색 원 중심의 후륜축 오프셋과 위치 반올림 오차까지 포함합니다.
            reach = self._search_radius() + offset + math.sqrt(2) * resolution
            padding = int(math.ceil(reach / resolution))
            self._prepare_ground_tiles(point_tiles, reach)
            below, above = np.zeros(len(z), dtype=bool), np.zeros(len(z), dtype=bool)
            for index, tile in enumerate(point_tiles):
                selected = np.flatnonzero(labels == index)
                low, high = self._ground_envelope(tile, padding)
                local = point_keys[selected] % cells
                low, high = low[local[:, 0], local[:, 1]], high[local[:, 0], local[:, 1]]
                known = np.isfinite(low) & np.isfinite(high)
                # 경사에서는 가능한 모든 주변 차량 지면 높이에 대해 범위 밖인 점만 제거합니다.
                # 지면을 알 수 없는 점은 보존하고 기존 unknown-ground 차단도 유지합니다.
                lower = np.nextafter(low + self.obstacle_min_height, -math.inf)
                upper = np.nextafter(high + (self.vehicle.height + 0.05), math.inf)
                below[selected] = known & (z[selected] < lower)
                above[selected] = known & (z[selected] > upper)
        self.removed_below, self.removed_above = int(below.sum()), int(above.sum())
        return self.points[~(below | above)]

    def _prepare_ground_tiles(self, point_tiles, reach):
        """원본 점으로 지면 높이를 미리 계산하며 점이 없는 넓은 영역은 저장하지 않습니다."""
        resolution, cells = self._ground_resolution, self._tile_cells
        span = int(math.ceil(max(reach, self.ground_radius + math.sqrt(2) * resolution)
                             / (resolution * cells)))
        tiles = np.unique(np.concatenate([point_tiles + [dx, dy]
                                          for dx in range(-span, span + 1)
                                          for dy in range(-span, span + 1)]), axis=0)
        centers = (tiles * cells + (cells - 1) / 2) * resolution
        radius = self.ground_radius + math.sqrt(2) * cells * resolution / 2
        counts = self._ground_tree.query_ball_point(centers, radius, return_length=True)
        tiles = tiles[counts >= self.min_ground_points]
        ix, iy = np.meshgrid(np.arange(cells), np.arange(cells), indexing='ij')
        local_keys = np.column_stack((ix.ravel(), iy.ravel()))
        self._ground_tiles = {}
        for tx, ty in tiles:
            centers = (local_keys + [tx * cells, ty * cells]) * resolution
            height = self._calculate_ground(centers).reshape(cells, cells)
            if np.isfinite(height).any():
                self._ground_tiles[(int(tx), int(ty))] = height

    def _calculate_ground(self, centers):
        """점군을 묶어 검색하고 기존 지면 분위수와 동일한 선형 보간을 계산합니다."""
        result = np.full(len(centers), math.nan)
        for begin in range(0, len(centers), 256):
            indices = self._ground_tree.query_ball_point(centers[begin:begin + 256],
                                                        self.ground_radius, return_sorted=False)
            counts = np.fromiter((len(items) for items in indices), dtype=int, count=len(indices))
            valid = np.flatnonzero(counts >= self.min_ground_points)
            if not len(valid):
                continue
            counts, indices = counts[valid], indices[valid]
            owners = np.repeat(np.arange(len(valid)), counts)
            offsets = np.cumsum(counts) - counts
            columns = np.arange(counts.sum()) - np.repeat(offsets, counts)
            samples = np.full((len(valid), counts.max()), math.inf)
            point_indices = np.fromiter(chain.from_iterable(indices), dtype=np.int64,
                                        count=int(counts.sum()))
            samples[owners, columns] = self.points[point_indices, 2]
            samples.sort(axis=1)
            rank = (counts - 1) * (self.ground_percentile / 100.)
            lower = np.floor(rank).astype(int)
            upper = np.minimum(lower + 1, counts - 1)
            a, b = samples[np.arange(len(valid)), lower], samples[np.arange(len(valid)), upper]
            fraction, delta = rank - lower, b - a
            heights = np.where(fraction >= .5, b - delta * (1 - fraction), a + delta * fraction)
            result[begin + valid] = heights
        return result

    def _ground_envelope(self, tile, padding):
        """각 점을 조회할 수 있는 주변 차량 위치의 지면 높이 범위를 보수적으로 구합니다."""
        cells = self._tile_cells
        span = int(math.ceil(padding / cells))
        side = (2 * span + 1) * cells
        heights = np.full((side, side), math.nan)
        for dx in range(-span, span + 1):
            for dy in range(-span, span + 1):
                values = self._ground_tiles.get((int(tile[0] + dx), int(tile[1] + dy)))
                if values is not None:
                    x, y = (dx + span) * cells, (dy + span) * cells
                    heights[x:x + cells, y:y + cells] = values
        known = np.isfinite(heights)
        low = minimum_filter(np.where(known, heights, math.inf), size=2 * padding + 1,
                             mode='constant', cval=math.inf)
        high = maximum_filter(np.where(known, heights, -math.inf), size=2 * padding + 1,
                              mode='constant', cval=-math.inf)
        begin = span * cells
        region = np.s_[begin:begin + cells, begin:begin + cells]
        return low[region], high[region]

    def ground_at(self, x, y):
        if self.ground_mode == 'flat':
            return self.ground_z
        key = (round(x / self._ground_resolution), round(y / self._ground_resolution))
        if self._ground_tiles is not None:
            cells = self._tile_cells
            height = self._ground_tiles.get((key[0] // cells, key[1] // cells))
            return float(height[key[0] % cells, key[1] % cells]) if height is not None else math.nan
        if key not in self._ground_cache:
            indices = self._ground_tree.query_ball_point(np.array(key) * self._ground_resolution,
                                                        self.ground_radius)
            height = (float(np.percentile(self.points[indices, 2], self.ground_percentile))
                      if len(indices) >= self.min_ground_points else math.nan)
            if len(self._ground_cache) >= 200000:
                self._ground_cache.clear()
            self._ground_cache[key] = height
        return self._ground_cache[key]

    def clearance(self, pose):
        if not np.isfinite(pose).all():
            return -math.inf
        corners = self.vehicle.corners(pose)
        xmin, xmax, ymin, ymax = self.bounds
        if ((corners[:, 0] < xmin).any() or (corners[:, 0] > xmax).any()
                or (corners[:, 1] < ymin).any() or (corners[:, 1] > ymax).any()):
            return -math.inf
        x, y, yaw = pose
        ground = self.ground_at(x, y)
        if not math.isfinite(ground):
            return -math.inf
        offset = (self.vehicle.front_extent - self.vehicle.rear_overhang) / 2
        center = [x + offset * math.cos(yaw), y + offset * math.sin(yaw)]
        radius = self._search_radius()
        indices = self.tree.query_ball_point(center, radius)
        nearby = self.collision_points[indices]
        relative_z = nearby[:, 2] - ground
        obstacles = nearby[(relative_z >= self.obstacle_min_height)
                           & (relative_z <= self.vehicle.height + 0.05), :2]
        return footprint_clearance([pose], obstacles, self.vehicle)

    def margins(self, poses):
        """여러 후보의 KD-tree 검색과 차체 거리 계산을 묶어 동일한 충돌 기준으로 검사합니다."""
        poses = np.asarray(poses, dtype=float)
        flat = poses.reshape(-1, 3)
        margins = np.empty(len(flat))
        # 긴 연결 경로에서도 중간 점 배열의 메모리 사용량을 제한합니다.
        for begin in range(0, len(flat), 256):
            chunk = flat[begin:begin + 256]
            margins[begin:begin + len(chunk)] = self._batch_clearance(chunk)
        return margins.reshape(poses.shape[:-1])

    def _batch_clearance(self, poses):
        result = np.full(len(poses), -math.inf)
        valid = np.flatnonzero(np.isfinite(poses).all(axis=1))
        if not len(valid):
            return result
        vehicle = self.vehicle
        chosen = poses[valid]
        x, y, yaw = chosen.T
        c, s = np.cos(yaw), np.sin(yaw)
        local_corners = vehicle.corners((0., 0., 0.))
        corner_x = x[:, None] + c[:, None] * local_corners[:, 0] - s[:, None] * local_corners[:, 1]
        corner_y = y[:, None] + s[:, None] * local_corners[:, 0] + c[:, None] * local_corners[:, 1]
        xmin, xmax, ymin, ymax = self.bounds
        inside_map = ((corner_x >= xmin).all(axis=1) & (corner_x <= xmax).all(axis=1)
                      & (corner_y >= ymin).all(axis=1) & (corner_y <= ymax).all(axis=1))
        valid, chosen = valid[inside_map], chosen[inside_map]
        if not len(valid):
            return result
        ground = np.array([self.ground_at(p[0], p[1]) for p in chosen])
        known = np.isfinite(ground)
        valid, chosen, ground = valid[known], chosen[known], ground[known]
        if not len(valid):
            return result
        x, y, yaw = chosen.T
        c, s = np.cos(yaw), np.sin(yaw)
        offset = (vehicle.front_extent - vehicle.rear_overhang) / 2
        centers = np.column_stack((x + offset * c, y + offset * s))
        radius = self._search_radius()
        indices = self.tree.query_ball_point(centers, radius)
        counts = np.fromiter((len(items) for items in indices), dtype=int, count=len(valid))
        clearance = np.full(len(valid), math.inf)
        if counts.sum():
            owners = np.repeat(np.arange(len(valid)), counts)
            points = self.collision_points[np.fromiter(chain.from_iterable(indices), dtype=np.int64,
                                                       count=int(counts.sum()))]
            height = points[:, 2] - ground[owners]
            obstacle = ((height >= self.obstacle_min_height) & (height <= vehicle.height + 0.05))
            owners, points = owners[obstacle], points[obstacle]
            dx, dy = points[:, 0] - x[owners], points[:, 1] - y[owners]
            px, py = c[owners] * dx + s[owners] * dy, -s[owners] * dx + c[owners] * dy
            longitudinal = np.maximum(np.maximum(-vehicle.rear_overhang - px, px - vehicle.front_extent), 0.)
            lateral = np.maximum(np.abs(py) - vehicle.width / 2, 0.)
            distance = np.hypot(longitudinal, lateral) - vehicle.margin
            inside_body = ((px >= -vehicle.rear_overhang) & (px <= vehicle.front_extent)
                           & (np.abs(py) <= vehicle.width / 2))
            distance[inside_body] = -max(vehicle.margin, 1e-6)
            np.minimum.at(clearance, owners, distance)
        result[valid] = clearance
        return result

    def path_is_free(self, poses):
        return bool((self.margins(poses) >= 0).all())


def live_obstacles(points_base, vehicle, min_height=0.08, max_range=4.0):
    """전체 3차원 좌표 변환 후 지면·상부·차체 반사점을 제거합니다."""
    points = np.asarray(points_base, dtype=float).reshape(-1, 3)
    points = points[np.isfinite(points).all(axis=1)]
    height = points[:, 2] + vehicle.base_height
    keep = ((height >= min_height) & (height <= vehicle.height + 0.05)
            & (np.hypot(points[:, 0], points[:, 1]) <= max_range))
    points = points[keep]
    # 차체 안쪽만 제외하여 차체 밖의 가까운 장애물 점은 보존합니다.
    is_self = ((points[:, 0] > -vehicle.rear_overhang + 0.03)
               & (points[:, 0] < vehicle.front_extent - 0.03)
               & (np.abs(points[:, 1]) < vehicle.width / 2 - 0.03))
    return points[~is_self, :2]
