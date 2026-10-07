"""지도 전처리의 파일 보존·장애물 보존·원본 충돌 판정과의 일치를 검사합니다."""

import hashlib
import math
import numpy as np
import pytest
from macaron_parking.cloud_map import CloudMap, load_pcd
from macaron_parking.hybrid_astar import HybridAStar
from macaron_parking.vehicle import Vehicle


def floor_points(height=0., step=.25):
    xy = np.array([[x, y] for x in np.arange(-4, 4.01, step)
                   for y in np.arange(-4, 4.01, step)])
    z = height(xy[:, 0], xy[:, 1]) if callable(height) else np.full(len(xy), height)
    return np.column_stack((xy, z))


def test_prefilter_preserves_pcd_file_and_input_array(tmp_path):
    points = np.vstack((floor_points(-.9), [[.8, 0, -.65], [.8, 0, 3.]]))
    path = tmp_path / 'original.pcd'
    header = ('FIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n'
              f'WIDTH {len(points)}\nHEIGHT 1\nPOINTS {len(points)}\nDATA ascii\n')
    path.write_text(header + '\n'.join(' '.join(map(str, point)) for point in points))
    digest = hashlib.sha256(path.read_bytes()).hexdigest()
    source = load_pcd(path)
    expected = source.copy()
    cloud = CloudMap(source, Vehicle(), ground_mode='flat', ground_z=-.9)
    assert hashlib.sha256(path.read_bytes()).hexdigest() == digest
    np.testing.assert_array_equal(source, expected)
    np.testing.assert_array_equal(cloud.points, expected)
    np.testing.assert_array_equal(cloud.collision_points, [[.8, 0, -.65]])
    assert cloud.removed_below == len(points) - 2 and cloud.removed_above == 1
    assert cloud.bounds == (-4., 4., -4., 4.)


@pytest.mark.parametrize('mode', ['flat', 'local'])
def test_floor_only_map_keeps_ground_and_bounds_after_all_collision_points_removed(mode):
    cloud = CloudMap(floor_points(-.9), Vehicle(), ground_mode=mode, ground_z=-.9)
    assert len(cloud.collision_points) == 0 and cloud.tree.n == 0
    assert cloud.ground_at(0, 0) == -.9
    assert math.isinf(cloud.clearance((0, 0, 0))) and cloud.clearance((0, 0, 0)) > 0
    assert cloud.clearance((3.9, 0, 0)) < 0
    result = HybridAStar(cloud).plan((0, 0, 0), (1, 0, 0))
    assert result.success and cloud.path_is_free(result.poses)


@pytest.mark.parametrize('mode', ['flat', 'local'])
def test_prefilter_keeps_low_obstacles_and_removes_high_points(mode):
    floor = floor_points(4.)
    cloud = CloudMap(np.vstack((floor, [[.8, 0, 4.11], [.8, 0, 7.]])), Vehicle(),
                     ground_mode=mode, ground_z=4.)
    assert np.any(np.all(cloud.collision_points == [.8, 0, 4.11], axis=1))
    assert not np.any(np.all(cloud.collision_points == [.8, 0, 7.], axis=1))
    assert cloud.clearance((0, 0, 0)) < 0


@pytest.mark.parametrize('percentile', [0., 15., 50., 100.])
def test_prefilter_matches_original_checks_on_slopes_and_tile_edges(percentile):
    rng = np.random.default_rng(43)
    floor = floor_points(lambda x, y: 2. + .06 * x - .03 * y)
    xy = rng.uniform(-3.8, 3.8, (300, 2))
    obstacles = np.column_stack((xy, 2. + .06 * xy[:, 0] - .03 * xy[:, 1]
                                 + rng.uniform(.08, 2., len(xy))))
    points = np.vstack((floor, obstacles))
    args = dict(ground_percentile=percentile)
    original = CloudMap(points, Vehicle(), prefilter=False, **args)
    filtered = CloudMap(points, Vehicle(), prefilter=True, **args)
    poses = rng.uniform([-4.2, -4.2, -math.pi], [4.2, 4.2, math.pi], (600, 3))
    poses[:4] = [[3.2, 0, 0], [-3.2, 0, math.pi], [0, 3.2, -math.pi/2], [0, -3.2, math.pi/2]]
    expected_ground = np.array([original.ground_at(x, y) for x, y, yaw in poses])
    actual_ground = np.array([filtered.ground_at(x, y) for x, y, yaw in poses])
    np.testing.assert_array_equal(actual_ground, expected_ground)
    expected = original.margins(poses)
    actual = filtered.margins(poses)
    np.testing.assert_allclose(actual, expected, rtol=1e-12, atol=1e-12)
    np.testing.assert_array_equal(actual >= 0, expected >= 0)
    for pose in poses[:30]:
        assert filtered.clearance(pose) == pytest.approx(original.clearance(pose))


def test_prefilter_keeps_step_obstacles_for_vehicle_on_lower_ground():
    # 점 자체의 주변 지면만 기준으로 지우면 높은 단차를 바닥으로 오인할 수 있습니다.
    points = floor_points(lambda x, y: np.where(x > 0, .4, 0.), step=.2)
    points = np.vstack((points, [[.9, 0, .4], [.9, 0, 3.]]))
    original = CloudMap(points, Vehicle(), prefilter=False)
    filtered = CloudMap(points, Vehicle())
    assert original.ground_at(.9, 0) == pytest.approx(.4)
    assert original.ground_at(0, 0) == 0
    assert np.any(np.all(filtered.collision_points == [.9, 0, .4], axis=1))
    assert not np.any(np.all(filtered.collision_points == [.9, 0, 3.], axis=1))
    assert filtered.clearance((0, 0, 0)) < 0
    assert filtered.clearance((0, 0, 0)) == pytest.approx(original.clearance((0, 0, 0)))


def test_unknown_ground_points_remain_and_do_not_allow_planning():
    cloud = CloudMap([[0, 0, 0], [1, 1, 2]], Vehicle(), min_ground_points=5,
                     bounds=(-4, 4, -4, 4))
    assert len(cloud.collision_points) == 2
    assert math.isnan(cloud.ground_at(0, 0))
    assert cloud.clearance((0, 0, 0)) < 0
    assert not HybridAStar(cloud).plan((0, 0, 0), (1, 0, 0)).success


def test_separated_map_regions_do_not_allocate_a_dense_bounding_box():
    patch = floor_points(0., step=.5)
    points = np.vstack((patch, patch + [100000., 0, 0]))
    cloud = CloudMap(points, Vehicle())
    assert len(cloud._ground_tiles) < 100
    assert cloud.ground_at(0, 0) == cloud.ground_at(100000, 0) == 0
    assert math.isnan(cloud.ground_at(50000, 0))
    assert cloud.clearance((50000, 0, 0)) < 0


def test_height_boundary_points_are_not_removed_by_rounding():
    heights = [.08, .55, np.nextafter(.08, math.inf), np.nextafter(.55, -math.inf)]
    points = np.vstack((floor_points(-.9), [[.8, 0, h - .9] for h in heights]))
    original = CloudMap(points, Vehicle(), ground_mode='flat', ground_z=-.9, prefilter=False)
    filtered = CloudMap(points, Vehicle(), ground_mode='flat', ground_z=-.9)
    expected = points[((points[:, 2] + .9) >= .08) & ((points[:, 2] + .9) <= .55)]
    np.testing.assert_array_equal(filtered.collision_points, expected)
    assert filtered.clearance((0, 0, 0)) == original.clearance((0, 0, 0))
