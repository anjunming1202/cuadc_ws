"""横向宽度尺寸估计单测：用真机相机模型把一个实心筒（筒顶 + 侧壁 + 筒底）投到画面，
取轮廓凸包当检测轮廓，比较横向宽度半径与面积等效半径。"""

import math

import cv2
import numpy as np
import pytest

from cuadc_perception import geometry

H_CYL = 0.30


def _project(points_world, cam, m, origin):
    d = (m.T @ (np.asarray(points_world) - origin).T).T
    return np.stack([cam.cx + cam.fx * d[:, 0] / d[:, 2], cam.cy + cam.fy * d[:, 1] / d[:, 2]], axis=1)


def _silhouette(cam, pose, x, y, r):
    m = geometry.cam_to_world_matrix(cam, pose)
    origin = geometry.camera_origin(cam, pose)
    ground = cam.fc_rest_clearance_m
    th = np.linspace(0, 2 * math.pi, 180, endpoint=False)
    ring = np.stack([x + r * np.cos(th), y + r * np.sin(th)], axis=1)
    top = np.column_stack([ring, np.full(len(th), ground - H_CYL)])
    bottom = np.column_stack([ring, np.full(len(th), ground)])
    px = _project(np.vstack([top, bottom]), cam, m, origin).astype(np.float32)
    hull = cv2.convexHull(px)
    mo = cv2.moments(hull)
    u, v = mo['m10'] / mo['m00'], mo['m01'] / mo['m00']
    _, rng = geometry.reproject_to_cylinder_top(u, v, cam, origin, m, H_CYL)
    area_r = math.sqrt(cv2.contourArea(hull) / math.pi) * rng / cam.fx
    return geometry.transverse_radius_m(hull, u, v, cam, m, rng), area_r


@pytest.mark.parametrize('h', [3.0, 5.0])
@pytest.mark.parametrize('r', [0.075, 0.10, 0.125])
@pytest.mark.parametrize('dx,dy', [(0.0, 0.0), (0.0, 1.0), (0.0, 2.0), (1.2, 0.0), (1.0, 2.2)])
def test_transverse_radius_unbiased_by_side_wall(h, r, dx, dy):
    cam = geometry.load_camera_params('real')
    pose = geometry.VehiclePose(position=(0.0, 0.0, -h), orientation_xyzw=(0.0, 0.0, 0.0, 1.0))
    if abs(dy) > 0.545 * (h + 0.11 - H_CYL) - r or abs(dx) > 0.299 * (h + 0.11 - H_CYL) - r:
        pytest.skip('outside the frame')
    tr, ar = _silhouette(cam, pose, dx, dy, r)
    assert abs(tr - r) / r < 0.03
    if math.hypot(dx, dy) >= 2.0:
        assert ar / r > 1.15          # 面积半径被侧壁撑大，横向宽度不受影响


def test_ranking_survives_off_nadir():
    """小筒在画面边缘、中筒在正下方：面积半径排序会颠倒，横向宽度不会。"""
    cam = geometry.load_camera_params('real')
    pose = geometry.VehiclePose(position=(0.0, 0.0, -5.0), orientation_xyzw=(0.0, 0.0, 0.0, 1.0))
    small_t, small_a = _silhouette(cam, pose, 0.0, 2.2, 0.075)
    mid_t, mid_a = _silhouette(cam, pose, 0.0, 0.0, 0.10)
    assert small_a > mid_a and small_t < mid_t


def test_tilted_vehicle():
    """机身横滚 10°：正下方点在画面里移开，横向方向跟着变，仍不受侧壁影响。"""
    cam = geometry.load_camera_params('real')
    s, c = math.sin(math.radians(5)), math.cos(math.radians(5))   # 半角四元数，roll 10°
    pose = geometry.VehiclePose(position=(0.0, 0.0, -5.0), orientation_xyzw=(s, 0.0, 0.0, c))
    tr, _ = _silhouette(cam, pose, 0.0, 1.5, 0.10)
    assert abs(tr - 0.10) / 0.10 < 0.03
