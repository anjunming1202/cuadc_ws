"""color_pad 候选须像筒顶：合成图(水泥地 + 蓝垫 + 筒/砖/反光)上的判定。"""

import cv2
import numpy as np

from cuadc_perception.detectors import color_pad as cp

CFG = cp.load_config('real')
CONCRETE, BLUE, WHITE = (170, 172, 175), (200, 100, 30), (240, 240, 240)
BRICK, SUNLIT_BRICK = (90, 120, 190), (170, 185, 225)  # BGR：砖、被晒白的砖


def scene(mat_center=(480, 270), mat_r=200):
    img = np.full((540, 960, 3), CONCRETE, np.uint8)
    cv2.circle(img, mat_center, mat_r, BLUE, -1)
    return img


def tube(img, center, r=45):
    cv2.circle(img, center, r, WHITE, -1)
    cv2.circle(img, center, int(r * 0.6), (70, 70, 70), -1)  # 筒内暗处(压重石/阴影)
    return img


def brick(img, center, size=(110, 70), color=BRICK):
    (cx, cy), (w, h) = center, size
    cv2.rectangle(img, (cx - w // 2, cy - h // 2), (cx + w // 2, cy + h // 2), color, -1)
    return img


def detect(img):
    return cp.detect_cylinders_pixels(img, CFG)


def test_white_tube_on_mat_is_found():
    obs = detect(tube(scene(), (480, 270)))
    assert len(obs) == 1 and abs(obs[0].u - 480) < 3 and abs(obs[0].v - 270) < 3


def test_brick_on_empty_mat_is_not_a_tube():
    assert detect(brick(scene(), (480, 270))) == []


def test_sunlit_brick_on_empty_mat_is_not_a_tube():
    assert detect(brick(scene(), (480, 270), color=SUNLIT_BRICK)) == []


def test_tube_chosen_over_larger_brick_on_same_mat():
    img = brick(tube(scene(), (430, 270), r=40), (560, 270), size=(120, 80))
    obs = detect(img)
    assert len(obs) == 1 and abs(obs[0].u - 430) < 3


def test_glare_streak_is_not_a_tube():
    img = scene()
    cv2.ellipse(img, (480, 270), (90, 9), 20, 0, 360, WHITE, -1)
    assert detect(img) == []


def test_concrete_beside_partial_mat_is_not_a_tube():
    img = scene(mat_center=(480, 640), mat_r=260)  # 垫子大半在画面下方之外，外接圆里含水泥地
    brick(img, (420, 400), size=(90, 60))
    assert detect(img) == []


def test_tube_cut_by_image_border_is_found():
    img = scene(mat_center=(480, 470), mat_r=220)
    tube(img, (480, 515), r=50)  # 筒被画面底边切开：洞不闭合，走白斑兜底
    obs = detect(img)
    assert len(obs) == 1 and abs(obs[0].u - 480) < 5
