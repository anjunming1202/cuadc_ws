#!/usr/bin/env python3
"""Verify field dimensions without needing to eyeball the Gazebo GUI.

Two independent checks:
1. config.py's constants against the numbers in the rules doc (catches
   hand-transcription mistakes -- config.py is the one place these were
   manually copied from
   docs/reference/CUADC_多旋翼无人机侦察与救援_场地布局定义.md).
2. The generated worlds/cuadc_field.sdf + its baked ground texture against
   config.py (catches templating bugs in gen_world.py/gen_textures.py --
   e.g. the earlier landing-zone-position bug this session found only by
   eyeballing a screenshot). SDF-native objects (tubes, ring, placards,
   vehicle spawn) are checked by parsing the SDF; the ground markings
   (zone outlines, H marker, takeoff line) are baked into a texture image,
   not SDF geometry, so those are checked by sampling actual pixel colors
   at the expected image coordinates instead.

Exits non-zero if anything fails. Usage: python3 field/check_dimensions.py
"""
import math
import os
import sys
import xml.etree.ElementTree as ET

from PIL import Image

import config
import gen_textures

HERE = os.path.dirname(os.path.abspath(__file__))
WORLD_PATH = os.path.join(HERE, '..', 'worlds', 'cuadc_field.sdf')
MARKINGS_PATH = os.path.join(HERE, '..', 'models', 'cuadc_ground',
                              'materials', 'textures', 'field_markings.png')

failures = []


def check(label, condition, detail=''):
    status = 'PASS' if condition else 'FAIL'
    print(f'[{status}] {label}' + (f' -- {detail}' if detail and not condition else ''))
    if not condition:
        failures.append(label)


def approx(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ---------------------------------------------------------------------------
# 1. config.py vs. the rules doc's stated numbers (hardcoded here from the
#    doc, independent of config.py, so this actually catches transcription
#    drift rather than comparing a file to itself).
# ---------------------------------------------------------------------------
print('== config.py vs. rules doc ==')
check('landing pad center == (0,0)', config.LANDING_PAD_CENTER == (0.0, 0.0))
check('landing pad diameter == 0.8', approx(config.LANDING_PAD_DIAMETER, 0.8))
check('landing zone x in [-3, 30]',
      approx(config.LANDING_ZONE_X_MIN, -3.0) and approx(config.LANDING_ZONE_X_MAX, 30.0))
check('field width (y range) == 8 ([-4,4])',
      approx(config.FIELD_Y_MIN, -4.0) and approx(config.FIELD_Y_MAX, 4.0))
check('takeoff line x == 2', approx(config.TAKEOFF_LINE_X, 2.0))
check('drop zone x in [30, 35]',
      approx(config.DROP_ZONE_X_MIN, 30.0) and approx(config.DROP_ZONE_X_MAX, 35.0))
check('drop cylinder sizes match rules table',
      [(d, h) for _l, d, h in config.DROP_CYLINDERS] == [(0.15, 0.30), (0.20, 0.30), (0.25, 0.30)])
check('B-ring outer diameter == 1.0', approx(config.DROP_ZONE_B_RING_OUTER_DIAMETER, 1.0))
check('transition zone x in (35, 55)',
      approx(config.TRANSITION_X_MIN, 35.0) and approx(config.TRANSITION_X_MAX, 55.0))
check('recon zone x in [55, 60]',
      approx(config.RECON_ZONE_X_MIN, 55.0) and approx(config.RECON_ZONE_X_MAX, 60.0))
check('recon cylinder is 5x d=0.20 h=0.15',
      config.RECON_CYLINDER_COUNT == 5 and approx(config.RECON_CYLINDER_DIAMETER, 0.20)
      and approx(config.RECON_CYLINDER_HEIGHT, 0.15))
check('recon placard count == 3 and size == 0.12',
      config.RECON_PLACARD_COUNT == 3 and approx(config.RECON_PLACARD_SIZE, 0.12))

# ---------------------------------------------------------------------------
# 2a. Generated SDF vs. config.py (parses <model> poses/geometry).
# ---------------------------------------------------------------------------
print('\n== generated cuadc_field.sdf vs. config.py ==')
if not os.path.exists(WORLD_PATH):
    print(f'world file not found at {WORLD_PATH} -- run gen_world.py first')
    sys.exit(1)

tree = ET.parse(WORLD_PATH)
world = tree.getroot().find('world')
models_by_name = {m.get('name'): m for m in world.findall('model')}


def model_pose_xy(name):
    pose = models_by_name[name].find('pose').text.split()
    return float(pose[0]), float(pose[1])


# Ground plane size.
ground_visual = models_by_name['cuadc_ground'].find("link/visual[@name='base']")
gw, gh = ground_visual.find('geometry/plane/size').text.split()
check('ground plane <size> matches config.GROUND_LENGTH/WIDTH',
      approx(float(gw), config.GROUND_LENGTH) and approx(float(gh), config.GROUND_WIDTH))

# Vehicle spawn == landing pad center (this session's actual bug: it used
# to spawn at the takeoff line instead).
vehicle_include = world.find("include[uri='model://iris_with_gimbal']")
vx, vy, *_ = vehicle_include.find('pose').text.split()
check('vehicle spawns on the landing pad center, not the takeoff line',
      approx(float(vx), config.LANDING_PAD_CENTER[0])
      and approx(float(vy), config.LANDING_PAD_CENTER[1]),
      f'got ({vx}, {vy}), expected {config.LANDING_PAD_CENTER}')

# Drop tubes: position + mesh filename encodes the right diameter/height.
for label, diameter, height in config.DROP_CYLINDERS:
    name = f'drop_tube_{label}'
    x, y = model_pose_xy(name)
    expected_x, expected_y = config.DROP_CYLINDER_POSITIONS[label]
    mesh_uri = models_by_name[name].find("link/visual/geometry/mesh/uri").text
    expected_mesh = f'tube_d{int(round(diameter*1000))}_h{int(round(height*1000))}.obj'
    check(f'{name} position matches config', approx(x, expected_x) and approx(y, expected_y))
    check(f'{name} mesh filename encodes d={diameter}m h={height}m',
          mesh_uri.endswith(expected_mesh))
    ring_name = f'drop_ring_{label}'
    rx, ry = model_pose_xy(ring_name)
    check(f'{ring_name} centered on its tube', approx(rx, expected_x) and approx(ry, expected_y))
    ring_radius = float(models_by_name[ring_name].find(
        "link/visual/geometry/cylinder/radius").text)
    check(f'{ring_name} radius == B-ring outer diameter / 2',
          approx(ring_radius, config.DROP_ZONE_B_RING_OUTER_DIAMETER / 2))

# Recon tubes + placards.
for key, (expected_x, expected_y) in config.RECON_CYLINDER_POSITIONS.items():
    name = f'recon_tube_{key}'
    x, y = model_pose_xy(name)
    check(f'{name} position matches config', approx(x, expected_x) and approx(y, expected_y))
    placard_key = config.RECON_CYLINDER_PLACARDS.get(key)
    placard_name = f'recon_placard_{key}'
    if placard_key:
        check(f'{placard_name} exists for a cylinder with an assigned placard',
              placard_name in models_by_name)
        if placard_name in models_by_name:
            plx, ply = model_pose_xy(placard_name)
            check(f'{placard_name} centered on its tube', approx(plx, expected_x) and approx(ply, expected_y))
            size = models_by_name[placard_name].find("link/visual/geometry/plane/size").text.split()
            check(f'{placard_name} size == 0.12x0.12',
                  approx(float(size[0]), config.RECON_PLACARD_SIZE)
                  and approx(float(size[1]), config.RECON_PLACARD_SIZE))
    else:
        check(f'{placard_name} absent for a cylinder with no assigned placard',
              placard_name not in models_by_name)

check('exactly RECON_PLACARD_COUNT placards total',
      sum(1 for k in config.RECON_CYLINDER_POSITIONS if k in config.RECON_CYLINDER_PLACARDS)
      == config.RECON_PLACARD_COUNT)

# Chalk lines: real thin-box geometry (config.CHALK_LINE_WIDTH is sub-pixel
# at the ground texture's resolution, see that constant's comment), so
# these are exact-size checks, not fuzzy pixel-color sampling.
chalk_x_lines = [  # (name, x_min, x_max, y)
    ('chalk_side_landing_drop_top', config.LANDING_ZONE_X_MIN, config.DROP_ZONE_X_MAX, config.FIELD_Y_MAX),
    ('chalk_side_landing_drop_bottom', config.LANDING_ZONE_X_MIN, config.DROP_ZONE_X_MAX, config.FIELD_Y_MIN),
    ('chalk_side_recon_top', config.RECON_ZONE_X_MIN, config.RECON_ZONE_X_MAX, config.FIELD_Y_MAX),
    ('chalk_side_recon_bottom', config.RECON_ZONE_X_MIN, config.RECON_ZONE_X_MAX, config.FIELD_Y_MIN),
]
for name, x_min, x_max, y in chalk_x_lines:
    size = [float(v) for v in models_by_name[name].find('link/visual/geometry/box/size').text.split()]
    x, py = model_pose_xy(name)
    check(f'{name} length matches its zone span', approx(size[0], x_max - x_min))
    check(f'{name} width == CHALK_LINE_WIDTH', approx(size[1], config.CHALK_LINE_WIDTH))
    check(f'{name} centered correctly', approx(x, (x_min + x_max) / 2) and approx(py, y))

chalk_y_lines = [  # (name, x)
    ('chalk_cross_landing_rear', config.LANDING_ZONE_X_MIN),
    ('chalk_cross_landing_drop', config.LANDING_ZONE_X_MAX),
    ('chalk_cross_drop_far', config.DROP_ZONE_X_MAX),
    ('chalk_cross_recon_near', config.RECON_ZONE_X_MIN),
    ('chalk_cross_recon_far', config.RECON_ZONE_X_MAX),
    ('chalk_takeoff_line', config.TAKEOFF_LINE_X),
]
for name, x in chalk_y_lines:
    size = [float(v) for v in models_by_name[name].find('link/visual/geometry/box/size').text.split()]
    px_, _py = model_pose_xy(name)
    check(f'{name} length matches field width', approx(size[1], config.FIELD_WIDTH))
    check(f'{name} width == CHALK_LINE_WIDTH', approx(size[0], config.CHALK_LINE_WIDTH))
    check(f'{name} positioned at x={x}', approx(px_, x))

# ---------------------------------------------------------------------------
# 2b. Baked ground texture vs. config.py -- the landing pad (blue disc + H)
#     is still a texture, not SDF geometry (only the zone-boundary/takeoff
#     lines moved to real geometry above), so check it by sampling actual
#     pixel color at the expected image coordinates.
# ---------------------------------------------------------------------------
print('\n== baked field_markings.png vs. config.py ==')
img = Image.open(MARKINGS_PATH).convert('RGB')
ppm = config.GROUND_PIXELS_PER_METER


def px(x_m, y_m):
    return gen_textures.world_to_px(x_m, y_m, ppm)


def sample(x_m, y_m):
    x, y = px(x_m, y_m)
    x, y = min(max(int(x), 0), img.width - 1), min(max(int(y), 0), img.height - 1)
    return img.getpixel((x, y))


def is_blue(rgb):
    r, g, b = rgb
    return b > 150 and b > r + 40 and b > g + 20


cx, cy = config.LANDING_PAD_CENTER
# Sample off-center (within the disc but outside the "H" glyph strokes,
# which occupy the middle of the circle) -- the exact center pixel lands
# on the glyph itself, not the disc's blue fill.
pad_r = config.LANDING_PAD_DIAMETER / 2
check('landing pad disc pixel (off-glyph) is blue', is_blue(sample(cx + pad_r * 0.7, cy)))

# ---------------------------------------------------------------------------
print()
if failures:
    print(f'{len(failures)} check(s) FAILED:')
    for f in failures:
        print(f'  - {f}')
    sys.exit(1)
print('All checks passed.')
