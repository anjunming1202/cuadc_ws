#!/usr/bin/env python3
"""Generate ground/field-marking texture PNGs from field/config.py -- the
single source of truth for dimensions.

Hazmat placard textures are NOT generated here -- see
extract_hazmat_images.py, which pulls the real images directly from the
source PDF. The drop-zone A/B ring is not a texture either anymore -- see
gen_world.py's ring_decal(), a plain flat-colored disc (the tube itself is
now a real hollow 3D mesh, so nothing needs a size-specific ring-cutout
texture; see gen_tube_mesh.py).

Run once at authoring time (not at simulation launch -- see
docs/spec/working-notes.md discussion): outputs are committed to the repo
like any other asset.

Usage: python3 field/gen_textures.py
(run from src/cuadc_sim/field/, or anywhere -- paths below are relative to
this file, not the cwd)
"""
import os

from PIL import Image, ImageDraw, ImageFont

import config

HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = os.path.join(HERE, '..', 'models')
# DroidSansFallbackFull covers CJK but -- despite the name -- has no usable
# ASCII glyphs of its own (renders "H"/digits/English as tofu boxes); use
# DejaVu for anything Latin/numeric, Droid only for actual Chinese text.
LATIN_FONT_PATH = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'

# H glyph "up" should point down the field (+X), same idea as
# gen_world.py's PLACARD_YAW_DEG. Unverified -- adjust if it renders wrong.
HAZMAT_UP_ROTATION_DEG = -90

# 比赛场地=水泥地：真机白天地面 luma 均值 ~180、p1–p99 ~135–200；sim 渲染约为贴图 ×0.875
CONCRETE = (200, 200, 200)
CONCRETE_NOISE = 35
WHITE = (255, 255, 255)
BLUE = (30, 120, 220)


def latin_font(size):
    return ImageFont.truetype(LATIN_FONT_PATH, size)


def save(img, model_name, filename):
    out_dir = os.path.join(MODELS, model_name, 'materials', 'textures')
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, filename)
    img.save(path)
    print(f'wrote {path} ({img.width}x{img.height})')


# ---------------------------------------------------------------------------
# Ground base: concrete-bright gray with patchy noise, sized to config.GROUND_LENGTH x
# config.GROUND_WIDTH (meters) at config.GROUND_PIXELS_PER_METER.
# ---------------------------------------------------------------------------
def _concrete(w, h):
    # Low-res noise upscaled with smoothing, not per-pixel noise -- the
    # latter is incompressible (12MB of unstructured noise for what should
    # be a cheap flat-ish texture, defeating the whole "keep the ground
    # light" lesson from this session's WSLg perf investigation).
    import numpy as np
    low_w, low_h = max(1, w // 20), max(1, h // 20)
    rng = np.random.default_rng(0)
    noise = rng.integers(-CONCRETE_NOISE, CONCRETE_NOISE + 1, size=(low_h, low_w, 1)).astype(np.float32)
    low = np.clip(np.array(CONCRETE, dtype=np.float32) + noise, 0, 255).astype(np.uint8)
    return Image.fromarray(low, mode='RGB').resize((w, h), Image.BICUBIC)


def gen_ground_base():
    ppm = config.GROUND_PIXELS_PER_METER
    img = _concrete(round(config.GROUND_LENGTH * ppm), round(config.GROUND_WIDTH * ppm))
    save(img, 'cuadc_ground', 'ground_base.png')


def world_to_px(x_m, y_m, ppm):
    """Map world (X,Y) meters to pixel coords within the ground canvas.
    Image row 0 = world Y_max edge (image "up" = world +Y)."""
    px = (x_m - config.GROUND_X_MIN) * ppm
    py = (config.GROUND_Y_MAX - y_m) * ppm
    return px, py


# ---------------------------------------------------------------------------
# Field markings overlay: same footprint/aspect as ground_base, so it can be
# laid directly on top (small Z offset in the SDF). Landing circle + H
# only -- the zone boundary outlines and takeoff line are real thin-box
# geometry now (gen_world.py's chalk_line()), not baked into this texture:
# a 4mm chalk line is sub-pixel at this texture's resolution (see
# config.CHALK_LINE_WIDTH's comment), so geometry is the only way to get
# the actual width right rather than an enlarged stand-in.
# ---------------------------------------------------------------------------
def gen_field_markings():
    ppm = config.GROUND_PIXELS_PER_METER
    w = round(config.GROUND_LENGTH * ppm)
    h = round(config.GROUND_WIDTH * ppm)
    img = _concrete(w, h)  # 不透明贴图会盖住 ground_base，底色须同为水泥地
    draw = ImageDraw.Draw(img)

    # Landing circle: blue disc + white "H", rotated to face the same way
    # as the recon placards (config.PLACARD_YAW_DEG in gen_world.py) --
    # "up" pointing along +X, the flight direction from landing zone to
    # drop/recon zones, instead of PIL's default "up" = image-up = +Y.
    # Same caveat as the placards: unverified against the actual renderer,
    # adjust HAZMAT_UP_ROTATION_DEG below if it renders wrong after launch.
    # One per field (config.FIELD_Y_OFFSETS) -- both share this one ground
    # texture rather than each field getting its own.
    base_cx, base_cy = config.LANDING_PAD_CENTER
    r_px = config.LANDING_PAD_DIAMETER / 2 * ppm
    glyph_size = round(r_px * 1.3)

    for field_id in config.FIELD_IDS:
        y_offset = config.FIELD_Y_OFFSETS[field_id]
        ccx, ccy = world_to_px(base_cx, base_cy + y_offset, ppm)
        draw.ellipse([ccx - r_px, ccy - r_px, ccx + r_px, ccy + r_px], fill=BLUE)

        glyph_img = Image.new('RGBA', (glyph_size * 2, glyph_size * 2), (0, 0, 0, 0))
        glyph_draw = ImageDraw.Draw(glyph_img)
        glyph_draw.text((glyph_size, glyph_size), 'H', font=latin_font(glyph_size),
                         fill=WHITE, anchor='mm')
        glyph_img = glyph_img.rotate(HAZMAT_UP_ROTATION_DEG, resample=Image.BICUBIC)
        img.paste(glyph_img, (round(ccx - glyph_size), round(ccy - glyph_size)), glyph_img)

    save(img, 'cuadc_ground', 'field_markings.png')


if __name__ == '__main__':
    gen_ground_base()
    gen_field_markings()
