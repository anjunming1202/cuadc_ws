"""Single source of truth for CUADC competition field dimensions (meters).

Source: docs/reference/CUADC_多旋翼无人机侦察与救援_场地布局定义.md -- the
precise, self-consistency-checked layout doc (supersedes the looser
competition-rules-summary.md for exact coordinates; that summary's "8m
长/5m宽" for the drop/recon zones was ambiguous about which axis was which
and turned out backwards -- see this doc's section 8/11 for the actual
30<=x<=35 / 55<=x<=60, -4<=y<=4 definitions).

Coordinate convention (matches the doc exactly): origin (0,0) is the
起降点/landing-pad center; +X is the direction of flight progression
(prep -> takeoff -> drop zone -> recon zone); +Y is lateral, field width
is uniformly 8m (y in [-4, 4]) across every zone.

gen_textures.py and gen_world.py are both written against these constants,
not hand-transcribed numbers -- change the rules interpretation here, not
in the generated outputs.
"""

# ---- Field width (shared by every zone) ----
FIELD_Y_MIN = -4.0
FIELD_Y_MAX = 4.0
FIELD_WIDTH = FIELD_Y_MAX - FIELD_Y_MIN  # 8.0

# ---- Chalk-line ground markings (zone boundary outlines, takeoff line) ----
# Not specified by the rules (a real-world painting/marking-crew detail).
# A hand-drawn chalk-stick line, not a wheeled field-marking machine's
# 5-10cm line -- modeled as real thin box geometry rather than baked into
# the ground texture, since 4mm is sub-pixel at the ground texture's
# deliberately-cheap resolution (GROUND_PIXELS_PER_METER below); a
# geometric box is exact regardless of any texture resolution choice.
CHALK_LINE_WIDTH = 0.004
CHALK_LINE_HEIGHT = 0.001

# ---- 起降区 Takeoff/landing zone ----
LANDING_ZONE_X_MIN = -3.0
LANDING_ZONE_X_MAX = 30.0  # == DROP_ZONE_X_MIN, zones are contiguous
LANDING_ZONE_LENGTH = LANDING_ZONE_X_MAX - LANDING_ZONE_X_MIN  # 33.0

LANDING_PAD_CENTER = (0.0, 0.0)
LANDING_PAD_DIAMETER = 0.8

TAKEOFF_LINE_X = 2.0

# ---- 准备区 Prep zone ----
# Depth is explicitly NOT fixed by the rules (site-measured in practice);
# this is only an engineering placeholder for drawing/ground-extent
# purposes, per the reference doc's section 5.
PREP_ZONE_BOUNDARY_X = LANDING_ZONE_X_MIN  # -3.0
PREP_ZONE_DEPTH = 5.0

# ---- 投放区 Payload drop zone ----
DROP_ZONE_X_MIN = 30.0
DROP_ZONE_X_MAX = 35.0
DROP_ZONE_LENGTH = DROP_ZONE_X_MAX - DROP_ZONE_X_MIN  # 5.0

# (label, diameter, height) -- all white, per the rules.
DROP_CYLINDERS = [
    ("1", 0.15, 0.30),
    ("2", 0.20, 0.30),
    ("3", 0.25, 0.30),
]
# B-ring outer diameter is fixed regardless of which cylinder it surrounds
# (docs/lessons.md pixel-measured this from the rules figure). Since the
# cylinders are now modeled as actual hollow tubes (see gen_tube_mesh.py)
# rather than solid discs, the ground marking underneath only needs to be
# a single plain blue disc of this diameter -- the tube's own wall
# geometry naturally occludes the part of the disc it sits over, there's
# no need for a separately-sized "ring cutout" texture per cylinder like
# the first pass at this had.
DROP_ZONE_B_RING_OUTER_DIAMETER = 1.0

# Tube wall thickness -- not specified by the rules (section 15 lists
# "筒体壁厚" as an explicitly unfixed 3D parameter). Thin enough to read
# visually as a rim, not a solid block.
TUBE_WALL_THICKNESS = 0.01

# ---- 投放区与侦察区之间的过渡区 Transition zone (no markings/objects) ----
TRANSITION_X_MIN = DROP_ZONE_X_MAX     # 35.0
TRANSITION_X_MAX = 55.0

# ---- 灾情侦察区 Disaster reconnaissance zone ----
RECON_ZONE_X_MIN = 55.0
RECON_ZONE_X_MAX = 60.0
RECON_ZONE_LENGTH = RECON_ZONE_X_MAX - RECON_ZONE_X_MIN  # 5.0

RECON_CYLINDER_DIAMETER = 0.20
RECON_CYLINDER_HEIGHT = 0.15
RECON_CYLINDER_COUNT = 5
RECON_PLACARD_COUNT = 3     # of the 5 cylinders, how many carry a placard
RECON_PLACARD_SIZE = 0.12   # 12x12cm, per the rules

# ---- Cylinder placement ----
# The rules call for random placement each competition run (section
# 8.3/11.2), but for day-to-day dev/testing these are fixed, easy
# positions instead: comfortably clear of every zone edge (>=0.8m past
# each cylinder's own radius) and well separated from each other (no
# adjacency stress-testing). A GUI tool to hand-place/randomize/apply
# these interactively is a reasonable later addition (see the note in
# gen_world.py's module docstring) but out of scope for now -- change
# these dicts directly and regenerate (gen_world.py) when a different
# fixed layout is wanted.
DROP_CYLINDER_POSITIONS = {
    "1": (31.0, -1.5),
    "2": (32.5, 1.5),
    "3": (34.0, 0.0),
}

# ---- 投放区单点测试用出生点（gen_world.py 生成的平行 world:
# cuadc_field_drop_zone_bench.sdf）----
# 投放区反复调试用，飞机直接出生在投放区近边界线后方，不用从起降区飞
# 过去。放在 DROP_ZONE_X_MIN 往前 1m，留出的间距比 LANDING_PAD_DIAMETER
# 的半径大得多，也比 DROP_CYLINDER_POSITIONS 里任何一个筒离近边界的距离
# 更靠外——出生点不会跟场地已知的固定圆筒位置重叠（圆筒摆放目前是固定
# 坐标，不是随机的，见上面的说明；如果以后真的改成随机摆放，只要摆放
# 范围仍然限制在 DROP_ZONE_X_MIN 以内，这个出生点依然安全）。
DROP_ZONE_BENCH_SPAWN = (DROP_ZONE_X_MIN - 1.0, 0.0)

RECON_CYLINDER_POSITIONS = {
    "a": (55.8, -2.5),
    "b": (56.6, 1.8),
    "c": (57.5, -0.3),
    "d": (58.3, 2.5),
    "e": (59.2, -1.8),
}

# Which of the 5 recon cylinders carry a placard, and which of the 10
# known types each shows -- also fixed for now, same reasoning as above.
RECON_CYLINDER_PLACARDS = {
    "a": "flammable",
    "c": "corrosive",
    "e": "biohazard",
}

# ---- Parallel fields (doc section 13) ----
# Two identical fields side by side; only Y differs, per the doc's own
# transform: (x,y) -> (x, y+y_k). Field A keeps the canonical centerline
# (y=0) that every constant above is already defined against; field B is
# the same layout translated by the doc's recommended minimum spacing
# (edge gap >=5m, centerline gap >=13m for two 8m-wide fields). All the
# DROP_CYLINDER_POSITIONS/RECON_CYLINDER_POSITIONS/etc. dicts above stay
# field-A-local -- gen_world.py adds each field's offset when
# instantiating that field's objects, rather than this file storing
# already-shifted coordinates for field B.
FIELD_IDS = ['A', 'B']
FIELD_Y_OFFSETS = {'A': 0.0, 'B': 13.0}

# ---- Ground plane extents ----
# Covers the prep zone (using its engineering-placeholder depth) through
# past the recon zone, and both fields' full lateral extent, with margin
# on all sides -- generous enough that flying near a zone boundary still
# shows ground-colored texture underneath, not the plane's raw edge.
# Single source of truth for both the ground texture's aspect ratio and
# the SDF plane's <size>.
GROUND_MARGIN = 10.0
GROUND_X_MIN = PREP_ZONE_BOUNDARY_X - PREP_ZONE_DEPTH - GROUND_MARGIN
GROUND_X_MAX = RECON_ZONE_X_MAX + GROUND_MARGIN
GROUND_Y_MIN = min(FIELD_Y_MIN + off for off in FIELD_Y_OFFSETS.values()) - GROUND_MARGIN
GROUND_Y_MAX = max(FIELD_Y_MAX + off for off in FIELD_Y_OFFSETS.values()) + GROUND_MARGIN
GROUND_LENGTH = GROUND_X_MAX - GROUND_X_MIN  # along X
GROUND_WIDTH = GROUND_Y_MAX - GROUND_Y_MIN   # along Y

# ---- Texture generation ----
# Purely an image-resolution choice -- does not affect real-world size (a
# plane's SDF <size> in meters is what any image gets stretched to fill,
# regardless of its pixel count). Deliberately low: the ground spans
# ~78x18m, and a high px/m there means a multi-megapixel, multi-MB VRAM
# texture -- the exact mistake this session's WSLg perf investigation
# flagged.
GROUND_PIXELS_PER_METER = 30
