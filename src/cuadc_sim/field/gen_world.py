#!/usr/bin/env python3
"""Generate worlds/cuadc_field.sdf from field/config.py + hazmat_placards.py.

SDF has no real templating (an <include> can override name/pose but not
arbitrary internal properties like which texture a material uses), and
this world needs several near-identical-but-not-quite objects (3
differently-sized drop tubes, 5 recon tubes where 3 carry one of 10
different placard textures) -- so the world file itself is generated, the
same way the textures/meshes are, rather than hand-transcribed from
config.py (hand-transcription is exactly the kind of copy-paste drift this
config-driven approach exists to avoid).

Run once at authoring time, like gen_textures.py/gen_tube_mesh.py; output
is committed. Usage: python3 field/gen_world.py
"""
import os

import config
import hazmat_placards

HERE = os.path.dirname(os.path.abspath(__file__))
WORLDS = os.path.join(HERE, '..', 'worlds')
# Relative to worlds/cuadc_field.sdf's own directory -- not model:// URIs,
# since these aren't <include>'d as models here (no model.config needed),
# just reached into directly for their materials/textures (or meshes). A
# plain relative path resolves regardless of GZ_SIM_RESOURCE_PATH.
GROUND_TEX = '../models/cuadc_ground/materials/textures'
PLACARD_TEX = '../models/hazmat_placard/materials/textures'
TUBE_MESH = '../models/cuadc_tube/meshes'

# Placard "up" should point down the field (+X). Unverified -- adjust if
# it renders sideways/upside down.
PLACARD_YAW_DEG = -90.0

# Vehicle spawn yaw, empirically determined (not derived) -- re-check
# visually if this model or its upstream deps ever change.
VEHICLE_YAW_DEG = 0.0


def plain_material(rgb):
    r, g, b = rgb
    return f"""<material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.1 0.1 0.1 1</specular>
        </material>"""


def textured_material(albedo_uri):
    return f"""<material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0.05 0.05 0.05 1</specular>
          <pbr>
            <metal>
              <albedo_map>{albedo_uri}</albedo_map>
              <roughness>0.8</roughness>
              <metalness>0</metalness>
            </metal>
          </pbr>
        </material>"""


def tube_model(name, x, y, diameter, height, rgb=(0.95, 0.95, 0.95)):
    """A hollow tube (open top): visual is the procedurally generated mesh
    (see gen_tube_mesh.py), collision is a plain solid outer cylinder --
    a concave open-topped collision mesh isn't worth the complexity for
    what's an obstacle + software zone/visual check, not physics-accurate
    "payload bounces around inside the tube" simulation."""
    radius = diameter / 2
    mesh_name = f'tube_d{int(round(diameter * 1000))}_h{int(round(height * 1000))}.obj'
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <pose>0 0 {height / 2} 0 0 0</pose>
          <geometry><cylinder><radius>{radius}</radius><length>{height}</length></cylinder></geometry>
        </collision>
        <visual name="visual">
          <geometry><mesh><uri>{TUBE_MESH}/{mesh_name}</uri></mesh></geometry>
          {plain_material(rgb)}
        </visual>
      </link>
    </model>"""


def ring_decal(name, x, y):
    """Single plain blue disc (B-ring outer diameter, same for every drop
    cylinder regardless of its own size) laid on the ground under the
    tube -- since the tube is now a real hollow 3D shape, its own wall
    geometry naturally occludes the part of this disc it sits over; no
    per-cylinder-size ring-cutout texture is needed like the first pass
    at this had."""
    d = config.DROP_ZONE_B_RING_OUTER_DIAMETER
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.002 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><cylinder><radius>{d / 2}</radius><length>0.001</length></cylinder></geometry>
          {plain_material((0.12, 0.47, 0.86))}
        </visual>
      </link>
    </model>"""


def placard_decal(name, x, y, key):
    """Flat placard resting on the ground INSIDE the tube's open top (not
    on top of a solid cylinder -- the tube is hollow now, so "inside"
    means at ground level, visible by looking down through the opening)."""
    s = config.RECON_PLACARD_SIZE
    import math
    yaw_rad = math.radians(PLACARD_YAW_DEG)
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {y} 0.002 0 0 {yaw_rad:.6f}</pose>
      <link name="link">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{s} {s}</size></plane></geometry>
          {textured_material(f"{PLACARD_TEX}/{key}.png")}
        </visual>
      </link>
    </model>"""


def ground_model():
    return f"""
    <model name="cuadc_ground">
      <static>true</static>
      <pose>{(config.GROUND_X_MIN + config.GROUND_X_MAX) / 2} {(config.GROUND_Y_MIN + config.GROUND_Y_MAX) / 2} 0 0 0 0</pose>
      <link name="link">
        <collision name="collision">
          <geometry><plane><normal>0 0 1</normal></plane></geometry>
        </collision>
        <visual name="base">
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{config.GROUND_LENGTH} {config.GROUND_WIDTH}</size></plane></geometry>
          {textured_material(f"{GROUND_TEX}/ground_base.png")}
        </visual>
        <visual name="markings">
          <pose>0 0 0.001 0 0 0</pose>
          <cast_shadows>false</cast_shadows>
          <geometry><plane><normal>0 0 1</normal><size>{config.GROUND_LENGTH} {config.GROUND_WIDTH}</size></plane></geometry>
          {textured_material(f"{GROUND_TEX}/field_markings.png")}
        </visual>
      </link>
    </model>"""


def chalk_line_along_x(name, x_min, x_max, y, rgb=(0.95, 0.95, 0.95)):
    """A line running along X (a long zone-boundary edge, or the top/bottom
    side of a rectangle) at fixed Y."""
    length = x_max - x_min
    w = config.CHALK_LINE_WIDTH
    h = config.CHALK_LINE_HEIGHT
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{(x_min + x_max) / 2} {y} {h / 2} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{length} {w} {h}</size></box></geometry>
          {plain_material(rgb)}
        </visual>
      </link>
    </model>"""


def chalk_line_along_y(name, x, y_min, y_max, rgb=(0.95, 0.95, 0.95)):
    """A line running along Y (a short zone-boundary edge, or the takeoff
    line) at fixed X."""
    length = y_max - y_min
    w = config.CHALK_LINE_WIDTH
    h = config.CHALK_LINE_HEIGHT
    return f"""
    <model name="{name}">
      <static>true</static>
      <pose>{x} {(y_min + y_max) / 2} {h / 2} 0 0 0</pose>
      <link name="link">
        <visual name="visual">
          <cast_shadows>false</cast_shadows>
          <geometry><box><size>{w} {length} {h}</size></box></geometry>
          {plain_material(rgb)}
        </visual>
      </link>
    </model>"""


def build_chalk_lines(field_id, y_offset):
    """Zone boundary outlines + the takeoff line, as real thin-box geometry
    (see config.CHALK_LINE_WIDTH's comment for why not baked texture).
    Landing+drop zones are contiguous (no gap) so their shared boundary at
    x=DROP_ZONE_X_MIN is one line, not two overlapping ones; the
    transition zone has no boundary/side markings at all, per the rules
    doc (nothing distinguishes it -- its two ends ARE the drop-zone-far
    and recon-zone-near cross lines below). Names are prefixed with
    `field_id` (config.FIELD_IDS) since field B is the same layout
    translated by `y_offset`, not a distinct set of coordinates -- object
    names would otherwise collide between fields."""
    p = f'field{field_id}_'
    y0, y1 = config.FIELD_Y_MIN + y_offset, config.FIELD_Y_MAX + y_offset
    parts = [
        # Long edges (top/bottom), landing+drop zone run and recon zone run.
        chalk_line_along_x(f'{p}chalk_side_landing_drop_top',
                            config.LANDING_ZONE_X_MIN, config.DROP_ZONE_X_MAX, y1),
        chalk_line_along_x(f'{p}chalk_side_landing_drop_bottom',
                            config.LANDING_ZONE_X_MIN, config.DROP_ZONE_X_MAX, y0),
        chalk_line_along_x(f'{p}chalk_side_recon_top',
                            config.RECON_ZONE_X_MIN, config.RECON_ZONE_X_MAX, y1),
        chalk_line_along_x(f'{p}chalk_side_recon_bottom',
                            config.RECON_ZONE_X_MIN, config.RECON_ZONE_X_MAX, y0),
        # Cross lines at every zone boundary x-value (landing/drop share one).
        chalk_line_along_y(f'{p}chalk_cross_landing_rear', config.LANDING_ZONE_X_MIN, y0, y1),
        chalk_line_along_y(f'{p}chalk_cross_landing_drop', config.LANDING_ZONE_X_MAX, y0, y1),
        chalk_line_along_y(f'{p}chalk_cross_drop_far', config.DROP_ZONE_X_MAX, y0, y1),
        chalk_line_along_y(f'{p}chalk_cross_recon_near', config.RECON_ZONE_X_MIN, y0, y1),
        chalk_line_along_y(f'{p}chalk_cross_recon_far', config.RECON_ZONE_X_MAX, y0, y1),
        # Takeoff line -- distinct color (yellow), not a zone boundary.
        chalk_line_along_y(f'{p}chalk_takeoff_line', config.TAKEOFF_LINE_X, y0, y1,
                            rgb=(0.92, 0.78, 0.16)),
    ]
    return '\n'.join(parts)


def build_drop_zone(field_id, y_offset):
    p = f'field{field_id}_'
    parts = []
    for label, diameter, height in config.DROP_CYLINDERS:
        x, y = config.DROP_CYLINDER_POSITIONS[label]
        y += y_offset
        parts.append(ring_decal(f"{p}drop_ring_{label}", x, y))
        parts.append(tube_model(f"{p}drop_tube_{label}", x, y, diameter, height))
    return '\n'.join(parts)


def build_recon_zone(field_id, y_offset):
    p = f'field{field_id}_'
    parts = []
    for key, (x, y) in config.RECON_CYLINDER_POSITIONS.items():
        y += y_offset
        name = f"{p}recon_tube_{key}"
        placard_key = config.RECON_CYLINDER_PLACARDS.get(key)
        if placard_key:
            assert placard_key in hazmat_placards.PLACARDS, f"unknown placard key {placard_key}"
            parts.append(placard_decal(f"{p}recon_placard_{key}", x, y, placard_key))
        parts.append(tube_model(name, x, y, config.RECON_CYLINDER_DIAMETER,
                                 config.RECON_CYLINDER_HEIGHT))
    return '\n'.join(parts)


WORLD_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.9">
  <!--
    Generated by field/gen_world.py from field/config.py: do not hand-edit
    positions/sizes here, change the config and regenerate instead.

    Layout follows docs/reference/CUADC_多旋翼无人机侦察与救援_场地布局定义.md
    (起降区 to 投放区 to 灾情侦察区); ground/markings/ring/placard textures
    are deliberately plain-albedo-only (no PBR normal/roughness maps) per
    this session's WSLg rendering-performance investigation (see
    src/cuadc_sim/README.md). Drop/recon cylinders are hollow tubes
    (procedurally generated meshes, field/gen_tube_mesh.py), not solid
    discs: payloads go into them and placards sit inside them.

    Two parallel fields (doc section 13, config.FIELD_IDS/FIELD_Y_OFFSETS):
    field B is field A's layout translated in Y only. Only ONE vehicle
    spawns, on field A's landing pad: this project's SITL/ArduPilot
    integration is single-vehicle (one sim_vehicle.py instance), and
    field B exists here for spatial/visual completeness (so field A
    doesn't render as if it were alone on an empty range), not to
    simulate the other team's aircraft: a second vehicle would need a
    second SITL instance, out of scope for this change.
  -->
  <world name="cuadc_field">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"></plugin>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"></plugin>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"></plugin>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"></plugin>
    <plugin filename="gz-sim-navsat-system" name="gz::sim::systems::NavSat"></plugin>

    <scene>
      <ambient>1.0 1.0 1.0</ambient>
      <background>0.8 0.8 0.8</background>
    </scene>

    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.8 0.8 0.8 1</specular>
      <attenuation>
        <range>1000</range>
        <constant>0.9</constant>
        <linear>0.01</linear>
        <quadratic>0.001</quadratic>
      </attenuation>
      <direction>-0.5 0.1 -0.9</direction>
    </light>
{ground}
{fields}
    <include>
      <uri>model://iris_with_gimbal</uri>
      <pose degrees="true">{landing_x} {landing_y} 0.195 0 0 {vehicle_yaw}</pose>
    </include>

  </world>
</sdf>
"""


def build_fields():
    parts = []
    for field_id in config.FIELD_IDS:
        y_offset = config.FIELD_Y_OFFSETS[field_id]
        parts.append(build_chalk_lines(field_id, y_offset))
        parts.append(build_drop_zone(field_id, y_offset))
        parts.append(build_recon_zone(field_id, y_offset))
    return '\n'.join(parts)


def main():
    # Vehicle spawns on field A's landing pad specifically (offset 0.0),
    # not a bare LANDING_PAD_CENTER that happens to equal that today --
    # see the WORLD_TEMPLATE comment for why only field A gets a vehicle.
    landing_x, landing_y = config.LANDING_PAD_CENTER
    landing_y += config.FIELD_Y_OFFSETS['A']
    out = WORLD_TEMPLATE.format(
        ground=ground_model(),
        fields=build_fields(),
        landing_x=landing_x,
        landing_y=landing_y,
        vehicle_yaw=VEHICLE_YAW_DEG,
    )
    # XML forbids "--" anywhere inside a comment (only legal as the "-->"
    # terminator) -- a mistake this file's own module-docstring comment
    # has tripped on more than once while editing. Catch it here instead
    # of relying on a manual `gz sdf --check` afterward every time.
    import re
    for comment in re.findall(r'<!--(.*?)-->', out, re.DOTALL):
        assert '--' not in comment, (
            'generated XML comment contains "--", which is invalid inside an '
            'XML comment -- fix the offending text in WORLD_TEMPLATE above:\n'
            + comment)

    os.makedirs(WORLDS, exist_ok=True)
    path = os.path.join(WORLDS, 'cuadc_field.sdf')
    with open(path, 'w') as f:
        f.write(out)
    print(f'wrote {path}')


if __name__ == '__main__':
    main()
