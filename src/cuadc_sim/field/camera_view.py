#!/usr/bin/env python3
"""Snap the running Gazebo GUI camera to a preset view of cuadc_field.sdf.

gz-sim has no built-in preset-view buttons (Top/Front/Side like RViz) --
this is the workaround: gz-sim exposes the GUI camera pose as a service,
/gui/move_to/pose (gz.msgs.GUICamera in, gz.msgs.Boolean out), so a preset
is just a precomputed pose sent through that service via the `gz` CLI.
Presets are computed from config.py, not hand-measured, same reasoning as
gen_world.py.

Only works while a Gazebo GUI is already running (this does not launch
anything) -- run it from another terminal.

Usage: python3 field/camera_view.py <preset> [--field A|B]
  presets: overview, field, landing, drop, recon, flyover
"""
import argparse
import math
import subprocess

import config

PRESETS = ['overview', 'field', 'landing', 'drop', 'recon', 'flyover']


def euler_to_quat(roll, pitch, yaw):
    """SDF/ROS convention (intrinsic X-Y-Z, i.e. R = Rz(yaw)*Ry(pitch)*Rx(roll))."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def overhead_pose(cx, cy, span_x, span_y, factor):
    """Straight-down pose centered on (cx, cy), high enough to roughly fit
    span_x/span_y in frame -- factor is a rough empirical fudge (not derived
    from the GUI camera's actual FOV, which isn't available here); if a
    preset looks too tight or too distant, adjust that preset's factor."""
    height = max(span_x, span_y) * factor
    return cx, cy, height, 0.0, math.pi / 2, 0.0


def zone_pose(field_id, x_min, x_max, factor):
    y_off = config.FIELD_Y_OFFSETS[field_id]
    cx = (x_min + x_max) / 2
    cy = y_off
    return overhead_pose(cx, cy, x_max - x_min, config.FIELD_WIDTH, factor)


def preset_pose(name, field_id):
    y_off = config.FIELD_Y_OFFSETS[field_id]
    if name == 'overview':
        x_min = config.PREP_ZONE_BOUNDARY_X - config.PREP_ZONE_DEPTH
        x_max = config.RECON_ZONE_X_MAX
        y_min = min(config.FIELD_Y_MIN + off for off in config.FIELD_Y_OFFSETS.values())
        y_max = max(config.FIELD_Y_MAX + off for off in config.FIELD_Y_OFFSETS.values())
        return overhead_pose((x_min + x_max) / 2, (y_min + y_max) / 2,
                              x_max - x_min, y_max - y_min, factor=0.5)
    if name == 'field':
        x_min = config.PREP_ZONE_BOUNDARY_X - config.PREP_ZONE_DEPTH
        x_max = config.RECON_ZONE_X_MAX
        return zone_pose(field_id, x_min, x_max, factor=0.5)
    if name == 'landing':
        return zone_pose(field_id, config.LANDING_ZONE_X_MIN, config.LANDING_ZONE_X_MAX, factor=0.5)
    if name == 'drop':
        return zone_pose(field_id, config.DROP_ZONE_X_MIN, config.DROP_ZONE_X_MAX, factor=1.0)
    if name == 'recon':
        return zone_pose(field_id, config.RECON_ZONE_X_MIN, config.RECON_ZONE_X_MAX, factor=1.0)
    if name == 'flyover':
        # Elevated, angled view from behind the prep zone looking down the
        # runway (+X, the flight-progression direction) -- not top-down.
        cam_x = config.PREP_ZONE_BOUNDARY_X - config.PREP_ZONE_DEPTH - 5.0
        return cam_x, y_off, 12.0, 0.0, 0.4, 0.0
    raise ValueError(f'unknown preset {name!r}, choose from {PRESETS}')


def move_camera(x, y, z, roll, pitch, yaw):
    qx, qy, qz, qw = euler_to_quat(roll, pitch, yaw)
    req = (f'pose: {{position: {{x: {x} y: {y} z: {z}}} '
           f'orientation: {{x: {qx} y: {qy} z: {qz} w: {qw}}}}}')
    subprocess.run([
        'gz', 'service', '-s', '/gui/move_to/pose',
        '--reqtype', 'gz.msgs.GUICamera', '--reptype', 'gz.msgs.Boolean',
        '--timeout', '2000', '--req', req,
    ], check=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('preset', choices=PRESETS)
    parser.add_argument('--field', choices=config.FIELD_IDS, default='A')
    args = parser.parse_args()
    pose = preset_pose(args.preset, args.field)
    move_camera(*pose)


if __name__ == '__main__':
    main()
