#!/usr/bin/env python3
"""Generate hollow-tube OBJ meshes for the drop/recon-zone cylinders.

The rules call these "圆筒" (tubes/drums), not solid cylinders -- payloads
go INTO them (drop zone) and placards sit INSIDE them (recon zone), which
only makes visual sense if they're actually open at the top. SDF has no
built-in "tube" primitive, so this generates a plain OBJ (outer wall +
inner wall + bottom annular cap, top left open) procedurally -- no 3D
modeling tool needed, just the geometry.

Collision geometry stays a solid outer cylinder (in gen_world.py, not
here): a concave open-tube collision mesh is unnecessary complexity for
what's basically an obstacle + a software zone/visual check, not a
physics-accurate "payload bounces around inside" simulation.

Usage: python3 field/gen_tube_mesh.py
"""
import math
import os

HERE = os.path.dirname(os.path.abspath(__file__))
MESHES_DIR = os.path.join(HERE, '..', 'models', 'cuadc_tube', 'meshes')


def tube_obj(outer_radius, inner_radius, height, segments=28):
    n = segments
    verts = []
    for i in range(n):
        theta = 2 * math.pi * i / n
        cx, cy = math.cos(theta), math.sin(theta)
        verts.append((outer_radius * cx, outer_radius * cy, 0.0))       # 0*n + i: outer bottom
    for i in range(n):
        theta = 2 * math.pi * i / n
        cx, cy = math.cos(theta), math.sin(theta)
        verts.append((outer_radius * cx, outer_radius * cy, height))    # 1*n + i: outer top
    for i in range(n):
        theta = 2 * math.pi * i / n
        cx, cy = math.cos(theta), math.sin(theta)
        verts.append((inner_radius * cx, inner_radius * cy, 0.0))       # 2*n + i: inner bottom
    for i in range(n):
        theta = 2 * math.pi * i / n
        cx, cy = math.cos(theta), math.sin(theta)
        verts.append((inner_radius * cx, inner_radius * cy, height))    # 3*n + i: inner top

    def OB(i): return i % n + 1
    def OT(i): return n + i % n + 1
    def IB(i): return 2 * n + i % n + 1
    def IT(i): return 3 * n + i % n + 1

    faces = []
    for i in range(n):
        j = i + 1
        # Outer wall, normal points outward (+radial): CCW looking from outside.
        faces.append((OB(i), OB(j), OT(j), OT(i)))
        # Inner wall, normal points inward toward the cavity (-radial): reversed winding.
        faces.append((IB(j), IB(i), IT(i), IT(j)))
        # Bottom annular cap, normal points down (-Z).
        faces.append((OB(j), IB(j), IB(i), OB(i)))
        # Top stays open -- no cap -- that's the point.

    lines = ['# procedurally generated hollow tube, see field/gen_tube_mesh.py']
    for v in verts:
        lines.append(f'v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}')
    for f in faces:
        lines.append('f ' + ' '.join(str(vi) for vi in f))
    return '\n'.join(lines) + '\n'


def main():
    import config
    os.makedirs(MESHES_DIR, exist_ok=True)
    sizes = set()
    for _label, diameter, height in config.DROP_CYLINDERS:
        sizes.add((diameter, height))
    sizes.add((config.RECON_CYLINDER_DIAMETER, config.RECON_CYLINDER_HEIGHT))

    for diameter, height in sizes:
        outer_r = diameter / 2
        inner_r = outer_r - config.TUBE_WALL_THICKNESS
        name = f'tube_d{int(round(diameter * 1000))}_h{int(round(height * 1000))}.obj'
        path = os.path.join(MESHES_DIR, name)
        with open(path, 'w') as f:
            f.write(tube_obj(outer_r, inner_r, height))
        print(f'wrote {path}')


if __name__ == '__main__':
    main()
