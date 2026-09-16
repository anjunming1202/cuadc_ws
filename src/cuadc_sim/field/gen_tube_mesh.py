#!/usr/bin/env python3
"""Generate hollow-tube OBJ meshes for the drop/recon-zone cylinders.

The rules call these "圆筒" (tubes/drums), not solid cylinders -- payloads
go INTO them (drop zone) and placards sit INSIDE them (recon zone), which
only makes visual sense if they're actually open at the top. SDF has no
built-in "tube" primitive, so this generates a plain OBJ (outer wall +
inner wall + full closed bottom disk, top left open) procedurally -- no
3D modeling tool needed, just the geometry.

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
    # The floor has thickness (bottom_h), so the interior floor sits ABOVE the
    # ground plane instead of coincident with it -- a z=0 floor z-fights the
    # ground and the ground wins, so from above the tube looks bottomless.
    # Reusing the wall thickness as the bottom thickness keeps a drum-like look.
    bottom_h = outer_radius - inner_radius
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
        verts.append((inner_radius * cx, inner_radius * cy, bottom_h))  # 2*n + i: inner floor rim
    for i in range(n):
        theta = 2 * math.pi * i / n
        cx, cy = math.cos(theta), math.sin(theta)
        verts.append((inner_radius * cx, inner_radius * cy, height))    # 3*n + i: inner top
    verts.append((0.0, 0.0, 0.0))                                       # 4*n:   underside center (z=0)
    verts.append((0.0, 0.0, bottom_h))                                  # 4*n+1: floor center (z=bottom_h)

    def OB(i): return i % n + 1
    def OT(i): return n + i % n + 1
    def IF(i): return 2 * n + i % n + 1   # inner floor rim, at z = bottom_h
    def IT(i): return 3 * n + i % n + 1
    UC = 4 * n + 1  # underside center (z=0)
    FC = 4 * n + 2  # interior floor center (z=bottom_h)

    faces = []
    for i in range(n):
        j = i + 1
        # Outer wall, normal points outward (+radial): CCW looking from outside.
        faces.append((OB(i), OB(j), OT(j), OT(i)))
        # Inner wall (floor rim up to open top), normal points inward (-radial).
        faces.append((IF(j), IF(i), IT(i), IT(j)))
        # Top rim annulus (wall thickness), normal points up (+Z) so the rim
        # reads as a solid edge rather than a hollow slot from above.
        faces.append((OT(i), OT(j), IT(j), IT(i)))
        # Interior floor disk at z=bottom_h, fan from floor center, up (+Z):
        # this is what the drone sees looking down into the tube.
        faces.append((FC, IF(i), IF(j)))
        # Exterior underside disk at z=0, fan from underside center, down (-Z).
        faces.append((UC, OB(j), OB(i)))
        # Top of the cavity stays open -- that's the point.

    # Flat per-face normals: without them Gazebo/Ogre can't light the mesh
    # and it renders as a flat white blob with no depth cues.
    def face_normal(f):
        p0, p1, p2 = verts[f[0] - 1], verts[f[1] - 1], verts[f[2] - 1]
        ax, ay, az = p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2]
        bx, by, bz = p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2]
        nx, ny, nz = ay * bz - az * by, az * bx - ax * bz, ax * by - ay * bx
        L = math.sqrt(nx * nx + ny * ny + nz * nz) or 1.0
        return nx / L, ny / L, nz / L

    lines = ['# procedurally generated closed-bottom tube, see field/gen_tube_mesh.py']
    for v in verts:
        lines.append(f'v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}')
    for f in faces:
        nx, ny, nz = face_normal(f)
        lines.append(f'vn {nx:.6f} {ny:.6f} {nz:.6f}')
    for k, f in enumerate(faces, start=1):
        lines.append('f ' + ' '.join(f'{vi}//{k}' for vi in f))
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
