#!/usr/bin/env python3
"""仿真机型 model.sdf 与机体几何单一真源 cuadc_airframe/airframe.yaml 的一致性校验。
SDF 不能读 yaml，几何只能写两遍，这里把两遍对齐：改了 airframe.yaml 就改 SDF 再跑本脚本。
SDF 机体系是 前/左/上，airframe 是 前/右/下。退出码 0 = 一致。

    python3 src/cuadc_sim/field/check_airframe.py
"""
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent.parent / 'cuadc_airframe'))
import cuadc_airframe  # noqa: E402

SDF = HERE.parent / 'models' / 'cuadc_iris' / 'model.sdf'
TOL = 1e-3


def _pose(el):
    return [float(v) for v in el.find('pose').text.split()]


def main():
    af = cuadc_airframe.load()
    model = ET.parse(SDF).getroot().find('model')
    fails = []

    def check(name, got, want):
        ok = all(abs(g - w) <= TOL for g, w in zip(got, want))
        print(f'{"OK " if ok else "BAD"} {name}: sdf={tuple(round(g, 4) for g in got)} '
              f'airframe={tuple(round(w, 4) for w in want)}')
        if not ok:
            fails.append(name)

    cam = next(l for l in model.findall('link') if l.get('name') == 'camera_link')
    x, y, z = _pose(cam)[:3]
    check('camera position (F/R/D)', (x, -y, -z), af.camera_position_m)
    sensor = cam.find('sensor')
    r, pch, yw = _pose(sensor)[3:6]
    check('camera mount roll/pitch/yaw', (r, pch, -yw), af.camera_mount_rad)  # 前左上→前右下 yaw 反号

    base = next(l for l in model.findall('link') if l.get('name') == 'base_link')
    skids = [c for c in base.findall('collision') if c.get('name', '').startswith('skid')]
    bottoms = [_pose(c)[2] - float(c.find('geometry/cylinder/radius').text) for c in skids]
    check('fc_rest_clearance (skid bottom below FC)', (-min(bottoms),), (af.fc_rest_clearance_m,))

    bottles = {m.get('name'): _pose(m) for m in model.iter('model') if m.get('name', '').startswith('bottle_')}
    for port in af.drop_ports:
        p = bottles.get(f'bottle_{port.name}')
        if p is None:
            print(f'BAD drop port {port.name}: no bottle_{port.name} in SDF')
            fails.append(port.name)
            continue
        check(f'drop port {port.name} (F/R)', (p[0], -p[1]), port.position_m)

    print('RESULT:', 'PASS' if not fails else f'FAIL {fails}')
    return 0 if not fails else 1


if __name__ == '__main__':
    sys.exit(main())
