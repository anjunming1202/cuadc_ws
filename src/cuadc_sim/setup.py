import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cuadc_sim'


def data_files_tree(src_root, dest_root):
    """(dest_dir, [files]) pairs for every file under src_root, preserving
    its subdirectory structure under dest_root -- worlds/cuadc_field.sdf
    references its textures via relative paths (../models/...), so the
    install layout must mirror the source layout for those to still
    resolve once installed (see field/gen_world.py)."""
    pairs = []
    for dirpath, _dirnames, filenames in os.walk(src_root):
        if not filenames:
            continue
        # dirpath is already relative (e.g. "models/cuadc_ground/.../textures")
        # since src_root itself is relative -- keep that prefix verbatim.
        pairs.append((os.path.join(dest_root, dirpath),
                       [os.path.join(dirpath, f) for f in filenames]))
    return pairs


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config'), glob('config/*.parm')),
        *data_files_tree('models', os.path.join('share', package_name)),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junming An',
    maintainer_email='anjunming0819@outlook.com',
    description='CUADC 2026 simulation assets: SITL bringup, verification nodes, and (later) Gazebo world/models',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 操作性验证节点在 tests/ 子包（手动 ros2 run，非 pytest）
            'sitl_smoke_test = cuadc_sim.tests.sitl_smoke_test:main',
            'flight_acceptance_test = cuadc_sim.tests.flight_acceptance_test:main',
            'detection_delay_node = cuadc_sim.detection_delay_node:main',
        ],
    },
)
