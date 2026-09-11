import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cuadc_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junming An',
    maintainer_email='anjunming0819@outlook.com',
    description='感知/视觉模块：目标（圆筒）检测与定位，契约见 docs/spec/05-perception-decision-interface.md',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stub_perception_node = cuadc_perception.stub_perception_node:main',
            'cv_perception_node = cuadc_perception.cv_perception_node:main',
        ],
    },
)
