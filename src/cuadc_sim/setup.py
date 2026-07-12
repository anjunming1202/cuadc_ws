import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cuadc_sim'

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
            'sitl_smoke_test = cuadc_sim.sitl_smoke_test:main',
            'flight_acceptance_test = cuadc_sim.flight_acceptance_test:main',
        ],
    },
)
