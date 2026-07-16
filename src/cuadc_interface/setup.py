import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cuadc_interface'

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
    description='接口层：与 ArduPilot 的 MAVLink 通信桥接节点，契约见 docs/spec/04-interfaces.md',
    license='MIT',
    entry_points={
        'console_scripts': [
            'interface_node = cuadc_interface.interface_node:main',
            'interface_acceptance_test = cuadc_interface.interface_acceptance_test:main',
            'interface_maneuver_test = cuadc_interface.interface_maneuver_test:main',
            'interface_precision_test = cuadc_interface.interface_precision_test:main',
            'interface_velocity_test = cuadc_interface.interface_velocity_test:main',
        ],
    },
)
