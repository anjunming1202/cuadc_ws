import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cuadc_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # config yaml 随包一起装,节点用 __file__ 相对路径读:检测器 sim/real 档在
    # detectors/(见 color_pad.py load_config);相机内参 sim/real 档 camera.<profile>.yaml
    # 在包根(见 geometry.load_camera_params)。
    package_data={
        'cuadc_perception': ['*.yaml'],
        'cuadc_perception.detectors': ['*.yaml'],
    },
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
            # 无-ROS 预览模式(相机自检/调参 + ground-monitor 按名引用它),见 ADR-0008
            'cv_preview = cuadc_perception.cv_preview:main',
        ],
    },
)
