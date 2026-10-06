import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'cuadc_decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # 各算法/配置层的 sim/real yaml 跟着各自源码放,随包一起装,节点用 __file__ 相对
    # 路径读(见各 load_config/load_tuning/mission.load)。组织同 cuadc_perception/detectors。
    package_data={
        'cuadc_decision': ['*.yaml'],   # search_strategy.*, mission_logic.* (FSM 本体调参)
        'cuadc_decision.approaches': ['*.yaml'],
        'cuadc_decision.surveys': ['*.yaml'],
        'cuadc_decision.mission': ['*.yaml', 'scenarios/*.yaml'],  # 世界/任务/场景层(A)
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
    description='规划/决策模块：任务状态机，契约见 docs/spec/06-planning.md',
    license='MIT',
    entry_points={
        'console_scripts': [
            'decision_node = cuadc_decision.decision_node:main',
        ],
    },
)
