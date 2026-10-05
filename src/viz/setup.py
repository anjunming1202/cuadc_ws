import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'viz'


def _web_data_files():
    """把 web/ 下所有文件按相对路径原样装进 share/viz/web/，
    静态文件服务器（http.server）直接指到这个安装后的目录。"""
    files = []
    for root, _dirs, filenames in os.walk('web'):
        if not filenames:
            continue
        rel_root = os.path.relpath(root, 'web')
        dest = os.path.join('share', package_name, 'web', rel_root) if rel_root != '.' \
            else os.path.join('share', package_name, 'web')
        files.append((dest, [os.path.join(root, f) for f in filenames]))
    return files


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ] + _web_data_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Junming An',
    maintainer_email='anjunming0819@outlook.com',
    description='地面站实时监测 dashboard，契约见 docs/spec/04-interfaces.md、05-perception-decision-interface.md',
    license='MIT',
    entry_points={'console_scripts': []},
)
