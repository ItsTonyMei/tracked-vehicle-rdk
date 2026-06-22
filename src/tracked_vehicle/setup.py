from setuptools import setup
import os
from glob import glob

package_name = 'tracked_vehicle'

# 收集 launch 和 config 文件 (相对于项目根目录)
project_root = os.path.join(os.path.dirname(__file__), '..', '..')
launch_files = glob(os.path.join(project_root, 'launch', '*.py'))
config_files = glob(os.path.join(project_root, 'config', '*.yaml'))

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', launch_files),
        ('share/' + package_name + '/config', config_files),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@rdkx5.local',
    description='6WD heavy tracked vehicle — RDK X5 autonomous follower',
    license='MIT',
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = tracked_vehicle.cmd_vel_bridge:main',
            'person_tracker = tracked_vehicle.person_tracker:main',
            'display_node = tracked_vehicle.display_node:main',
        ],
    },
)
