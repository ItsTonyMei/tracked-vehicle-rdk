from setuptools import setup
import os
from glob import glob

package_name = 'tracked_vehicle'

setup(
    name=package_name,
    version='0.3.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sunrise',
    maintainer_email='sunrise@rdkx5.local',
    description='6WD heavy tracked vehicle — RDK X5 autonomous follower',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_bridge = tracked_vehicle.cmd_vel_bridge:main',
        ],
    },
)
