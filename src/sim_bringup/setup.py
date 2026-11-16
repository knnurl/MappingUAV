# filepath: src/sim_bringup/setup.py
import os
from glob import glob

from setuptools import setup

package_name = 'sim_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaan',
    maintainer_email='k.ural1@salford.ac.uk',
    description='Closed-loop rig (kinematic fake PX4 + raycast LiDAR world)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'fake_px4 = sim_bringup.fake_px4:main',
            'fake_lidar = sim_bringup.fake_lidar:main',
            'sim_mission = sim_bringup.sim_mission:main',
            'sim_scenarios = sim_bringup.sim_scenarios:main',
        ],
    },
)
