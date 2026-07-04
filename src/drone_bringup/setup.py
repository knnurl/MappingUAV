# filepath: src/drone_bringup/setup.py
import os
from glob import glob
from setuptools import setup

package_name = 'drone_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
         [f for f in glob('config/*') if os.path.isfile(f)]),
        (os.path.join('share', package_name, 'config', 'px4'), glob('config/px4/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaan',
    maintainer_email='kaan@example.com',
    description='Launch and configuration for GPS-denied X500 Gates 1-4 (no nodes)',
    license='MIT',
    entry_points={'console_scripts': []},
)
