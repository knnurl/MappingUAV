# filepath: src/px4_odom_bridge/setup.py
from setuptools import setup

package_name = 'px4_odom_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kaan',
    maintainer_email='kaan@example.com',
    description='FAST-LIO2 ENU/FLU to PX4 NED/FRD VehicleOdometry bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'odom_bridge_node = px4_odom_bridge.odom_bridge_node:main',
        ],
    },
)
