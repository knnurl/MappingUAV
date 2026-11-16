# filepath: src/map_export/setup.py
from setuptools import setup

package_name = 'map_export'

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
    maintainer_email='k.ural1@salford.ac.uk',
    description='Bounded /cloud_registered chunk recorder + offline merge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'recorder_node = map_export.recorder_node:main',
        ],
    },
)
