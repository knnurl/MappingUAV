# filepath: src/lio_health_guard/setup.py
from setuptools import setup

package_name = 'lio_health_guard'

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
    description='Derived FAST-LIO health guard',
    license='MIT',
    entry_points={
        'console_scripts': [
            'guard_node = lio_health_guard.guard_node:main',
        ],
    },
)
