# filepath: src/planner_shim/setup.py
from setuptools import setup

package_name = 'planner_shim'

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
    description='go_to_with_avoidance wrapper (WP-D)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'go_to_with_avoidance = planner_shim.go_to_with_avoidance_node:main',
        ],
    },
)
