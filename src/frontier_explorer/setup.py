# filepath: src/frontier_explorer/setup.py
from setuptools import setup

package_name = 'frontier_explorer'

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
    description='Frontier-based exploration goal selection (WP-F)',
    license='MIT',
    entry_points={
        'console_scripts': [
            'explorer_node = frontier_explorer.explorer_node:main',
        ],
    },
)
