from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Install SDF world file
        (os.path.join('share', package_name, 'worlds'),
            ['../ares_alex_default_world.sdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moham',
    maintainer_email='mzaitoun@purdue.edu',
    description='ARES Autonomous Drone Control System',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'nmpc_node = app.Nodes.NMPC_Node:main',
        ],
    },
)
