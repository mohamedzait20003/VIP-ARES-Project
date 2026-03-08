from setuptools import find_packages, setup

package_name = 'app'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'demo_publisher = app.demo_publisher:main',
            'demo_subscriber = app.demo_subscriber:main',
        ],
    },
)
