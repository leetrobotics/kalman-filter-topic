from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'kalman_filter'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test', 'starter', 'solution']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        # Include world files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.sdf')),
        # Include Foxglove layout
        (os.path.join('share', package_name, 'config', 'foxglove'),
         glob('config/foxglove/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Leet Robotics',
    maintainer_email='learn@leet-robotics.com',
    description='Learn Kalman Filtering with hands-on robotics examples',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kalman_node = kalman_filter.kalman_node:main',
            'waypoint_driver = kalman_filter.waypoint_driver:main',
            'noisy_odom = kalman_filter.noisy_odom:main',
            'ground_truth = kalman_filter.ground_truth:main',
            'teleop_bridge = kalman_filter.teleop_bridge:main',
        ],
    },
)
