from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'platoon_localization'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='xytron@example.com',
    description='Launch adapters for shared platoon localization',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            (
                'leader_vehicle_model_marker_node = '
                'platoon_localization.leader_vehicle_model_marker_node:main'
            ),
            (
                'relative_aruco_leader_node = '
                'platoon_localization.relative_aruco_leader_node:main'
            ),
            (
                'relative_gps_leader_node = '
                'platoon_localization.relative_gps_leader_node:main'
            ),
            (
                'relative_lidar_leader_node = '
                'platoon_localization.relative_lidar_leader_node:main'
            ),
            (
                'utm_offset_gps_odom_node = '
                'platoon_localization.utm_offset_gps_odom_node:main'
            ),
        ],
    },
)
