from glob import glob

from setuptools import find_packages, setup

package_name = 'auto_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoo',
    maintainer_email='smzzang21@konkuk.ac.kr',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'roi_path_node = auto_drive.roi_path_node:main',
            'pure_pursuit_node = auto_drive.pure_pursuit_node:main',
            'complex_target_node = auto_drive.complex_target_node:main',
            'complex_rrt_planner_node = '
            'auto_drive.complex_rrt_planner_node:main',
            'command_mux_node = auto_drive.command_mux_node:main',
            'mission_supervisor_node = '
            'auto_drive.mission_supervisor_node:main',
            'mission_zone_node = auto_drive.mission_zone_node:main',
            'traffic_signal_gate_node = '
            'auto_drive.traffic_signal_gate_node:main',
        ],
    },
)
