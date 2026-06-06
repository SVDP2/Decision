from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'platoon_v2v'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='xytron@example.com',
    description='Shared V2V nodes for leader-follower platooning',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'fake_leader_motion_node = platoon_v2v.fake_leader_motion_node:main',
            (
                'follower_reference_path_adapter_node = '
                'platoon_v2v.follower_reference_path_adapter_node:main'
            ),
            'follower_v2v_receiver_node = platoon_v2v.follower_v2v_receiver_node:main',
            'leader_preview_path_node = platoon_v2v.leader_preview_path_node:main',
            (
                'leader_reference_path_relay_node = '
                'platoon_v2v.leader_reference_path_relay_node:main'
            ),
            'leader_v2v_adapter_node = platoon_v2v.leader_v2v_adapter_node:main',
        ],
    },
)
