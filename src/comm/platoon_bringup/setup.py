from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'platoon_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (
            os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') + glob('config/*.xml'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='xytron',
    maintainer_email='xytron@example.com',
    description='Canonical launch wrappers for leader/follower platoon runtime',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'svdp_leader_internal = platoon_bringup.svdp_runtime:leader_internal_main',
            'svdp_leader_bridge = platoon_bringup.svdp_runtime:leader_bridge_main',
            'svdp_follower_internal = platoon_bringup.svdp_runtime:follower_internal_main',
            'svdp_follower_bridge = platoon_bringup.svdp_runtime:follower_bridge_main',
        ],
    },
)
