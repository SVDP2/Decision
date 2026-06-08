import os
from glob import glob

from setuptools import find_packages, setup


package_name = 'um980_driver'
doc_files = [
    'readme.md',
]

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'docs'), doc_files),
        (os.path.join('lib', package_name), glob('scripts/*')),
    ],
    install_requires=['setuptools', 'pyserial', 'textual', 'PyYAML'],
    zip_safe=True,
    maintainer='user1',
    maintainer_email='kikiws70@gmail.com',
    description='UM980 GNSS receiver driver with NMEA parsing, RTCM forwarding, ROS2 topics, and Textual TUI.',
    license='Apache-2.0',
    tests_require=['pytest'],
    options={
        'develop': {'script_dir': '$base/lib/um980_driver'},
        'install': {'install_scripts': '$base/lib/um980_driver'},
    },
    entry_points={
        'console_scripts': [
            'um980_console = um980_driver.console:main',
            'um980_tui = um980_driver.tui:main',
            'um980_driver_node = um980_driver.ros2_node:main',
        ],
    },
)
