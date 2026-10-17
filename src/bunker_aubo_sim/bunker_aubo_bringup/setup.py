import os
from glob import glob

from setuptools import setup

package_name = 'bunker_aubo_bringup'

setup(
    name=package_name,
    version='1.1.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        (os.path.join('lib', package_name), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pradeep Nahak',
    maintainer_email='praddepnahak2008@gmail.com',
    description='Single-command Gazebo Harmonic bring-up for the BUNKER + AUBO-i5 mobile manipulator.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'base_cmd_relay = bunker_aubo_bringup.base_cmd_relay:main',
            'gripper_action_server = bunker_aubo_bringup.gripper_action_server:main',
            'smoke_check = bunker_aubo_bringup.smoke_check:main',
            'track_visualizer = bunker_aubo_bringup.track_visualizer:main',
        ],
    },
)
