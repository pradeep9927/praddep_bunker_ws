import os
from glob import glob

from setuptools import setup

package_name = 'bunker_aubo_slam'

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
    description='RTAB-Map multi-sensor SLAM (VLP-16 + 2x D435 + IMU + track odometry) with evaluation tools.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'multi_rgbd_sync = bunker_aubo_slam.multi_rgbd_sync:main',
            'lidar_self_filter = bunker_aubo_slam.lidar_self_filter:main',
            'slam_evaluator = bunker_aubo_slam.slam_evaluator:main',
            'export_maps = bunker_aubo_slam.export_maps:main',
            'evaluate_map = bunker_aubo_slam.evaluate_map:main',
            'fill_results = bunker_aubo_slam.results_report:main',
        ],
    },
)
