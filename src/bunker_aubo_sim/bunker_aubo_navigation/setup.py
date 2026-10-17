import os
from glob import glob

from setuptools import setup

package_name = 'bunker_aubo_navigation'

setup(
    name=package_name,
    version='1.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pradeep Nahak',
    maintainer_email='praddepnahak2008@gmail.com',
    description='Nav2 autonomous navigation for the BUNKER + AUBO-i5 (RTAB-Map or AMCL localization).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'go_to = bunker_aubo_navigation.go_to:main',
        ],
    },
)
