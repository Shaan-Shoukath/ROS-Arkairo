from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'detection_and_geotag'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Disease detection and GPS geotagging for Drone-1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_and_geotag_node = detection_and_geotag.detection_and_geotag_node:main',
            'detection_test_node = detection_and_geotag.detection_test_node:main',
        ],
    },
)
