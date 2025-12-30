from setuptools import setup
import os
from glob import glob

package_name = 'local_detection_and_centering'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Shaan Shoukath',
    maintainer_email='shaan@example.com',
    description='Combined local disease detection and visual servoing for Drone-2',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detection_centering_node = local_detection_and_centering.detection_centering_node:main',
        ],
    },
)
