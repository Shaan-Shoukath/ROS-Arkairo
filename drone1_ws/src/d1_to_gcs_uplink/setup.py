from setuptools import find_packages, setup

package_name = 'd1_to_gcs_uplink'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/uplink_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Forward disease geotags from Drone-1 to GCS',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'd1_to_gcs_uplink_node = d1_to_gcs_uplink.d1_to_gcs_uplink_node:main',
        ],
    },
)
