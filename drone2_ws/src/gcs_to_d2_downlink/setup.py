from setuptools import find_packages, setup

package_name = 'gcs_to_d2_downlink'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/downlink_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Maintainer',
    maintainer_email='maintainer@example.com',
    description='Convert GCS orders into Drone-2 local triggers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gcs_to_d2_downlink_node = gcs_to_d2_downlink.gcs_to_d2_downlink_node:main',
        ],
    },
)
