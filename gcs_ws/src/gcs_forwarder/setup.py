from setuptools import find_packages, setup

package_name = 'gcs_forwarder'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/forwarder_params.yaml']),
    ],
    install_requires=['setuptools', 'pymavlink'],
    zip_safe=True,
    maintainer='Shaan Shoukath',
    maintainer_email='shaan@example.com',
    description='GCS geotag forwarder - relays from Drone 1 to Drone 2',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'forwarder_node = gcs_forwarder.forwarder_node:main',
        ],
    },
)
