from setuptools import find_packages, setup
package_name = 'local_detection'
setup(
    name=package_name, version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/detection_params.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Maintainer', maintainer_email='maintainer@example.com',
    description='Local disease confirmation for Drone-2', license='Apache-2.0',
    entry_points={'console_scripts': [
        'local_detection_node = local_detection.local_detection_node:main',
    ]},
)
