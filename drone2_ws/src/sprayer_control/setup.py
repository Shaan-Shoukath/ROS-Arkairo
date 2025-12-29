from setuptools import find_packages, setup
package_name = 'sprayer_control'
setup(
    name=package_name, version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sprayer_params.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Maintainer', maintainer_email='maintainer@example.com',
    description='Spray actuation with safety checks', license='Apache-2.0',
    entry_points={'console_scripts': [
        'sprayer_control_node = sprayer_control.sprayer_control_node:main',
    ]},
)
