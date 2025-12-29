from setuptools import find_packages, setup
package_name = 'centering_controller'
setup(
    name=package_name, version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/centering_params.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Maintainer', maintainer_email='maintainer@example.com',
    description='Fine position alignment for spray targeting', license='Apache-2.0',
    entry_points={'console_scripts': [
        'centering_controller_node = centering_controller.centering_controller_node:main',
    ]},
)
