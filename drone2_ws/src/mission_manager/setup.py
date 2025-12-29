from setuptools import find_packages, setup
package_name = 'mission_manager'
setup(
    name=package_name, version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/manager_params.yaml']),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Maintainer', maintainer_email='maintainer@example.com',
    description='Mission state management and GCS reporting', license='Apache-2.0',
    entry_points={'console_scripts': [
        'mission_manager_node = mission_manager.mission_manager_node:main',
    ]},
)
