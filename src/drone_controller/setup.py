from setuptools import find_packages, setup

package_name = 'drone_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='guy',
    maintainer_email='112532019+ituretsky@users.noreply.github.com',
    description='ROS 2 package for controlling ArduPilot drones',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'controller_node = drone_controller.controller_node:main',
        ],
    },
)
