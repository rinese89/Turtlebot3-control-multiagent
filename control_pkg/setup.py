import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'control_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'turtle_control.launch.py'))),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'turtlebot3_state_publisher.launch.py'))),
    (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', 'ld08.launch.py'))),
    (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.json'))),
    (os.path.join('share', package_name, 'param'), glob(os.path.join('param', '*.yaml'))),
    
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'controller_node = control_pkg.controller_node:main',
                'broadcaster_reference = control_pkg.broadcaster_reference:main',
                'turtlebot3_ros=turtlebot3_node.turtlebot3_ros:main',
                'aruco_nav_node = control_pkg.aruco_nav_node:main',
                'aruco_PID_node = control_pkg.aruco_PID_node:main',
                'camera_servo_node = control_pkg.camera_servo_node:main',
                'control_follow_line = control_pkg.control_follow_line:main',
        ],
    },
)
