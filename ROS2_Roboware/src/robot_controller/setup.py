from setuptools import find_packages, setup

package_name = 'robot_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # launchディレクトリをインストールに追加
        ('share/' + package_name + '/launch', ['launch/robot_gazebo_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='altair',
    maintainer_email='Altairu@github.comu',
    description='ROS2 package for robot control',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'controller_node = robot_controller.controller_node:main',
        'serial_send_node = robot_controller.serial_send_node:main',
        'serial_read_node = robot_controller.serial_read_node:main',
        'rviz_position_node = robot_controller.rviz_position_node:main',
    ],
    },
)
