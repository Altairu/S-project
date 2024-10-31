from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

def generate_launch_description():
    # URDFファイルのパス
    urdf_path = '/home/altair/S-project/ROS2_Roboware/src/robot_controller/urdf/robot.urdf'

    # URDFファイルを読み込み
    with open(urdf_path, 'r') as urdf_file:
        robot_description_content = urdf_file.read()

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}],
        ),

        # Gazeboを起動
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        )
    ])
