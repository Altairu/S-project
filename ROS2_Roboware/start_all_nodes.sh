#!/bin/bash
colcon build
# Set up the ROS 2 environment
source /opt/ros/humble/setup.bash
source ~/S-project/ROS2_Roboware/install/setup.bash

# Start controller_node in a new tab
gnome-terminal --tab --title="controller_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/S-project/ROS2_Roboware/install/setup.bash; ros2 run robot_controller controller_node; exec bash"

# Start serial_read_node in a new tab
gnome-terminal --tab --title="serial_read_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/S-project/ROS2_Roboware/install/setup.bash; ros2 run robot_controller serial_read_node; exec bash"

# Start serial_send_node in a new tab
gnome-terminal --tab --title="serial_send_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/S-project/ROS2_Roboware/install/setup.bash; ros2 run robot_controller serial_send_node; exec bash"

# Start rviz_position_node in a new tab
gnome-terminal --tab --title="rviz_position_node" -- bash -c "source /opt/ros/humble/setup.bash; source ~/S-project/ROS2_Roboware/install/setup.bash; ros2 run robot_controller rviz_position_node; exec bash"

# Gazebo の起動
#gnome-terminal --tab --title="Gazebo" -- bash -c "source /opt/ros/humble/setup.bash; source ~/S-project/ROS2_Roboware/install/setup.bash; ros2 launch robot_controller robot_gazebo_launch.py; exec bash"
