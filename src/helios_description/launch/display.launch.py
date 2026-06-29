"""仅可视化 URDF：robot_state_publisher + joint_state_publisher_gui + RViz。

不需要 Gazebo，最快验证模型与 TF 是否正确：
  ros2 launch helios_description display.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("helios_description")
    urdf = os.path.join(pkg, "urdf", "helios.urdf.xacro")
    rviz_cfg = os.path.join(pkg, "rviz", "display.rviz")

    use_gui = LaunchConfiguration("gui")
    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true",
                              description="是否打开 joint_state_publisher_gui"),

        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            condition=IfCondition(use_gui),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            arguments=["-d", rviz_cfg],
            output="screen",
        ),
    ])
