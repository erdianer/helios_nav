"""建图：bringup + slam_toolbox（async 在线建图）。

用法：
  ros2 launch helios_nav mapping.launch.py
建图完成后保存地图：
  ros2 run nav2_map_server map_saver_cli -f ~/maps/helios_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("helios_nav")
    slam_params = os.path.join(pkg, "config", "slam_toolbox.yaml")

    merge = LaunchConfiguration("merge")

    return LaunchDescription([
        DeclareLaunchArgument("merge", default_value="false",
                              description="是否合并前后雷达（建图建议先用单雷达验证）"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "bringup.launch.py")),
            launch_arguments={"merge": merge, "relay_cmd_vel": "true"}.items(),
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_params],
        ),
    ])
