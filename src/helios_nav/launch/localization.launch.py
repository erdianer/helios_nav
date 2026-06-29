"""定位：bringup + map_server + amcl（带生命周期管理）。

用法：
  ros2 launch helios_nav localization.launch.py map:=/home/admin/maps/helios_map.yaml
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
    nav2_params = os.path.join(pkg, "config", "nav2_params.yaml")

    map_yaml = LaunchConfiguration("map")
    merge = LaunchConfiguration("merge")

    return LaunchDescription([
        DeclareLaunchArgument("map", description="地图 yaml 路径"),
        DeclareLaunchArgument("merge", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "bringup.launch.py")),
            launch_arguments={"merge": merge, "relay_cmd_vel": "true"}.items(),
        ),

        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[{"yaml_filename": map_yaml}],
        ),
        Node(
            package="nav2_amcl",
            executable="amcl",
            name="amcl",
            output="screen",
            parameters=[nav2_params],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_localization",
            output="screen",
            parameters=[{
                "autostart": True,
                "node_names": ["map_server", "amcl"],
            }],
        ),
    ])
