"""完整导航：定位 + Nav2 导航栈。

包含：
- localization.launch.py（bringup + map_server + amcl）
- Nav2：controller / planner / smoother / behaviors / bt_navigator / velocity_smoother

用法：
  ros2 launch helios_nav navigation.launch.py map:=/home/admin/maps/helios_map.yaml
之后在 RViz 用 2D Pose Estimate 初始化，再用 Nav2 Goal 发目标点。
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

    # (ROS 包名, 可执行文件/节点名)
    nav2_nodes = [
        ("nav2_controller", "controller_server"),
        ("nav2_smoother", "smoother_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_bt_navigator", "bt_navigator"),
        ("nav2_velocity_smoother", "velocity_smoother"),
    ]

    nodes = [
        DeclareLaunchArgument("map", description="地图 yaml 路径"),
        DeclareLaunchArgument("merge", default_value="false"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "localization.launch.py")),
            launch_arguments={"map": map_yaml, "merge": merge}.items(),
        ),
    ]

    for pkg_name, exe in nav2_nodes:
        nodes.append(Node(
            package=pkg_name,
            executable=exe,
            name=exe,
            output="screen",
            parameters=[nav2_params],
        ))

    nodes.append(Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "autostart": True,
            "node_names": [
                "controller_server",
                "smoother_server",
                "planner_server",
                "behavior_server",
                "bt_navigator",
                "velocity_smoother",
            ],
        }],
    ))

    return LaunchDescription(nodes)
