"""底盘对接层 bringup。

启动：
- laser_preprocess：雷达 101.0->inf 过滤（可选合并双雷达）-> /scan
- odom_publisher  ：system_state 速度积分 -> /odom + odom->base_link TF
- cmd_vel_relay   ：/cmd_vel -> 底盘 remote_control_cmd_vel（并开启远程控制）
- static_tf       ：占位雷达外参（真实外参确认后改 static_tf.launch.py）

不包含 SLAM / 导航，供建图或导航 launch 复用。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory("helios_nav")

    merge = LaunchConfiguration("merge")
    use_static_tf = LaunchConfiguration("use_static_tf")
    relay_cmd_vel = LaunchConfiguration("relay_cmd_vel")
    use_odom = LaunchConfiguration("use_odom")

    return LaunchDescription([
        DeclareLaunchArgument("merge", default_value="false",
                              description="是否合并前后雷达为单个 /scan"),
        DeclareLaunchArgument("use_static_tf", default_value="true",
                              description="是否发布占位雷达 TF"),
        DeclareLaunchArgument("relay_cmd_vel", default_value="true",
                              description="是否启用 cmd_vel 中继到底盘"),
        DeclareLaunchArgument("use_odom", default_value="true",
                              description="是否从 system_state 积分发布 odom"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, "launch", "static_tf.launch.py")),
            condition=IfCondition(use_static_tf),
        ),

        Node(
            package="helios_nav",
            executable="laser_preprocess",
            name="laser_preprocess",
            output="screen",
            parameters=[{
                "front_topic": "/sr_amr_control/front/scan",
                "rear_topic": "/sr_amr_control/rear/scan",
                "output_topic": "/scan",
                "merge": merge,
                "target_frame": "base_link",
                "invalid_value": 101.0,
                "range_min": 0.05,
                "range_max": 25.0,
            }],
        ),

        Node(
            package="helios_nav",
            executable="odom_publisher",
            name="odom_publisher",
            output="screen",
            condition=IfCondition(use_odom),
            parameters=[{
                "system_state_topic": "/sr_amr_control/system_state",
                "odom_topic": "/odom",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "publish_tf": True,
            }],
        ),

        Node(
            package="helios_nav",
            executable="cmd_vel_relay",
            name="cmd_vel_relay",
            output="screen",
            condition=IfCondition(relay_cmd_vel),
            parameters=[{
                "in_topic": "/cmd_vel",
                "out_topic": "/sr_amr_control/remote_control_cmd_vel",
                "enable_on_start": True,
                "oba_on_start": True,
            }],
        ),
    ])
