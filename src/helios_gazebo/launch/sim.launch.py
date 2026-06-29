"""Helios 仿真基础环境（不含 SLAM/Nav2）。

启动内容：
- Gazebo + minimal world
- robot_state_publisher（发布 URDF 的 TF）
- spawn 机器人到 Gazebo
- laser_preprocess（合并前后雷达 -> /scan，并把无效值转 inf）
- RViz（可选）

Gazebo 的 planar_move 插件负责 /cmd_vel 订阅、/odom 与 odom->base_footprint TF。

用法：
  ros2 launch helios_gazebo sim.launch.py
然后可用键盘遥控验证移动：
  ros2 run teleop_twist_keyboard teleop_twist_keyboard
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_desc = get_package_share_directory("helios_description")
    pkg_gazebo = get_package_share_directory("helios_gazebo")
    pkg_gazebo_ros = get_package_share_directory("gazebo_ros")

    urdf = os.path.join(pkg_desc, "urdf", "helios.urdf.xacro")
    rviz_cfg = os.path.join(pkg_desc, "rviz", "display.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rviz = LaunchConfiguration("rviz")
    merge = LaunchConfiguration("merge")
    world = LaunchConfiguration("world")

    robot_description = ParameterValue(Command(["xacro ", urdf]), value_type=str)

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, "launch", "gazebo.launch.py")
        ),
        launch_arguments={"world": world, "verbose": "false"}.items(),
    )

    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    spawn = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic", "robot_description",
            "-entity", "helios",
            "-x", "0.0", "-y", "0.0", "-z", "0.2",
        ],
        output="screen",
    )
    delayed_spawn = TimerAction(period=5.0, actions=[spawn])

    laser_preprocess = Node(
        package="helios_nav",
        executable="laser_preprocess",
        name="laser_preprocess",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "front_topic": "/sr_amr_control/front/scan",
            "rear_topic": "/sr_amr_control/rear/scan",
            "output_topic": "/scan",
            "merge": merge,
            "target_frame": "base_link",
            "invalid_value": 101.0,
            "range_min": 0.10,
            "range_max": 25.0,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_cfg],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("merge", default_value="true",
                              description="true=合并前后雷达为 /scan"),
        DeclareLaunchArgument(
            "world",
            default_value=PathJoinSubstitution(
                [FindPackageShare("helios_gazebo"), "worlds", "minimal.world"]
            ),
            description="Gazebo world 文件路径",
        ),
        gazebo,
        rsp,
        delayed_spawn,
        laser_preprocess,
        rviz,
    ])
