"""占位静态 TF（真实外参确认前使用）。

发布 base_link -> 前/后雷达 link 的静态变换。
当前为占位值（仅前后偏移示意），真实安装位姿确认后必须替换 x/y/z/yaw。
雷达 frame_id 需与底盘 SDK launch 时设置的一致：
  lidar_front_frame_id / lidar_rear_frame_id
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    front_frame = LaunchConfiguration("front_frame")
    rear_frame = LaunchConfiguration("rear_frame")
    base_frame = LaunchConfiguration("base_frame")

    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("front_frame", default_value="right_front_laser_link"),
        DeclareLaunchArgument("rear_frame", default_value="left_behind_laser_link"),

        # TODO: 用真实外参替换 --x/--y/--z/--yaw（占位值）
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_to_front_laser",
            arguments=[
                "--x", "0.30", "--y", "0.0", "--z", "0.20",
                "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", base_frame, "--child-frame-id", front_frame,
            ],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="tf_base_to_rear_laser",
            arguments=[
                "--x", "-0.30", "--y", "0.0", "--z", "0.20",
                "--yaw", "3.14159", "--pitch", "0.0", "--roll", "0.0",
                "--frame-id", base_frame, "--child-frame-id", rear_frame,
            ],
        ),
    ])
