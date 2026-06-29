#!/usr/bin/env python3
"""里程计发布节点。

底盘 SDK 未提供标准 nav_msgs/Odometry，但 system_state 中有车体坐标系下的速度：
  linear_velocity_x / linear_velocity_y / angular_velocity

本节点对速度做积分，发布：
  - /odom (nav_msgs/Odometry)
  - TF: odom -> base_link

注意：
  - 这是轮速积分里程计，会漂移；amcl / slam_toolbox 会通过激光修正 map->odom。
  - 不使用 system_state.current_pose（地图坐标系），避免依赖厂家定位。
  - 上电后位姿从 (0,0,0) 开始；建图/定位前请确保机器人静止或接受初始漂移。
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sr_amr_interfaces.msg import SystemState
from tf2_ros import TransformBroadcaster


def yaw_to_quaternion(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class OdomPublisher(Node):
    def __init__(self):
        super().__init__("odom_publisher")

        self.declare_parameter("system_state_topic", "/sr_amr_control/system_state")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("max_dt", 0.5)  # 超过该间隔视为断流，不积分

        self.system_state_topic = self.get_parameter("system_state_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.odom_frame = self.get_parameter("odom_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.publish_tf = self.get_parameter("publish_tf").value
        self.max_dt = float(self.get_parameter("max_dt").value)

        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._last_time = None

        self._odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            SystemState,
            self.system_state_topic,
            self._on_system_state,
            qos_profile_sensor_data,
        )

        self.get_logger().info(
            f"odom 积分：{self.system_state_topic} -> {self.odom_topic}, "
            f"TF {self.odom_frame}->{self.base_frame}"
        )

    def _on_system_state(self, msg: SystemState):
        now = self.get_clock().now()
        if self._last_time is None:
            self._last_time = now
            self._publish(now, msg)
            return

        dt = (now - self._last_time).nanoseconds * 1e-9
        if dt <= 0.0:
            return
        if dt > self.max_dt:
            self.get_logger().warn(
                f"system_state 间隔 {dt:.2f}s 过大，跳过积分",
                throttle_duration_sec=2.0,
            )
            self._last_time = now
            self._publish(now, msg)
            return

        vx = float(msg.linear_velocity_x)
        vy = float(msg.linear_velocity_y)
        wz = float(msg.angular_velocity)

        # 车体坐标系速度 -> 里程计坐标系位移（全向底盘）
        cos_y = math.cos(self._yaw)
        sin_y = math.sin(self._yaw)
        self._x += (vx * cos_y - vy * sin_y) * dt
        self._y += (vx * sin_y + vy * cos_y) * dt
        self._yaw += wz * dt
        self._yaw = math.atan2(math.sin(self._yaw), math.cos(self._yaw))

        self._last_time = now
        self._publish(now, msg, vx, vy, wz)

    def _publish(self, stamp_time, msg: SystemState, vx=0.0, vy=0.0, wz=0.0):
        if msg.estop_active:
            vx = vy = wz = 0.0

        odom = Odometry()
        odom.header.stamp = stamp_time.to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation = yaw_to_quaternion(self._yaw)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        self._odom_pub.publish(odom)

        if not self.publish_tf:
            return

        tf = TransformStamped()
        tf.header.stamp = odom.header.stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self._x
        tf.transform.translation.y = self._y
        tf.transform.rotation = odom.pose.pose.orientation
        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
