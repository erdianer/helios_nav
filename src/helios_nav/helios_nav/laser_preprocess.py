#!/usr/bin/env python3
"""雷达预处理节点。

职责：
1. 把底盘 SDK 发布的前/后 LaserScan 中的无效占位值（默认 101.0）转换为 inf，
   否则 slam_toolbox / amcl / costmap 会把它当成真实远距离回波，污染地图与定位。
2. 可选：把前后两个雷达合并为一个 base_link 下的 360° LaserScan，供 SLAM / AMCL 使用。

合并依赖 TF：target_frame <- 各雷达 frame。真实外参确认前，可用 static_tf.launch.py
发布占位 TF；正式使用必须填入真实安装位姿。

模式：
- merge=false：仅对 front_topic 做无效值过滤后输出（单雷达验证最简单）。
- merge=true ：前后雷达都转换到 target_frame 后合并为单帧输出。
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

import tf2_ros


def yaw_from_quat(x, y, z, w):
    """从四元数提取绕 Z 轴 yaw（2D 投影用）。"""
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LaserPreprocess(Node):
    def __init__(self):
        super().__init__("laser_preprocess")

        # ---- 参数 ----
        self.declare_parameter("front_topic", "/sr_amr_control/front/scan")
        self.declare_parameter("rear_topic", "/sr_amr_control/rear/scan")
        self.declare_parameter("output_topic", "/scan")
        self.declare_parameter("merge", False)
        self.declare_parameter("target_frame", "base_link")
        # 无效占位值（底盘用 101.0 表示无回波），等于或大于该值视为无效
        self.declare_parameter("invalid_value", 101.0)
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 25.0)
        # 合并输出的角度配置
        self.declare_parameter("out_angle_min", -math.pi)
        self.declare_parameter("out_angle_max", math.pi)
        self.declare_parameter("out_angle_increment", math.radians(1.0))

        self.front_topic = self.get_parameter("front_topic").value
        self.rear_topic = self.get_parameter("rear_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.merge = self.get_parameter("merge").value
        self.target_frame = self.get_parameter("target_frame").value
        self.invalid_value = float(self.get_parameter("invalid_value").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.out_angle_min = float(self.get_parameter("out_angle_min").value)
        self.out_angle_max = float(self.get_parameter("out_angle_max").value)
        self.out_angle_increment = float(self.get_parameter("out_angle_increment").value)

        self.pub = self.create_publisher(LaserScan, self.output_topic, qos_profile_sensor_data)

        if self.merge:
            self.tf_buffer = tf2_ros.Buffer()
            self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
            self._front = None
            self._rear = None
            self.create_subscription(
                LaserScan, self.front_topic, self._on_front, qos_profile_sensor_data
            )
            self.create_subscription(
                LaserScan, self.rear_topic, self._on_rear, qos_profile_sensor_data
            )
            self.get_logger().info(
                f"merge 模式：{self.front_topic} + {self.rear_topic} -> {self.output_topic}"
                f"（target_frame={self.target_frame}）"
            )
        else:
            self.create_subscription(
                LaserScan, self.front_topic, self._on_single, qos_profile_sensor_data
            )
            self.get_logger().info(
                f"单雷达过滤模式：{self.front_topic} -> {self.output_topic}"
            )

    # ---------- 单雷达：仅过滤无效值 ----------
    def _on_single(self, msg: LaserScan):
        out = LaserScan()
        out.header = msg.header
        out.angle_min = msg.angle_min
        out.angle_max = msg.angle_max
        out.angle_increment = msg.angle_increment
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = self.range_min
        out.range_max = self.range_max
        ranges = np.asarray(msg.ranges, dtype=np.float32)
        ranges = self._sanitize(ranges)
        out.ranges = ranges.tolist()
        out.intensities = msg.intensities
        self.pub.publish(out)

    def _sanitize(self, ranges: np.ndarray) -> np.ndarray:
        """无效/越界值置为 inf。"""
        invalid = (
            ~np.isfinite(ranges)
            | (ranges >= self.invalid_value)
            | (ranges < self.range_min)
            | (ranges > self.range_max)
        )
        ranges = ranges.copy()
        ranges[invalid] = float("inf")
        return ranges

    # ---------- 合并模式 ----------
    def _on_front(self, msg: LaserScan):
        self._front = msg
        self._try_merge()

    def _on_rear(self, msg: LaserScan):
        self._rear = msg
        self._try_merge()

    def _scan_to_points(self, msg: LaserScan):
        """把一帧 scan 过滤后转为 target_frame 下的 (x, y) 点集。"""
        ranges = self._sanitize(np.asarray(msg.ranges, dtype=np.float64))
        n = ranges.shape[0]
        angles = msg.angle_min + np.arange(n) * msg.angle_increment
        valid = np.isfinite(ranges)
        r = ranges[valid]
        a = angles[valid]
        xs = r * np.cos(a)
        ys = r * np.sin(a)

        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame, msg.header.frame_id, rclpy.time.Time()
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(
                f"缺少 TF {self.target_frame} <- {msg.header.frame_id}: {exc}",
                throttle_duration_sec=2.0,
            )
            return None

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = yaw_from_quat(q.x, q.y, q.z, q.w)
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        tx = cos_y * xs - sin_y * ys + t.x
        ty = sin_y * xs + cos_y * ys + t.y
        return tx, ty

    def _try_merge(self):
        if self._front is None or self._rear is None:
            return

        pts = []
        for scan in (self._front, self._rear):
            res = self._scan_to_points(scan)
            if res is not None:
                pts.append(res)
        if not pts:
            return

        xs = np.concatenate([p[0] for p in pts])
        ys = np.concatenate([p[1] for p in pts])
        r = np.hypot(xs, ys)
        a = np.arctan2(ys, xs)

        n_bins = int(round((self.out_angle_max - self.out_angle_min) / self.out_angle_increment))
        out_ranges = np.full(n_bins, np.inf, dtype=np.float64)
        idx = np.floor((a - self.out_angle_min) / self.out_angle_increment).astype(int)
        inside = (idx >= 0) & (idx < n_bins) & (r >= self.range_min) & (r <= self.range_max)
        idx = idx[inside]
        r = r[inside]
        # 同一角度 bin 取最近距离
        for i, rng in zip(idx, r):
            if rng < out_ranges[i]:
                out_ranges[i] = rng

        out = LaserScan()
        # 用较新的时间戳
        out.header.stamp = self._front.header.stamp
        out.header.frame_id = self.target_frame
        out.angle_min = self.out_angle_min
        out.angle_max = self.out_angle_max
        out.angle_increment = self.out_angle_increment
        out.time_increment = 0.0
        out.scan_time = 0.0
        out.range_min = self.range_min
        out.range_max = self.range_max
        out.ranges = out_ranges.astype(np.float32).tolist()
        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = LaserPreprocess()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
