#!/usr/bin/env python3
"""cmd_vel 中继节点。

Nav2 输出标准 /cmd_vel（geometry_msgs/Twist），底盘 SDK 接收的是
/sr_amr_control/remote_control_cmd_vel，且需要先调用 remote_control_enabled
服务开启远程控制。本节点负责：

1. 启动时调用 /sr_amr_control/remote_control_enabled 开启远程控制；
   可选调用 remote_control_oba_enabled 开启/关闭远程控制避障。
2. 订阅 /cmd_vel，转发到 /sr_amr_control/remote_control_cmd_vel。
3. 退出时关闭远程控制（可配置）。

注意：全向底盘可下发 linear.x / linear.y / angular.z；若底盘实际不支持横移，
linear.y 会被忽略。
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__("cmd_vel_relay")

        self.declare_parameter("in_topic", "/cmd_vel")
        self.declare_parameter("out_topic", "/sr_amr_control/remote_control_cmd_vel")
        self.declare_parameter("enable_service", "/sr_amr_control/remote_control_enabled")
        self.declare_parameter("oba_service", "/sr_amr_control/remote_control_oba_enabled")
        self.declare_parameter("enable_on_start", True)
        self.declare_parameter("oba_on_start", True)
        self.declare_parameter("disable_on_exit", True)

        self.in_topic = self.get_parameter("in_topic").value
        self.out_topic = self.get_parameter("out_topic").value
        self.enable_service = self.get_parameter("enable_service").value
        self.oba_service = self.get_parameter("oba_service").value
        self.enable_on_start = self.get_parameter("enable_on_start").value
        self.oba_on_start = self.get_parameter("oba_on_start").value
        self.disable_on_exit = self.get_parameter("disable_on_exit").value

        self.pub = self.create_publisher(Twist, self.out_topic, 10)
        self.create_subscription(Twist, self.in_topic, self._on_cmd, 10)

        self._enable_cli = self.create_client(SetBool, self.enable_service)
        self._oba_cli = self.create_client(SetBool, self.oba_service)

        if self.enable_on_start:
            self._call_setbool(self._enable_cli, self.enable_service, True)
        if self.oba_on_start:
            self._call_setbool(self._oba_cli, self.oba_service, True)

        self.get_logger().info(f"cmd_vel 中继：{self.in_topic} -> {self.out_topic}")

    def _call_setbool(self, client, name, value):
        if not client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(f"服务 {name} 不可用，跳过（请确认底盘节点已启动）")
            return
        req = SetBool.Request()
        req.data = value
        future = client.call_async(req)
        future.add_done_callback(
            lambda f, n=name, v=value: self._log_result(f, n, v)
        )

    def _log_result(self, future, name, value):
        try:
            resp = future.result()
            self.get_logger().info(f"{name} set {value} -> success={resp.success} {resp.message}")
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f"{name} 调用失败: {exc}")

    def _on_cmd(self, msg: Twist):
        self.pub.publish(msg)

    def destroy_node(self):
        if self.disable_on_exit:
            try:
                self._call_setbool(self._enable_cli, self.enable_service, False)
            except Exception:  # noqa: BLE001
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
