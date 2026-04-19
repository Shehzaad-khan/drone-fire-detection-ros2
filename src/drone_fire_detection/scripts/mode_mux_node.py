#!/usr/bin/env python3

"""Mode multiplexer — routes cmd_vel_auto or cmd_vel_teleop to cmd_vel.

Switch mode at runtime:
  ros2 topic pub --once /set_mode std_msgs/msg/String "{data: 'manual'}"
  ros2 topic pub --once /set_mode std_msgs/msg/String "{data: 'autonomous'}"
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String

MODES = ("autonomous", "manual")


class ModeMuxNode(Node):
    def __init__(self) -> None:
        super().__init__("mode_mux")

        self._mode: str = "autonomous"

        self._pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self._clear_pub = self.create_publisher(String, "/clear_fire", 10)

        self.create_subscription(String, "/set_mode", self._set_mode_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_auto", self._auto_cb, 10)
        self.create_subscription(Twist, "/cmd_vel_teleop", self._teleop_cb, 10)

        self.get_logger().info("Mode mux ready — current mode: autonomous")
        self.get_logger().info(
            "  Switch to manual:     ros2 topic pub --once /set_mode "
            "std_msgs/msg/String \"{data: 'manual'}\""
        )
        self.get_logger().info(
            "  Switch to autonomous: ros2 topic pub --once /set_mode "
            "std_msgs/msg/String \"{data: 'autonomous'}\""
        )

    def _set_mode_cb(self, msg: String) -> None:
        if msg.data not in MODES:
            self.get_logger().warning(f"Unknown mode '{msg.data}'. Use: {MODES}")
            return
        if msg.data != self._mode:
            self._mode = msg.data
            self.get_logger().info(f"Mode switched → {self._mode}")
            if self._mode == "autonomous":
                clear = String()
                clear.data = "clear"
                self._clear_pub.publish(clear)

    def _auto_cb(self, msg: Twist) -> None:
        if self._mode == "autonomous":
            self._pub.publish(msg)

    def _teleop_cb(self, msg: Twist) -> None:
        if self._mode == "manual":
            self._pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ModeMuxNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
