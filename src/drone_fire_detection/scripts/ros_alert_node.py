#!/usr/bin/env python3

"""ROS 2 alert node that logs detections and publishes RViz markers."""

from __future__ import annotations

import csv
import re
from datetime import datetime
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class FireAlertNode(Node):
    """Log fire detections to CSV and visualize them in RViz."""

    _COORDINATE_PATTERN = re.compile(
        r"x=(?P<x>-?\d+(?:\.\d+)?),\s*y=(?P<y>-?\d+(?:\.\d+)?)"
    )

    def __init__(self) -> None:
        super().__init__("fire_alert_node")

        self.declare_parameter("fire_detected_topic", "/fire_detected")
        self.declare_parameter("marker_topic", "/fire_markers")
        self.declare_parameter("marker_frame_id", "map")
        self.declare_parameter("marker_scale_m", 0.5)
        self.declare_parameter("csv_log_path", "fire_log.csv")

        self._marker_id = 0
        self._csv_file = open(
            str(self.get_parameter("csv_log_path").value), mode="w", newline=""
        )
        self._writer = csv.writer(self._csv_file)
        self._writer.writerow(["Time", "X", "Y", "Confidence"])

        self._subscription = self.create_subscription(
            String,
            str(self.get_parameter("fire_detected_topic").value),
            self.listener_callback,
            10,
        )
        self._marker_pub = self.create_publisher(
            Marker,
            str(self.get_parameter("marker_topic").value),
            10,
        )

        self.get_logger().info("Fire alert node started.")

    def listener_callback(self, msg: String) -> None:
        coordinates = self._parse_coordinates(msg.data)
        if coordinates is None:
            self.get_logger().warning(
                f"Received fire detection message with unparseable coordinates: {msg.data}"
            )
            return

        x, y = coordinates
        timestamp = datetime.now().strftime("%H:%M:%S")

        self.get_logger().info(f"FIRE DETECTED: x={x:.1f}, y={y:.1f}")

        self._writer.writerow([timestamp, x, y, 1.0])
        self._csv_file.flush()

        marker = Marker()
        marker.header.frame_id = str(self.get_parameter("marker_frame_id").value)
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "fire_detections"
        marker.id = self._marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5
        marker.pose.orientation.w = 1.0

        scale = float(self.get_parameter("marker_scale_m").value)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = scale

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.color.a = 1.0

        self._marker_pub.publish(marker)
        self._marker_id += 1

    @classmethod
    def _parse_coordinates(cls, message: str) -> Optional[Tuple[float, float]]:
        match = cls._COORDINATE_PATTERN.search(message)
        if match is None:
            return None

        return float(match.group("x")), float(match.group("y"))

    def destroy_node(self) -> bool:
        """Close the CSV file before shutting down."""
        self._csv_file.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FireAlertNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down fire alert node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
