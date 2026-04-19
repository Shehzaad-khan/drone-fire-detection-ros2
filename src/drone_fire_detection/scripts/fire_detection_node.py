#!/usr/bin/env python3

"""ROS 2 node for detecting fire-like regions in a drone camera stream."""

from __future__ import annotations

from typing import List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge, CvBridgeError
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String


class FireDetectionNode(Node):
    """Subscribe to a camera feed, detect fire-like regions, and publish alerts."""

    def __init__(self) -> None:
        super().__init__("fire_detection_node")

        self.declare_parameter("camera_topic", "/drone/camera")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("fire_detected_topic", "/fire_detected")
        self.declare_parameter("min_contour_area", 500.0)
        self.declare_parameter("window_name", "Fire Detection")
        self.declare_parameter("display_window", False)

        self._bridge = CvBridge()
        self._window_name = str(self.get_parameter("window_name").value)
        self._display_window = bool(self.get_parameter("display_window").value)
        self._min_contour_area = float(self.get_parameter("min_contour_area").value)

        # Current drone world position, updated via odometry.
        self._world_x: float = 0.0
        self._world_y: float = 0.0

        # Red wraps in HSV, so two ranges are required to cover both ends.
        self._red_ranges: List[Tuple[np.ndarray, np.ndarray]] = [
            (np.array([0, 120, 120], dtype=np.uint8), np.array([10, 255, 255], dtype=np.uint8)),
            (np.array([170, 120, 120], dtype=np.uint8), np.array([180, 255, 255], dtype=np.uint8)),
        ]
        self._orange_range: Tuple[np.ndarray, np.ndarray] = (
            np.array([11, 120, 120], dtype=np.uint8),
            np.array([25, 255, 255], dtype=np.uint8),
        )

        self._odom_sub = self.create_subscription(
            Odometry,
            str(self.get_parameter("odom_topic").value),
            self._odom_callback,
            10,
        )
        self._image_sub = self.create_subscription(
            Image,
            str(self.get_parameter("camera_topic").value),
            self.image_callback,
            10,
        )
        self._fire_pub = self.create_publisher(
            String,
            str(self.get_parameter("fire_detected_topic").value),
            10,
        )

        if self._display_window:
            cv2.namedWindow(self._window_name, cv2.WINDOW_NORMAL)

        self.get_logger().info("Fire detection node started.")

    def _odom_callback(self, msg: Odometry) -> None:
        """Cache the drone's world position for use when publishing fire alerts."""
        self._world_x = msg.pose.pose.position.x
        self._world_y = msg.pose.pose.position.y

    def image_callback(self, msg: Image) -> None:
        """Process each incoming image, annotate detections, and publish results."""
        frame = self._convert_image(msg)
        if frame is None:
            return

        processed_frame, fire_center = self._detect_fire(frame)

        if fire_center is not None:
            # Publish world coordinates (drone position) so downstream nodes can
            # place accurate RViz markers and log meaningful GPS-like positions.
            alert_msg = String()
            alert_msg.data = (
                f"Fire detected at x={self._world_x:.2f}, y={self._world_y:.2f}"
            )
            self._fire_pub.publish(alert_msg)

        if self._display_window:
            cv2.imshow(self._window_name, processed_frame)
            cv2.waitKey(1)

    def _convert_image(self, msg: Image) -> Optional[np.ndarray]:
        """Convert a ROS image to an OpenCV BGR frame."""
        try:
            return self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            self.get_logger().error(f"Failed to convert image with cv_bridge: {exc}")
            return None

    def _detect_fire(self, frame: np.ndarray) -> Tuple[np.ndarray, Optional[Tuple[int, int]]]:
        """Detect fire-like regions using HSV thresholding and contour filtering."""
        annotated_frame = frame.copy()
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Build individual masks for red and orange, then combine them into a
        # single binary mask used for contour extraction.
        combined_mask = np.zeros(hsv_frame.shape[:2], dtype=np.uint8)
        for lower, upper in self._red_ranges:
            combined_mask = cv2.bitwise_or(combined_mask, cv2.inRange(hsv_frame, lower, upper))

        orange_mask = cv2.inRange(hsv_frame, self._orange_range[0], self._orange_range[1])
        combined_mask = cv2.bitwise_or(combined_mask, orange_mask)

        # Clean up small noise before contour extraction to make detections more stable.
        kernel = np.ones((5, 5), dtype=np.uint8)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        combined_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_DILATE, kernel)

        contours, _ = cv2.findContours(
            combined_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        largest_center: Optional[Tuple[int, int]] = None
        largest_area = 0.0

        # Keep only meaningful detections, annotate them on the frame, and track
        # the largest region for alert publication.
        for contour in contours:
            area = cv2.contourArea(contour)
            if area < self._min_contour_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + (w // 2)
            center_y = y + (h // 2)

            cv2.rectangle(annotated_frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            if area > largest_area:
                largest_area = area
                largest_center = (center_x, center_y)

        # Overlay status text when at least one valid fire region has been found.
        if largest_center is not None:
            cv2.putText(
                annotated_frame,
                "FIRE DETECTED",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1.0,
                (0, 0, 255),
                3,
                cv2.LINE_AA,
            )

        return annotated_frame, largest_center

    def destroy_node(self) -> None:
        if self._display_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args: Optional[List[str]] = None) -> None:
    rclpy.init(args=args)
    node = FireDetectionNode()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        node.get_logger().info("Shutting down fire detection node.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
