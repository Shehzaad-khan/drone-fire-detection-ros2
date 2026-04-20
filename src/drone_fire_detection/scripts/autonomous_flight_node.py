#!/usr/bin/env python3

import math
from enum import Enum
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Empty, String


class MissionState(Enum):
    SEARCH = "search"
    LAND = "land"
    DONE = "done"


class AutonomousFlightNode(Node):
    def __init__(self) -> None:
        super().__init__("autonomous_flight_node")

        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("takeoff_topic", "/drone/takeoff")
        self.declare_parameter("land_topic", "/drone/land")
        self.declare_parameter("fire_detected_topic", "/fire_detected")
        self.declare_parameter("use_takeoff_land_topics", False)
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("altitude_target_m", 2.5)
        self.declare_parameter("altitude_tolerance_m", 2.0)
        self.declare_parameter("xy_tolerance_m", 0.5)
        self.declare_parameter("yaw_tolerance_rad", 0.3)
        self.declare_parameter("max_lin_speed", 0.8)
        self.declare_parameter("max_z_speed", 0.5)
        self.declare_parameter("max_yaw_rate", 0.6)
        self.declare_parameter("kp_xy", 0.6)
        self.declare_parameter("kp_z", 0.7)
        self.declare_parameter("kp_yaw", 1.0)
        self.declare_parameter("search_x_min", -6.0)
        self.declare_parameter("search_x_max", 6.0)
        self.declare_parameter("search_y_min", -6.0)
        self.declare_parameter("search_y_max", 6.0)
        self.declare_parameter("search_row_spacing", 2.0)
        self.declare_parameter("start_yaw_rad", 0.0)
        self.declare_parameter("fire_hover_duration_s", 10.0)

        self.cmd_vel_pub = self.create_publisher(
            Twist, self.get_parameter("cmd_vel_topic").value, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, self.get_parameter("odom_topic").value, self.odom_callback, 10
        )
        self.fire_detected_sub = self.create_subscription(
            String,
            self.get_parameter("fire_detected_topic").value,
            self.fire_detected_callback,
            10,
        )
        self.create_subscription(String, "/clear_fire", self._clear_fire_cb, 10)

        self.xy_tol = float(self.get_parameter("xy_tolerance_m").value)
        self.yaw_tol = float(self.get_parameter("yaw_tolerance_rad").value)
        self.max_lin_speed = float(self.get_parameter("max_lin_speed").value)
        self.max_yaw_rate = float(self.get_parameter("max_yaw_rate").value)
        self.kp_xy = float(self.get_parameter("kp_xy").value)
        self.kp_yaw = float(self.get_parameter("kp_yaw").value)
        self.start_yaw = float(self.get_parameter("start_yaw_rad").value)
        self._hover_duration = float(self.get_parameter("fire_hover_duration_s").value)

        self.current_pose = None
        self.current_yaw = 0.0
        # PlanarMove is 2D — drone spawns at correct altitude, go straight to SEARCH
        self.state = MissionState.SEARCH
        self.fire_detected = False
        self.hover_target: Optional[Tuple[float, float, float]] = None
        self._fire_hover_end: Optional[float] = None

        self.search_waypoints = self.build_lawnmower_waypoints()
        self.current_wp_idx = 0

        rate_hz = float(self.get_parameter("control_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate_hz, self.control_loop)

        self.get_logger().info(
            f"Autonomous flight started — {len(self.search_waypoints)} waypoints."
        )

    def build_lawnmower_waypoints(self) -> List[Tuple[float, float, float]]:
        x_min = float(self.get_parameter("search_x_min").value)
        x_max = float(self.get_parameter("search_x_max").value)
        y_min = float(self.get_parameter("search_y_min").value)
        y_max = float(self.get_parameter("search_y_max").value)
        row_spacing = float(self.get_parameter("search_row_spacing").value)
        if row_spacing <= 0.0:
            row_spacing = 1.0

        y_rows: List[float] = []
        y = y_min
        while y <= y_max + 1e-6:
            y_rows.append(y)
            y += row_spacing

        waypoints: List[Tuple[float, float, float]] = []
        left_to_right = True
        for y_row in y_rows:
            if left_to_right:
                waypoints.append((x_min, y_row, self.start_yaw))
                waypoints.append((x_max, y_row, self.start_yaw))
            else:
                waypoints.append((x_max, y_row, self.start_yaw))
                waypoints.append((x_min, y_row, self.start_yaw))
            left_to_right = not left_to_right
        return waypoints

    def odom_callback(self, msg: Odometry) -> None:
        self.current_pose = msg.pose.pose
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.current_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self) -> None:
        if self.current_pose is None:
            return

        if self.fire_detected:
            self.handle_hover()
            return

        if self.state == MissionState.SEARCH:
            self.handle_search()
        elif self.state == MissionState.LAND:
            self.handle_land()
        else:
            self.publish_stop()

    def handle_search(self) -> None:
        if self.current_wp_idx >= len(self.search_waypoints):
            self.publish_stop()
            self.state = MissionState.LAND
            self.get_logger().info("Search complete. Landing.")
            return

        tx, ty, tyaw = self.search_waypoints[self.current_wp_idx]
        cx = self.current_pose.position.x
        cy = self.current_pose.position.y
        ex = tx - cx
        ey = ty - cy
        yaw_error = self.normalize_angle(tyaw - self.current_yaw)

        # Only XY + yaw check — Z ignored (PlanarMove is 2D)
        if math.hypot(ex, ey) < self.xy_tol and abs(yaw_error) < self.yaw_tol:
            self.current_wp_idx += 1
            self.get_logger().info(
                f"Waypoint {self.current_wp_idx}/{len(self.search_waypoints)} reached."
            )
            self.publish_stop()
            return

        vx = self.clamp(self.kp_xy * ex, self.max_lin_speed)
        vy = self.clamp(self.kp_xy * ey, self.max_lin_speed)
        wz = self.clamp(self.kp_yaw * yaw_error, self.max_yaw_rate)
        self.publish_cmd(vx, vy, 0.0, wz)

    def handle_land(self) -> None:
        # PlanarMove can't descend — just declare mission done
        self.publish_stop()
        self.state = MissionState.DONE
        self.get_logger().info("Mission complete.")

    def handle_hover(self) -> None:
        if self.hover_target is None:
            self.publish_stop()
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        if self._fire_hover_end is not None and now >= self._fire_hover_end:
            self.get_logger().info("Hover complete — resuming search.")
            self.fire_detected = False
            self.hover_target = None
            self._fire_hover_end = None
            return

        tx, ty, tyaw = self.hover_target
        ex = tx - self.current_pose.position.x
        ey = ty - self.current_pose.position.y
        yaw_error = self.normalize_angle(tyaw - self.current_yaw)

        vx = self.clamp(self.kp_xy * ex, self.max_lin_speed)
        vy = self.clamp(self.kp_xy * ey, self.max_lin_speed)
        wz = self.clamp(self.kp_yaw * yaw_error, self.max_yaw_rate)
        self.publish_cmd(vx, vy, 0.0, wz)

    def fire_detected_callback(self, msg: String) -> None:
        if self.current_pose is None or self.fire_detected:
            return
        self.fire_detected = True
        self.hover_target = (
            self.current_pose.position.x,
            self.current_pose.position.y,
            self.current_yaw,
        )
        self._fire_hover_end = (
            self.get_clock().now().nanoseconds * 1e-9 + self._hover_duration
        )
        self.get_logger().info(
            f"Fire detected — hovering {self._hover_duration:.0f}s then resuming. "
            f"{msg.data}"
        )

    def _clear_fire_cb(self, msg: String) -> None:
        if self.fire_detected:
            self.get_logger().info("Fire flag cleared — resuming search.")
        self.fire_detected = False
        self.hover_target = None
        self._fire_hover_end = None

    def publish_cmd(self, vx: float, vy: float, vz: float, wz: float) -> None:
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.linear.z = vz
        msg.angular.z = wz
        self.cmd_vel_pub.publish(msg)

    def publish_stop(self) -> None:
        self.publish_cmd(0.0, 0.0, 0.0, 0.0)

    @staticmethod
    def clamp(value: float, limit: float) -> float:
        return max(-limit, min(limit, value))

    @staticmethod
    def normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AutonomousFlightNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        node.get_logger().info("Shutting down autonomous flight node.")
    finally:
        node.publish_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
