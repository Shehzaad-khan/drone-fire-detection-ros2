import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from visualization_msgs.msg import Marker
import csv
from datetime import datetime

class FireAlertNode(Node):

    def __init__(self):
        super().__init__('fire_alert_node')

        self.subscription = self.create_subscription(
            String,
            '/fire_detected',
            self.listener_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/fire_markers',
            10
        )

        self.file = open("fire_log.csv", mode="w", newline="")
        self.writer = csv.writer(self.file)
        self.writer.writerow(["Time", "X", "Y", "Confidence"])

    def listener_callback(self, msg):
        timestamp = datetime.now().strftime("%H:%M:%S")

        self.get_logger().info(f'🔥 FIRE DETECTED: {msg.data}')

        # Simulated parsing (since msg is string)
        x, y = 5.0, 3.0

        self.writer.writerow([timestamp, x, y, 0.9])

        marker = Marker()
        marker.header.frame_id = "map"
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.5

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5

        marker.color.r = 1.0
        marker.color.g = 0.2
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FireAlertNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()