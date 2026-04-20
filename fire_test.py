#!/usr/bin/env python3
"""
End-to-end fire detection test.
Publishes a synthetic orange fire image → verifies fire_detection_node detects it
→ verifies ros_alert_node writes to CSV.
"""
import time, csv, os, sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from cv_bridge import CvBridge

CSV_PATH = "/home/pes2ug23cs349/fire_log.csv"

class FireTestNode(Node):
    def __init__(self):
        super().__init__("fire_test_node")
        self.bridge = CvBridge()
        self.detected = []

        # Publish fake camera frames
        self.img_pub = self.create_publisher(Image, "/drone/camera/image_raw", 10)
        # Publish fake odom so flight node has a pose
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        # Listen for detections
        self.create_subscription(String, "/fire_detected", self._on_detect, 10)

        self.create_timer(0.1, self._publish_fire_image)   # 10 fps
        self.create_timer(0.05, self._publish_odom)        # 20 fps
        self.get_logger().info("Test node started — publishing orange fire image...")

    def _publish_fire_image(self):
        # Create a 640x480 image filled with bright orange (HSV ~15°, S=255, V=255)
        # In BGR: orange = (0, 165, 255)
        img = np.full((480, 640, 3), (0, 140, 255), dtype=np.uint8)
        # Add a bright red patch in the center to ensure HSV hit
        img[180:300, 260:380] = (0, 0, 255)   # pure red patch
        msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.img_pub.publish(msg)

    def _publish_odom(self):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = 2.0
        msg.pose.pose.position.y = 2.5
        msg.pose.pose.position.z = 2.5
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

    def _on_detect(self, msg):
        self.detected.append(msg.data)
        self.get_logger().info(f"[PROOF] /fire_detected received: '{msg.data}'")


def main():
    rclpy.init()

    # Clean old CSV
    if os.path.exists(CSV_PATH):
        os.remove(CSV_PATH)
        print(f"Cleared old {CSV_PATH}")

    node = FireTestNode()
    print("\nPublishing fire image for 8 seconds...\n")

    start = time.time()
    while time.time() - start < 8.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    print("\n" + "="*50)
    print("RESULTS")
    print("="*50)

    if node.detected:
        print(f"\n✅ /fire_detected received {len(node.detected)} message(s):")
        for d in node.detected:
            print(f"   → {d}")
    else:
        print("\n❌ No /fire_detected messages received")

    print("\n--- CSV CONTENTS ---")
    if os.path.exists(CSV_PATH):
        with open(CSV_PATH) as f:
            content = f.read()
        print(content)
        rows = list(csv.reader(content.strip().splitlines()))
        data_rows = [r for r in rows if r[0] != "Time"]
        if data_rows:
            print(f"✅ CSV has {len(data_rows)} fire detection(s) logged")
        else:
            print("❌ CSV exists but has no data rows")
    else:
        print("❌ CSV file not created")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
