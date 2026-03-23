#!/usr/bin/env python3
"""
Stereo calibration image capture script.
Subscribes to left and right camera topics and saves synchronized image pairs.
Press Enter to capture, 'q' + Enter to quit.
"""

import os
import threading
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

SAVE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "calibration_images")
LEFT_TOPIC = "/left/image_raw"
RIGHT_TOPIC = "/right/image_raw"
SYNC_THRESHOLD = 0.05  # seconds


class StereoCaptureNode(Node):
    def __init__(self):
        super().__init__("stereo_capture")
        self.bridge = CvBridge()
        self.latest_left = None
        self.latest_right = None
        self.latest_left_time = None
        self.latest_right_time = None
        self.lock = threading.Lock()
        self.capture_count = 0

        os.makedirs(os.path.join(SAVE_DIR, "left"), exist_ok=True)
        os.makedirs(os.path.join(SAVE_DIR, "right"), exist_ok=True)

        self.create_subscription(Image, LEFT_TOPIC, self.left_callback, 10)
        self.create_subscription(Image, RIGHT_TOPIC, self.right_callback, 10)

        self.get_logger().info(f"Subscribing to {LEFT_TOPIC} and {RIGHT_TOPIC}")
        self.get_logger().info(f"Saving images to {SAVE_DIR}")

    def left_callback(self, msg):
        with self.lock:
            self.latest_left = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_left_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def right_callback(self, msg):
        with self.lock:
            self.latest_right = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.latest_right_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def capture(self):
        with self.lock:
            if self.latest_left is None or self.latest_right is None:
                print("ERROR: No images received yet - are the camera nodes running?")
                return False

            time_diff = abs(self.latest_left_time - self.latest_right_time)
            if time_diff > SYNC_THRESHOLD:
                print(f"WARNING: Images are {time_diff*1000:.1f}ms apart (max {SYNC_THRESHOLD*1000:.0f}ms) - skipping, try again")
                return False

            left_path = os.path.join(SAVE_DIR, "left", f"left_{self.capture_count:04d}.png")
            right_path = os.path.join(SAVE_DIR, "right", f"right_{self.capture_count:04d}.png")
            cv2.imwrite(left_path, self.latest_left)
            cv2.imwrite(right_path, self.latest_right)
            self.capture_count += 1
            print(f"  Saved pair {self.capture_count:02d}  (sync delta: {time_diff*1000:.1f}ms)")
            return True


def main():
    rclpy.init()
    node = StereoCaptureNode()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    print("\n=== Stereo Calibration Image Capture ===")
    print(f"Topics : {LEFT_TOPIC}  |  {RIGHT_TOPIC}")
    print(f"Output : {SAVE_DIR}")
    print()
    print("Instructions:")
    print("  - Hold the checkerboard still in a new position")
    print("  - Press Enter to capture that position")
    print("  - Aim for ~20 captures covering:")
    print("      * Different distances (near/far)")
    print("      * Tilted left/right and up/down")
    print("      * All corners of the frame")
    print("  - Press 'q' + Enter to finish\n")

    while True:
        try:
            user_input = input("Press Enter to capture (q to quit): ")
            if user_input.strip().lower() == "q":
                break
            node.capture()
        except (KeyboardInterrupt, EOFError):
            break

    print(f"\nCapture complete: {node.capture_count} pairs saved to {SAVE_DIR}")
    print("Run calibrate_stereo.py next.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
