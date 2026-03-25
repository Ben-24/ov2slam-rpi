#!/usr/bin/env python3
"""
Diagnostic script: save one frame from each camera and print brightness stats.
Run with both cameras publishing:
    python3 calibration/check_cameras.py
Saves /tmp/cam0.png and /tmp/cam1.png for visual inspection.
"""
import sys
import os
import numpy as np

def main():
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
    except ImportError:
        print("ERROR: rclpy not found. Source ROS2 first:")
        print("  source /opt/ros/humble/setup.bash")
        sys.exit(1)

    try:
        import cv2
    except ImportError:
        print("ERROR: cv2 not found. Install: pip3 install opencv-python")
        sys.exit(1)

    class CameraChecker(Node):
        def __init__(self):
            super().__init__('camera_checker')
            self.frames = {}
            for topic in ['/cam0/image_raw', '/cam1/image_raw']:
                self.create_subscription(Image, topic, lambda msg, t=topic: self.cb(msg, t), 1)
            self.get_logger().info("Waiting for frames from /cam0/image_raw and /cam1/image_raw ...")

        def cb(self, msg, topic):
            if topic not in self.frames:
                arr = np.frombuffer(msg.data, dtype=np.uint8)
                total_pixels = msg.height * msg.width
                channels = len(arr) // total_pixels
                if channels == 1:
                    arr = arr.reshape(msg.height, msg.width)
                else:
                    arr = arr.reshape(msg.height, msg.width, channels)
                # For stats, use only RGB channels (ignore alpha if present)
                arr_rgb = arr[:, :, :3] if channels == 4 else arr
                self.frames[topic] = arr_rgb
                print(f"\n{topic}:")
                print(f"  Size:     {msg.width} x {msg.height}")
                print(f"  Encoding: {msg.encoding}")
                print(f"  Mean brightness: {arr_rgb.mean():.1f}")
                print(f"  Std dev:         {arr_rgb.std():.1f}")
                print(f"  Min/Max:         {arr_rgb.min()} / {arr_rgb.max()}")
                # Drop alpha channel if present so PNG renders correctly
                if channels == 4:
                    arr_save = arr[:, :, :3]  # keep BGR only
                else:
                    arr_save = arr
                out = f"/tmp/{topic.replace('/', '_').strip('_')}.png"
                cv2.imwrite(out, arr_save)
                print(f"  Saved to: {out}")

    rclpy.init()
    node = CameraChecker()
    print("Spinning — Ctrl+C to stop once both cameras are captured\n")
    try:
        while len(node.frames) < 2:
            rclpy.spin_once(node, timeout_sec=1.0)
            if len(node.frames) == 0:
                print("  (no frames yet — are both cameras running?)")
    except KeyboardInterrupt:
        pass

    if len(node.frames) < 2:
        missing = {'/cam0/image_raw', '/cam1/image_raw'} - set(node.frames.keys())
        print(f"\nWARNING: No frames received from: {missing}")
        print("Check that both camera nodes are running.")
    else:
        cams = sorted(node.frames.keys())
        b0 = node.frames[cams[0]].mean()
        b1 = node.frames[cams[1]].mean()
        ratio = max(b0, b1) / max(min(b0, b1), 1.0)
        print(f"\nBrightness ratio: {ratio:.2f}x  (ideal = 1.0, > 2.0 = likely problem)")
        if ratio > 2.0:
            print("WARNING: Large brightness difference — cameras may have independent auto-exposure.")
            print("  Fix: ensure both camera_ros nodes use the same gain/exposure settings.")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
