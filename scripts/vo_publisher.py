#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import subprocess
import re

# Regex to capture x, y, z from gz output
pattern = re.compile(
    r"position\s*{\s*.*?x:\s*([0-9eE\.\-]+)\s*y:\s*([0-9eE\.\-]+)\s*z:\s*([0-9eE\.\-]+)",
    re.DOTALL
)

class GzPosePublisher(Node):
    def __init__(self):
        super().__init__('gz_pose_publisher')
        self.publisher_ = self.create_publisher(Point, '/vo_gz_data', 10)

        self.get_logger().info("Starting to publish the Pose Data !!!")
        self.start_gz_listener()

    def start_gz_listener(self):
        cmd = ["gz", "topic", "-e", "-t", "/model/iris/pose"]
        self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, text=True)

        self.buffer = ""
        self.create_timer(0.01, self.read_pose)

    def read_pose(self):
        try:
            line = self.proc.stdout.readline()
            if not line:
                return
            self.buffer += line
            if "}" in line:
                match = pattern.search(self.buffer)
                if match:
                    x, y, z = map(float, match.groups())
                    msg = Point()
                    msg.x = x
                    msg.y = y
                    msg.z = z
                    self.publisher_.publish(msg)
                    self.get_logger().info(f"Published ENU Point: x={x:.3f}, y={y:.3f}, z={z:.3f}")
                self.buffer = ""
        except Exception as e:
            self.get_logger().error(f"Error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = GzPosePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Terminating !!!")
    finally:
        node.proc.terminate()
        node.proc.wait()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

