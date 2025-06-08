#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

def main():
    rclpy.init()
    node = Node("vo_gz_subscriber")

    def callback(msg):
        node.get_logger().info(f" Received Point: x={msg.x:.3f}, y={msg.y:.3f}, z={msg.z:.3f}")

    node.create_subscription(Point, "/vo_gz_data", callback, 10)

    node.get_logger().info(" Listening to /vo_gz_data topic...")
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info(" Shutting down subscriber node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

