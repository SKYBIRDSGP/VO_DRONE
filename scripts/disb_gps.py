#!/usr/bin/env python3
import time
import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Point
from pymavlink import mavutil

class VisionPoseBridge(Node):
    def __init__(self):
        super().__init__('vision_pose_bridge')

        # Connect to MAVLink
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
        self.master.wait_heartbeat()
        self.get_logger().info("Heartbeat received from ArduPilot")

        # Setup initial GPS and EKF data
        self.setup_ekf_and_home()

        # Latest pose from Gazebo
        self.latest_vo = Point()

        # ROS 2 Subscriber to Gazebo VO data
        self.subscription = self.create_subscription(Point, '/vo_gz_data', self.vo_callback, 0)
        self.get_logger().info("Subscribed to /vo_gz_data")

        # Frequency of Publishing the VisualOdom messages to the Ardupilot --> 10Hz
        self.timer = self.create_timer(0.1, self.send_vision_position)

    def setup_ekf_and_home(self):
        lat, lon, alt = self.get_gps_data()
        self.set_home_position(lat, lon)
        self.reset_global_origin(lat, lon, alt)
        self.send_initial_vo_burst()
        self.set_mode("GUIDED")
        self.get_logger().info("ArduPilot setup complete")

    def get_gps_data(self):
        # Fake GPS data for simulation
        return -35.3632621, 149.165237, 583.97

    def set_home_position(self, lat, lon):
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1, 0, 0, 0, 0,
            lat, lon, 0
        )
        self.get_logger().info("Home position set")

    def reset_global_origin(self, lat, lon, alt):
        self.master.mav.set_gps_global_origin_send(
            self.master.target_system,
            int(lat * 1e7),
            int(lon * 1e7),
            int(alt * 1000)
        )
        self.get_logger().info(f"Global origin set: lat={lat}, lon={lon}, alt={alt}")

    def send_initial_vo_burst(self):
        for _ in range(15):
            self.send_vp_to_nav(0, 0, 0)
            time.sleep(0.1)
        self.get_logger().info("Sent VO burst for EKF init")

    def set_mode(self, mode):
        mode_id = self.master.mode_mapping().get(mode)
        if mode_id is None:
            self.get_logger().error(f"Unknown mode: {mode}")
            return
        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id
        )
        ack = self.master.recv_match(type='HEARTBEAT', blocking=True)
        if ack.custom_mode == mode_id:
            self.get_logger().info(f" Mode set to {mode}")
        else:
            self.get_logger().warn(f" Mode set to {mode} may have failed")

    def vo_callback(self, msg: Point):
        self.latest_vo = msg

    def send_vp_to_nav(self, x, y, z):
        self.master.mav.global_vision_position_estimate_send(
            int(time.time() * 1e6),
            y, x, -z,
            0.0, 0.0, 0.0,
            [10, 0, 0, 0, 0, 0,
             10, 0, 0, 0, 0,
             1, 0, 0, 0,
             1, 0, 0,
             1, 0, 1]
        )
        self.get_logger().info(f" Sent VO: x={x:.2f}, y={y:.2f}, z={z:.2f}")

    def send_vision_position(self):
        self.send_vp_to_nav(
            self.latest_vo.x,
            self.latest_vo.y,
            self.latest_vo.z
        )

def main(args=None):
    rclpy.init(args=args)
    node = VisionPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()