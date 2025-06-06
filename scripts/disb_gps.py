import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
import time
import math
import tf_transformations

class VOPublisher(Node):
    def __init__(self):
        super().__init__('vo_publisher')

        # MAVLink connection
        self.master = mavutil.mavlink_connection('udp:127.0.0.1:14551')
        self.master.wait_heartbeat()
        self.get_logger().info('Connected to MAVLink')

        # Set EKF origin/home position (DO THIS EARLY!)
        self.set_home_position()
        self.get_logger().info('Sent Home Position.')

        # Start VO publisher
        self.subscription = self.create_subscription(
            Odometry,
            '/odometry',
            self.odom_callback,
            10
        )

        # Optional: Set to LOITER or GUIDED

        # Optional: Arm the vehicle (if needed)
        # self.arm_drone()

        # Send a few fake VO msgs to wake EKF before main odom_callback runs
        self.bootstrap_vo()

    def bootstrap_vo(self):
        self.get_logger().info("Sending dummy VO data for EKF init...")
        for _ in range(20):
            now = int(time.time() * 1e6)
            self.master.mav.vision_position_estimate_send(
                now,
                0.0, 0.0, -0.1,   # z < 0 to simulate slight altitude
                0.0, 0.0, 0.0,
                [0]*21, 0
            )
            time.sleep(0.1)
        self.get_logger().info("VO bootstrap done.")

    def odom_callback(self, msg):
        now = int(time.time() * 1e6)
        roll, pitch, yaw = self.euler_from_quaternion(msg.pose.pose.orientation)

        self.master.mav.vision_position_estimate_send(
            now,
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
            roll,
            pitch,
            yaw,
            [0]*21,
            0
        )
        self.get_logger().info(f"VO sent: x={msg.pose.pose.position.x:.2f}, y={msg.pose.pose.position.y:.2f}")

    def set_home_position(self):
        lat, lon, alt = -35.3632621, 149.165237, 583.97  # IIT Dharwad or fake location
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1,  # Use current location (relative=1)
            0, 0, 0, 0,
            lat, lon, alt
        )
        self.get_logger().info("Sent the Home position !")

    def euler_from_quaternion(self, q):
        orientation_list = [q.x, q.y, q.z, q.w]
        return tf_transformations.euler_from_quaternion(orientation_list)

def main(args=None):
    rclpy.init(args=args)
    node = VOPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
