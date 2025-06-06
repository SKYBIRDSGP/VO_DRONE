#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from pymavlink import mavutil
from geometry_msgs.msg import Twist
from mavros_msgs.srv import CommandBool, SetMode
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller')
        
        # MAVLink Connection
        self.mavlink = mavutil.mavlink_connection('udpin:0.0.0.0:14550')
        self.mavlink.wait_heartbeat()
        self.get_logger().info("MAVLink Connected!")

        # MAVROS Services
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')

        # Velocity Publisher (20Hz+ required)
        self.vel_pub = self.create_publisher(Twist, '/mavros/setpoint_velocity/cmd_vel_unstamped', 10)
        
        # Wait for MAVROS services
        while not self.arm_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('MAVROS arming service not available, waiting...')
        self.get_logger().info("MAVROS Ready!")

    def set_mode(self, mode="GUIDED"):
        req = SetMode.Request()
        req.custom_mode = mode
        future = self.mode_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info(f"Mode set to {mode}")

    def arm(self):
        req = CommandBool.Request()
        req.value = True
        future = self.arm_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        self.get_logger().info("Armed!")

    def takeoff(self, altitude=5.0):  # Explicit float
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system, self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, altitude
        )
        self.get_logger().info(f"Taking off to {altitude}m...")
        time.sleep(5)

    def send_velocity(self, vx=0.0, vy=0.0, vz=0.0, duration=5.0):  # All params as floats
        cmd_vel = Twist()
        cmd_vel.linear.x = float(vx)  # Explicit float conversion
        cmd_vel.linear.y = float(vy)  # <-- Fix applied here
        cmd_vel.linear.z = float(vz)
        
        start_time = time.time()
        rate = self.create_rate(20)  # 20Hz
        
        while (time.time() - start_time) < duration and rclpy.ok():
            self.vel_pub.publish(cmd_vel)
            rate.sleep()
        self.get_logger().info(f"Velocity command finished ({duration}s)")

    def land(self):
        self.mavlink.mav.command_long_send(
            self.mavlink.target_system, self.mavlink.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0, 0, 0, 0, 0, 0, 0, 0
        )
        self.get_logger().info("Landing...")

def main():
    rclpy.init()
    controller = DroneController()
    
    # Execute mission
    controller.set_mode("GUIDED")
    controller.arm()
    controller.takeoff()
    
    # Move forward at 1.0 m/s (all values as floats)
    controller.send_velocity(vx=1.0, vy=0.0, vz=0.0, duration=5.0)
    
    controller.land()
    rclpy.shutdown()

if __name__ == '__main__':
    main()