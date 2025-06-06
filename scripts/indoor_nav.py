import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from pymavlink import mavutil
import time
import math

drone_X = 0
drone_Y = 0
drone_Z = 0
heading = 0

# Connect to the SITL simulator via TCP(Transmission Communication Protocol)
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')

# Wait until heartbeat is received
master.wait_heartbeat()
print("Heartbeat received from Ardupilot !")

def get_global_alt():
    """
    Altitude of the drone through the barometer.
    """
    # Request the GLOBAL_POSITION_INT message
    master.mav.command_long_send(
        master.target_system,                      # Target system
        master.target_component,                   # Target component
        mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE,   # MAVLink command to request a specific message
        0,                                         # Confirmation
        33,                                        # Message ID of GLOBAL_POSITION_INT
        0, 0, 0, 0, 0, 0                           # Unused parameters
    )

    # Wait for the GLOBAL_POSITION_INT message response
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)

    relative_alt = msg.relative_alt / 1000.0  # Convert to meters (above ground level)
   
    # Return the extracted data
    return relative_alt

def  get_gps_data():
        """
        Make fake GPS data for the drone.
        """
        lat,lon,alt = -35.3632621, 149.165237, 583.97 #IIT Dharwad location
        return lat , lon , alt 

 
def get_drone_location():
    """Function to get GPS location"""
    lat,lon,alt = get_gps_data()
    return lat, lon

def reset_global_origin(latitude, longitude, altitude):
    """
    Reset the global origin for the autopilot's local position calculations.
    """
    master.mav.set_gps_global_origin_send(
        master.target_system,            # Target system ID
        int(latitude * 1e7),             # Latitude in degrees * 1e7
        int(longitude * 1e7),            # Longitude in degrees * 1e7
        int(altitude * 1000)             # Altitude in millimeters
    )
    print(f"Global origin reset to: Lat={latitude}, Lon={longitude}, Alt={altitude}m")

def set_home_position(lat,lon):
    master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME,
            1,                      # set position
            0,                      # param1
            0,                      # param2
            0,                      # param3
            0,                      # param4
            lat , # Current Lat
            lon,  # CurrentLong
             0   # alt
            ) 
    
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
        [0]*21,  # Optional covariance
        0
    )