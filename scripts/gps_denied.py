# from aruco_tracker import PrecisionLandingNode
import time
import csv
from pymavlink import mavutil
# import rospy
# from gazebo_msgs.msg import ModelStates
# import tf.transformations as tf_trans  # Usually provided by tf in ROS
import os

# aruco_tracker = PrecisionLandingNode()

def get_unique_filename(base_name="logs/log", extension=".csv"):
    i = 1
    while True:
        filename = f"{base_name}{i}{extension}"
        if not os.path.exists(filename):
            return filename
        i += 1


# Get a new filename
csv_file = get_unique_filename()
file = open(csv_file, 'w', newline='')
writer = csv.writer(file)
writer.writerow(["Drone_X","Drone_Y","Drone_Z","Aruco_X","Aruco_Y","Aruco_Z","Drone_yaw"])

"""
Set up mavlink connection


"""

"""
FLOW_TYPE = 1 for your px4 sensor
AHRS_GPS_USE = 0 - Disable GPS for AHRS calculations
GPS_TYPE = 0 - Disable primary GPS
VISO_TYPE = 1 - Set visual odometry type as MAVLINK
EK3_GPS_CHECK = 0 - Disable GPS checks
GPS_AUTO_SWITCH = 0 - Disable auto switch from primary to secondary GPS
GPS_PRIMARY = 0 - Set first (disabled) GPS as primary
EK3_SRC1_VELZ = 0 - Dont use any vertical velocity source
EK3_SRC1_POSXY = 6 - Use External Navigation (vision position estimates) for x and y position
EK3_SRC1_VELXY = 0 - Dont use any horizontal vertical velocity source
ARMING_CHECK = 388598 - Disable GPS arming checks
VISO_DELAY = 250 - Delay parameter for Visual odometry
"""


drone_X = 0
drone_Y = 0
drone_Z = 0
heading = 0

# Connect to the SITL simulation via TCP
# Replace 'localhost' with your simulator's IP address if it's running on a different machine
master = mavutil.mavlink_connection('udp:127.0.0.1:14551')  # Use the correct IP and port

master.wait_heartbeat()  # Wait until we get a heartbeat from the simulation
print("Heartbeat received from SITL simulation.")


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



def send_vp_to_nav(x_global, y_global, alt):
    """
    Sends a vision position estimate message to the autopilot using MAVLink.
    """

    # Send VISION_POSITION_ESTIMATE message
    master.mav.global_vision_position_estimate_send(
        0,  # Timestamp (in microseconds)
        x_global,         # X position (latitude in meters or local frame)
        y_global,         # Y position (longitude in meters or local frame)
        alt,              # Z position (altitude in meters)
        0,                # Roll angle (radians)
        0,                # Pitch angle (radians)
        0,                # Yaw angle (radians)
        [10, 0, 0, 0, 0, 0, 10, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 1, 0,1], # covariance matrix for vision estimates. First two non zeros are the uncertainty in x and y respectively
    )

    # Debug information
    # print(f"Sent VISION_POSITION_ESTIMATE: Lat={x_global}, Lon={y_global}, Alt={alt}")



# def takeoff_drone(target_alt):
#     """
#     Commands the drone to take off to a specified altitude.
#     """
#     print(f"Initiating takeoff to {target_alt} meters...")
#     master.mav.command_long_send(
#         master.target_system,
#         master.target_component,
#         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
#         0,
#         0, 0, 0, 0, 0, 0,
#         target_alt
#     )
#     print("Takeoff command sent.")
    
# def force_arm_message_send():
#     """
#     Forcefully arms the drone, bypassing safety checks.
#     """
#     print("Forcefully arming the drone...")
#     master.mav.command_long_send(
#         master.target_system,
#         master.target_component,
#         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#         0,                  # Confirmation
#         1,                  # Arm (1 to arm, 0 to disarm)
#         21196,              #  Force arm bypass code (magic number)
#         0, 0, 0, 0, 0       # Unused parameters
#     )
   
# def is_drone_armed():
#     """
#     Checks the armed state by monitoring SYS_STATUS message.
#     """
#     return True if master.motors_armed() else False





def set_initial_position():
    """Set initial position to the drone."""
    for _ in range(10): # drone doesnt listen if told once
        
        # send_to_mav(22.6121311, 71.1460848, 200.0,1,0,0,0  )
        lat_gps,long_gps,alt_gps = get_gps_data()
        reset_global_origin(lat_gps, long_gps, alt_gps)    
        orig_lat,orig_lon,origin_alt = lat_gps,long_gps,alt_gps
        print(lat_gps,long_gps,alt_gps )
        print(time.time())
        time.sleep(0.2)

    print("Initial position origin set...")
    for i in range(15):
        # send initial location to the drone for some time. 
        # (This makes the drone happy and ready to arm)
        
        send_vp_to_nav(0,0,0) # initial position for arming
        time.sleep(0.2)
    print("Drone now can be force armed...")
    return  orig_lat,orig_lon,origin_alt 



def set_mode(mode):
    """
    Sets the flight mode of the drone.
    :param mode: The desired mode as a string (e.g., "AUTO", "GUIDED", "STABILIZE").
    """
    # Get the mode ID from the mode string
    mode_id = master.mode_mapping().get(mode)
    
    if mode_id is None:
        print(f"Unknown mode: {mode}")
        return False

    # Send the command to set the mode
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id
    )

    # Wait for the mode to be confirmed
    ack = master.recv_match(type='HEARTBEAT', blocking=True)
    if ack.custom_mode == mode_id:
        print(f"Mode successfully set to {mode}.")
        return True
    time.sleep(0.1)

        
    print(f"Mode set to {mode} FAILED.")
    return False
 


# def arm_and_takeoff(target_alt):
#   """Arm and take off drone and take to height. Function will release 
#   control only when drone reaches target height, else it will keep on 
#   retrying and rearming"""
  
#   send_vp_to_nav(0,0,0) # send 0,0,0 after few lines of code for matching frequency of 5hz
#   set_mode("GUIDED")
   
#   force_arm_message_send()
#   time.sleep(0.2)
  
#   print("Arming...")
#   takeoff_drone(target_alt)
#   print("Trying to take off...")
#   counter = 0
#   while True: 
#       send_vp_to_nav(0,0,0)
#       alt = get_global_alt()
#       print("Current altitude = ", alt)
#       print(is_drone_armed())
#       if abs(alt  - target_alt) <1: 
#               set_initial_position() 
#               # set initial position to remove any drifts that would have been 
#               # caused  while drone got armed
#               return True
#       time.sleep(0.2)
#       if not is_drone_armed(): # sometimes drone disarms due to error
#         set_initial_position()
#         print("Some error in arming.. Retrying...")
#         return arm_and_takeoff(target_alt)
      


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


def lock_yaw(heading_deg=0.0, yaw_speed=30.0, direction=1, relative=False):
    """
    Lock drone yaw to a specific heading.

    Parameters:
        master (mavutil.mavlink_connection): The MAVLink connection.
        heading_deg (float): Desired yaw in degrees.
        yaw_speed (float): Speed in deg/s.
        direction (int): 1 = clockwise, -1 = counter-clockwise.
        relative (bool): If True, heading is relative to current yaw.
    """
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        heading_deg,      # target yaw in degrees
        yaw_speed,        # yaw speed in deg/s
        direction,        # direction (1=cw, -1=ccw)
        int(relative),    # relative=1, absolute=0
        0, 0, 0           # unused
    )


def get_drone_yaw():
    master.mav.request_data_stream_send(master.target_system, master.target_component,
                                    mavutil.mavlink.MAV_DATA_STREAM_EXTRA2, 25, 1)
    attitude_msg = master.recv_match(type='VFR_HUD', blocking=True)
    if attitude_msg:
        heading = float(attitude_msg.heading)  # Heading in degrees (float for full decimal precision)
    return heading


my_lat,my_lon,_ = get_gps_data()
set_home_position(my_lat,my_lon) 
target_alt = 80
send_vp_to_nav(0,0,0) # send 0,0,0 after few lines of code for matching frequency of 5hz
set_mode("GUIDED")
# force_arm_message_send()
time.sleep(0.2)
# takeoff_drone(target_alt)

while True: 

    lock_yaw()
    alt = get_global_alt()
    # print(aruco_tracker.aruco_X,-aruco_tracker.aruco_Y)
    # send_vp_to_nav(aruco_tracker.aruco_X,-aruco_tracker.aruco_Y,alt)
    # writer.writerow([aruco_tracker.drone_X,aruco_tracker.drone_Y,aruco_tracker.drone_Z,aruco_tracker.aruco_X,aruco_tracker.aruco_Y,aruco_tracker.aruco_Z,aruco_tracker.drone_yaw])
    if abs(alt  - target_alt) <1: 
        set_initial_position() 
        break
        
    send_vp_to_nav(0,0,0) # send 0,0,0 after few lines of code for matching frequency of 5hz
    # if not is_drone_armed(): # sometimes drone disarms due to error
    set_initial_position()
    #     print("Some error in arming.. Retrying...")
    #     force_arm_message_send()
    #     send_vp_to_nav(0,0,0) # send 0,0,0 after few lines of code for matching frequency of 5hz
    #     set_mode("GUIDED")
    #     force_arm_message_send()
    #     time.sleep(0.2)
    #     print("Arming...")
    #     takeoff_drone(target_alt)
    #     print("Trying to take off...")
    
    time.sleep(0.1)

# print(f"Drone is at {target_alt} m height now...")
# set_mode("LAND")

try:
    while True: 
        
        lock_yaw()
        alt = get_global_alt()
        # send_vp_to_nav(aruco_tracker.aruco_X,-aruco_tracker.aruco_Y,alt)
        # writer.writerow([aruco_tracker.drone_X,aruco_tracker.drone_Y,aruco_tracker.drone_Z,aruco_tracker.aruco_X,aruco_tracker.aruco_Y,aruco_tracker.aruco_Z,aruco_tracker.drone_yaw])
        time.sleep(0.1)
except KeyboardInterrupt:
    print("Keyboard interrupt detected. Exiting...")
    