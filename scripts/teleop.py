from pymavlink import mavutil
import time

# Connect to MAVProxy (UDP port 14550)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for heartbeat
master.wait_heartbeat()
print("Heartbeat received!")

# Arm the drone
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)
print("Arming...")

# Wait for arming confirmation
while not master.motors_armed():
    print("Waiting for arming...")
    time.sleep(1)

# Takeoff to 5m
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 5
)
print("Taking off...")
time.sleep(5)  # Wait for takeoff

# --- FIXED: Use a 32-bit compatible timestamp ---
timestamp = int(time.time() * 1000) % (2**32)  # Ensure 32-bit value

# Move forward at 1 m/s (NED coordinates)
master.mav.set_position_target_local_ned_send(
    timestamp,  # Now guaranteed to be 32-bit
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  # Velocity control (ignore position)
    0, 0, -5,  # Position (not used)
    5, 0, 0,   # Velocity (forward 1 m/s, no side/lateral)
    0, 0, 0,   # Acceleration (not used)
    0, 0       # Yaw, yaw rate
)
print("Moving forward...")
time.sleep(5)  # Move for 5 sec

# -----------------------------------------------------------------
# Move sideways :
# Move forward at 1 m/s (NED coordinates)
master.mav.set_position_target_local_ned_send(
    timestamp,  # Now guaranteed to be 32-bit
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  # Velocity control (ignore position)
    0, 0, -5,  # Position (not used)
    0, 1, 0,   # Velocity (forward 1 m/s, no side/lateral)
    0, 0, 0,   # Acceleration (not used)
    0, 0       # Yaw, yaw rate
)
print("Moving Sideways...")
time.sleep(5)  # Move for 5 sec
# -----------------------------------------------------------------

# -----------------------------------------------------------------
# Move sideways :
# Move forward at 1 m/s (NED coordinates)
master.mav.set_position_target_local_ned_send(
    timestamp,  # Now guaranteed to be 32-bit
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  # Velocity control (ignore position)
    0, 0, -5,  # Position (not used)
    0, -5, 0,   # Velocity (forward 1 m/s, no side/lateral)
    0, 0, 0,   # Acceleration (not used)
    0, 0       # Yaw, yaw rate
)
print("Moving Sideways...")
time.sleep(5)  # Move for 5 sec
# -----------------------------------------------------------------

# -----------------------------------------------------------------
# Move sideways :
# Move forward at 1 m/s (NED coordinates)
master.mav.set_position_target_local_ned_send(
    timestamp,  # Now guaranteed to be 32-bit
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  # Velocity control (ignore position)
    0, 0, -5,  # Position (not used)
    -5.0, 0, 0,   # Velocity (forward 1 m/s, no side/lateral)
    0, 0, 0,   # Acceleration (not used)
    0, 0       # Yaw, yaw rate
)
print("Moving Sideways...")
time.sleep(5)  # Move for 5 sec
# -----------------------------------------------------------------


# Land
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0
)
print("Landing...")