from pymavlink import mavutil
import time

# Starting a connection with the UDP port
master = mavutil.mavlink_connection('udpin:localhost:14551')

# Waits for the first heartbeat

master.wait_heartbeat()
print("Heartbeat from system ")

master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        4  # GUIDED mode for ArduCopter
    )
print("Setting GUIDED mode...")
time.sleep(1)

# Arm the drone
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
    0, 1, 0, 0, 0, 0, 0, 0
)

while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
        if msg and msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            print("Armed!")
            break
        print("Waiting for arming...")
        time.sleep(1.5)

time.sleep(2.0)

# Takeoff to 5m
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, 5
)
print("Taking off...")
time.sleep(5)  # Wait for takeoff

# Takeoff to 5m
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0
)
print("Landing...")
time.sleep(5)  # Wait for takeoff
