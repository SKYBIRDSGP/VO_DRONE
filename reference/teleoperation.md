# TELEOPERATION OF IRIS USING MAVLink and MAVROS

### Making the drone to some height:

``` python
# Takeoff to x m
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0, 0, 0, 0, 0, 0, 0, x
)
```

### Making the drone move in X, Y Or Z:

``` python
# Move forward at 1 m/s (NED coordinates)
master.mav.set_position_target_local_ned_send(
    timestamp,  # Now guaranteed to be 32-bit
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    0b0000111111000111,  # Velocity control (ignore position)
    0, 0, -5,  # Position (not used)
    x, y, z,   # Velocity (x, y, z in their respective direction)
    0, 0, 0,   # Acceleration (not used)
    0, 0       # Yaw, yaw rate
)
```

### Making the drone Land:

``` python
# Land
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_NAV_LAND,
    0, 0, 0, 0, 0, 0, 0, 0
)
```