#!/usr/bin/env python3

import sys
import geometry_msgs.msg

from pymavlink import mavutil
import time

if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

# Connect to MAVProxy (UDP port 14550)
master = mavutil.mavlink_connection('udpin:0.0.0.0:14551')

# Wait for heartbeat
master.wait_heartbeat()
print("Heartbeat received!")

# master.mav.param_set_send(
#     master.target_system,
#     master.target_component,
#     b"EK3_GPS_TYPE",
#     3,
#     mavutil.mavlink.MAV_PARAM_TYPE_INT32,
# )

def getKey(settings):
    if sys.platform == 'win32':
        # getwch() returns a string on Windows
        key = msvcrt.getwch()
    else:
        tty.setraw(sys.stdin.fileno())
        # sys.stdin.read() returns a string on Linux
        key = sys.stdin.read(1)
        if key == '\x1b':  # if the first character is \x1b, we might be dealing with an arrow key
            additional_chars = sys.stdin.read(2)  # read the next two characters
            key += additional_chars  # append these characters to the key
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def saveTerminalSettings():
    if sys.platform == 'win32':
        return None
    return termios.tcgetattr(sys.stdin)


def restoreTerminalSettings(old_settings):
    if sys.platform == 'win32':
        return
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

# --- Use a 32-bit compatible timestamp ---
timestamp = int(time.time() * 1000) % (2**32)  # Ensure 32-bit value

def main():
    settings = saveTerminalSettings()
    arm_toggle = False
        
    try:
        print("Give the Command")
        # print(vels(speed, turn))
        while True:
            key = getKey(settings)

            if key == ' ':
                if not arm_toggle:
                    arm_toggle = not arm_toggle
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

                    print("Arming the Drone !")
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

                else:
                    arm_toggle = not arm_toggle
                    # Land
                    print("Landing the Drone !")
                    master.mav.command_long_send(
                        master.target_system, master.target_component,
                        mavutil.mavlink.MAV_CMD_NAV_LAND,
                        0, 0, 0, 0, 0, 0, 0, 0
                    )
                    time.sleep(1.5)
            
            elif key == 't':
                # Takeoff to 2m
                master.mav.command_long_send(
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
                    0, 0, 0, 0, 0, 0, 0, 2
                )
                print("Taking Off !")
                time.sleep(2.5)   # Wait for takeoff

            elif key == '\x1b[A' :
                # Move forward at 1 m/s (NED coordinates)
                master.mav.set_position_target_local_ned_send(
                    timestamp,  # Now guaranteed to be 32-bit
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,  # Velocity control (ignore position)
                    0, 0, -2,  # Position (not used)
                    0.5, 0, 0,   # Velocity (forward 1 m/s, no side/lateral)
                    0, 0, 0,   # Acceleration (not used)
                    0, 0       # Yaw, yaw rate
                )
                print("Moving Ahead !")
                time.sleep(2.5) 
            
            elif key == '\x1b[B' :
                # Move forward at 1 m/s (NED coordinates)
                master.mav.set_position_target_local_ned_send(
                    timestamp,  # Now guaranteed to be 32-bit
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,  # Velocity control (ignore position)
                    0, 0, -2,  # Position (not used)
                    -0.5, 0, 0,   # Velocity (forward 1 m/s, no side/lateral)
                    0, 0, 0,   # Acceleration (not used)
                    0, 0       # Yaw, yaw rate
                )
                print("Moving Back !")
                time.sleep(2.5) 
            
            elif key == '\x1b[C' :
                # Move forward at 1 m/s (NED coordinates)
                master.mav.set_position_target_local_ned_send(
                    timestamp,  # Now guaranteed to be 32-bit
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,  # Velocity control (ignore position)
                    0, 0, -2,  # Position (not used)
                    0, 0.5, 0,   # Velocity (forward 1 m/s, no side/lateral)
                    0, 0, 0,   # Acceleration (not used)
                    0, 0       # Yaw, yaw rate
                )
                print("Moving Left !")
                time.sleep(2.5) 
            
            elif key == '\x1b[D' :
                # Move forward at 1 m/s (NED coordinates)
                master.mav.set_position_target_local_ned_send(
                    timestamp,  # Now guaranteed to be 32-bit
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,
                    0b0000111111000111,  # Velocity control (ignore position)
                    0, 0, -2,  # Position (not used)
                    0, -0.5, 0,   # Velocity (forward 1 m/s, no side/lateral)
                    0, 0, 0,   # Acceleration (not used)
                    0, 0       # Yaw, yaw rate
                )
                print("Moving Right !")
                time.sleep(2.5) 
            
            elif key == 'w' :
                # Move forward at 1 m/s (NED coordinates)
                master.mav.set_position_target_local_ned_send(
                    timestamp,  # Now guaranteed to be 32-bit
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111000111,  # Velocity control (ignore position)
                    0, 0, -2,  # Position (not used)
                    0, 0, -0.5,   # Velocity (forward 1 m/s, no side/lateral)
                    0, 0, 0,   # Acceleration (not used)
                    0, 0       # Yaw, yaw rate
                )
                print("Moving Right !")
                time.sleep(2.5) 
            
            elif key == 's' :
                # Move forward at 1 m/s (NED coordinates)
                master.mav.set_position_target_local_ned_send(
                    timestamp,  # Now guaranteed to be 32-bit
                    master.target_system, master.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111000111,  # Velocity control (ignore position)
                    0, 0, -2,  # Position (not used)
                    0, 0, 0.5,   # Velocity (forward 1 m/s, no side/lateral)
                    0, 0, 0,   # Acceleration (not used)
                    0, 0       # Yaw, yaw rate
                )
                print("Moving Right !")
                time.sleep(2.5) 

            elif key == 'c':
                # Rotate clockwise (negative yaw rate)
                master.mav.set_position_target_local_ned_send(
                    timestamp,
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,  # Use BODY frame
                    0b0000011111000111,  # Enable velocity + yaw rate
                    0, 0, 0,             # position (ignored)
                    0, 0, 0,             # velocity (ignored)
                    0, 0, 0,             # accel (ignored)
                    0, -0.5              # yaw, yaw_rate (rad/s)
                )
                print("Yawing Clockwise")
                time.sleep(0.5)

            elif key == 'v':
                # Rotate counter-clockwise (positive yaw rate)
                master.mav.set_position_target_local_ned_send(
                    timestamp,
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_FRAME_BODY_NED,  # Use BODY frame
                    0b0000011111000111,  # Enable velocity + yaw rate
                    0, 0, 0,
                    0, 0, 0,
                    0, 0, 0,
                    0, 0.5               # yaw, yaw_rate (rad/s)
                )
                print("Yawing Counter-Clockwise")
                time.sleep(0.5)
                        
            elif key == 'C':
                return

            else:
                print("Teleoperated Drone 😄")    

    except Exception as e:
        print("Exception !!!")

if __name__ == '__main__':
    main()