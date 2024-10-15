import time
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for a heartbeat before sending commands
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")

# Define a function to send manual control commands
def send_manual_control(x, y, z, r, buttons):
    try:
        master.mav.manual_control_send(
            master.target_system,
            x,
            y,
            z,
            r,
            buttons
        )
        print("Manual control command sent")
    except Exception as e:
        print(f"Failed to send manual control command: {e}")

# Define a function to send GPS input
def send_gps_input():
    try:
        master.mav.gps_input_send(
            0,  # Timestamp (micros since boot or Unix epoch)
            0,  # ID of the GPS for multiple GPS inputs
            # Flags indicating which fields to ignore (see GPS_INPUT_IGNORE_FLAGS enum).
            # All other fields must be provided.
            (mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ |
             mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_VERT |
             mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY),
            0,  # GPS time (milliseconds from start of GPS week)
            0,  # GPS week number
            3,  # 0-1: no fix, 2: 2D fix, 3: 3D fix. 4: 3D with DGPS. 5: 3D with RTK
            0,  # Latitude (WGS84), in degrees * 1E7
            0,  # Longitude (WGS84), in degrees * 1E7
            0,  # Altitude (AMSL, not WGS84), in m (positive for up)
            1,  # GPS HDOP horizontal dilution of position in m
            1,  # GPS VDOP vertical dilution of position in m
            0,  # GPS velocity in m/s in NORTH direction in earth-fixed NED frame
            0,  # GPS velocity in m/s in EAST direction in earth-fixed NED frame
            0,  # GPS velocity in m/s in DOWN direction in earth-fixed NED frame
            0,  # GPS speed accuracy in m/s
            0,  # GPS horizontal accuracy in m
            0,  # GPS vertical accuracy in m
            7   # Number of satellites visible.
        )
        print("GPS input sent")
    except Exception as e:
        print(f"Failed to send GPS input: {e}")

# Example values for manual control
# x, y, z, and r are joystick inputs; buttons is a bitmask for button presses
x = 500  # Positive x value
y = -500  # Negative y value
z = 250  # Neutral throttle
r = 500  # Positive rotation
buttons = 0  # No buttons pressed

# Main loop
while True:
    # Send manual control command
    send_manual_control(x, y, z, r, buttons)
    
    # Send GPS input
    send_gps_input()
    
    # Wait for a short interval before sending the next set of commands
    time.sleep(0.2)  # Adjust this value as needed
