import time
import math
from pymavlink import mavutil

# Initialize connection to the MAVLink device
connection = mavutil.mavlink_connection('udpout:127.0.0.1:14550')


def send_velocity_target(vx, vy, vz, yaw):
    """
    Send velocity target to the MAVLink device.
    
    Parameters:
        vx (float): Velocity in the X direction.
        vy (float): Velocity in the Y direction.
        vz (float): Velocity in the Z direction.
        yaw (float): Yaw angle in radians.
    """
    try:
        # Use a smaller, more controlled timestamp for testing
        timestamp = int(time.time() * 1000) % 4294967296  # Ensure it fits within 32-bit unsigned integer range
        
        # Debug print for timestamp
        print(f"Timestamp: {timestamp}")

        connection.mav.set_position_target_local_ned_send(
            timestamp,  # Timestamp
            1,  # Target system (usually 1 for the single drone)
            1,  # Target component (usually 1 for the single drone)
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # Coordinate frame
            0b0000111111000111,  # Type mask (position and velocity targets enabled)
            0.0,  # x position (m)
            0.0,  # y position (m)
            0.0,  # z position (m)
            float(vx),  # x velocity (m/s)
            float(vy),  # y velocity (m/s)
            float(vz),  # z velocity (m/s)
            0.0,  # x acceleration (m/s^2)
            0.0,  # y acceleration (m/s^2)
            0.0,  # z acceleration (m/s^2)
            float(yaw),  # yaw (radians)
            0.0  # yaw rate (radians/s)
        )
    except Exception as e:
        print(f"Failed to send velocity target: {e}")

def get_current_position():
    """
    Retrieve the current position from the MAVLink device.
    
    Returns:
        tuple: (x, y, z, vx, vy, vz) representing current position and velocities.
    """
    try:
        message = connection.recv_match(type='LOCAL_POSITION_NED', blocking=True)
        x = message.x
        y = message.y
        z = message.z
        vx = message.vx
        vy = message.vy
        vz = message.vz
        return x, y, z, vx, vy, vz
    except Exception as e:
        print(f"Failed to get current position: {e}")
        return None

def main():
    # Example velocity values (m/s) and yaw angle (radians)
    vx = 1.0  # Move forward at 1 m/s
    vy = 0.0  # No sideward movement
    vz = 0.0  # No vertical movement
    yaw = 0.0  # No change in yaw angle
    
    print("Connection established")
    
    while True:
        send_velocity_target(vx, vy, vz, yaw)
        position = get_current_position()
        if position:
            x, y, z, vx, vy, vz = position
            print(f"Current position: x={x}, y={y}, z={z}, vx={vx}, vy={vy}, vz={vz}")
        else:
            print("Position data unavailable")
        
        time.sleep(1)  # Wait for 1 second before sending the next command

if __name__ == "__main__":
    main()

