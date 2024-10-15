from dronekit import connect, VehicleMode, LocationGlobalRelative
from pymavlink import mavutil
import time

# Connect to the Vehicle (replace 'tcp:127.0.0.1:5760' with your connection string)
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Function to arm and takeoff to a specified altitude
def arm_and_takeoff(target_altitude):
    print("Arming motors")
    vehicle.mode = VehicleMode("GUIDED")
    vehicle.armed = True

    # Confirm vehicle armed before attempting to take off
    while not vehicle.armed:
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(target_altitude)

    # Wait until the vehicle reaches a safe height before proceeding
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        if vehicle.location.global_relative_frame.alt >= target_altitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

# Takeoff to 10 meters
arm_and_takeoff(10)

# Function to send NED velocity commands
def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,  # frame
        0b0000111111000111,  # type_mask (only velocity components enabled)
        0, 0, 0,  # x, y, z positions (not used)
        velocity_x, velocity_y, velocity_z,  # x, y, z velocity in m/s
        0, 0, 0,  # accelerations (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    
    for _ in range(0, duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)

# Example: Move forward at 1 m/s for 5 seconds
send_ned_velocity(1, 0, 0, 5)

# Hover for 2 seconds
send_ned_velocity(0, 0, 0, 2)

# Land the drone
print("Landing")
vehicle.mode = VehicleMode("LAND")

# Close the vehicle object
vehicle.close()

