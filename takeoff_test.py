from dronekit import connect, VehicleMode
from pymavlink import mavutil

# Connect to the Vehicle (use your connection string)
vehicle = connect('tcp:127.0.0.1:5760', wait_ready=True)

# Function to send a MAV_CMD_NAV_TAKEOFF command
def send_takeoff_command(target_altitude):
    msg = vehicle.message_factory.command_long_encode(
        0, 0,    # target_system, target_component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
        0,       # confirmation
        0, 0, 0, 0,  # param 1-4 (unused)
        0, 0, target_altitude)  # param 5-7: latitude, longitude, altitude
        
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Example: Arm and take off to 10 meters
vehicle.mode = VehicleMode("GUIDED")
vehicle.armed = True
while not vehicle.armed:
    print("Waiting for arming...")
    time.sleep(1)

send_takeoff_command(10)

# Monitor altitude until reaching the target
while True:
    print(" Altitude: ", vehicle.location.global_relative_frame.alt)
    if vehicle.location.global_relative_frame.alt >= 10 * 0.95:
        print("Reached target altitude")
        break
    time.sleep(1)

# Close the vehicle object
vehicle.close()
