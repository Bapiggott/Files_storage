import time
from pymavlink import mavutil

# Create the connection
master = mavutil.mavlink_connection('udpin:0.0.0.0:14550')

# Wait for a heartbeat before sending commands
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received")

# Arm the drone
def arm_drone():
    master.arducopter_arm()  # Arm the drone
    master.motors_armed_wait()  # Wait until motors are armed
    print("Drone armed")

# Takeoff to a specified altitude
def takeoff(altitude):
    master.mav.command_long_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,  # command
        0,  # confirmation
        0, 0, 0, 0,  # params 1-4 are unused
        0, 0,  # latitude, longitude (not used)
        altitude  # altitude (in meters)
    )
    print(f"Takeoff command sent to {altitude} meters")

# Send mission items (waypoints)
def send_mission():
    waypoints = [
        (37.0, -122.0, 10),  # (latitude, longitude, altitude)
        (37.0, -122.1, 20),
        (37.1, -122.1, 30),
    ]
    for seq, waypoint in enumerate(waypoints):
        lat, lon, alt = waypoint
        master.mav.mission_item_send(
            master.target_system,
            master.target_component,
            seq,  # sequence number
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,  # command
            2,  # current/next waypoint (0=next, 1=current)
            1,  # autocontinue
            0, 0, 0, 0,  # params 1-4 (not used)
            lat,  # latitude
            lon,  # longitude
            alt   # altitude
        )
        print(f"Waypoint {seq} sent: lat={lat}, lon={lon}, alt={alt} meters")
    master.mav.mission_count_send(master.target_system, master.target_component, len(waypoints))
    print("Mission sent")

# Set the mode to AUTO to start the mission
def start_mission():
    master.set_mode('AUTO')
    print("Mission started")

# Return to launch (RTL)
def return_to_launch():
    master.mav.command_long_send(
        master.target_system,  # target_system
        master.target_component,  # target_component
        mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,  # command
        0,  # confirmation
        0, 0, 0, 0, 0, 0, 0  # params 1-7 are unused
    )
    print("Return to launch command sent")

# Arm the drone and take off
arm_drone()
time.sleep(1)  # Wait a bit to ensure the drone is armed
takeoff(10)  # Takeoff to 10 meters

# Wait for the drone to reach the takeoff altitude
time.sleep(10)  # Adjust this based on the takeoff altitude

# Send the mission waypoints
send_mission()
time.sleep(2)

# Start the mission
start_mission()

# Wait for mission completion
time.sleep(30)  # Adjust this based on mission length

# Return to launch
return_to_launch()

# Wait for the drone to return and land
time.sleep(20)  # Adjust this based on the distance to home
