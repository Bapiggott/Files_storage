import asyncio
import math
from mavsdk import System
import requests
from datetime import datetime

drone = System()
starting_position = None

# Function to ensure timeout with telemetry checks
async def timeout_check(timeout_seconds, check_function):
    start_time = asyncio.get_event_loop().time()
    while (asyncio.get_event_loop().time() - start_time) < timeout_seconds:
        if await check_function():
            return True
        await asyncio.sleep(1)  # Avoid tight loops
    print("Timeout reached.")
    return False

# Helper function to calculate distance between two GPS coordinates
def calculate_distance(lat1, lon1, lat2, lon2):
    R = 6371000  # Radius of Earth in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = math.sin(delta_phi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2) ** 2
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

    return R * c

# Connect to the drone
async def connect_drone():
    global drone
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            break

# Ensure the drone is armed and has taken off (without offboard)
async def ensure_armed_and_taken_off():
    global drone

    is_armed = False
    async for armed in drone.telemetry.armed():
        is_armed = armed
        break

    if not is_armed:
        print("Arming drone...")
        try:
            await drone.action.arm()
            print("Drone armed")
        except Exception as e:
            print(f"Failed to arm the drone: {e}")
            return
    else:
        print("Drone is already armed")

    in_air = False
    async for air in drone.telemetry.in_air():
        in_air = air
        break

    if not in_air:
        print("Taking off...")
        try:
            async for position in drone.telemetry.position():
                starting_altitude = position.relative_altitude_m
                print(f"Current altitude: {starting_altitude} meters")
                break

            await drone.action.set_takeoff_altitude(1.5)
            await drone.action.takeoff()

            # Use a timeout to wait until the drone reaches the target altitude
            await timeout_check(2000, lambda: asyncio.ensure_future(reached_target_altitude(1.5, starting_altitude)))
        except Exception as e:
            print(f"Failed to take off: {e}")
            return
    else:
        print("Drone is already in the air")

async def get_current_altitude():
    global drone
    async for position in drone.telemetry.position():
        current_altitude = position.relative_altitude_m
        print(f"Current altitude: {current_altitude} meters")
        return current_altitude
# Check if the drone has reached a target altitude
async def reached_target_altitude(target_altitude, starting_altitude, tolerance=0.01, stability_count=15):
    stable_readings = 0  # Counter for how many consecutive stable readings we get
    previous_altitude = None

    async for position in drone.telemetry.position():
        #print(f"Starting alt: {starting_altitude}")
        current_altitude = await get_current_altitude() #position.relative_altitude_m
        #print(f"Current altitude: {current_altitude} meters")

        # Check if the current altitude is within the tolerance of the target altitude
        if current_altitude >= (target_altitude - tolerance):
            print(f"Reached target altitude of {target_altitude} meters (within tolerance of {tolerance} meters)")
            return True

        # If this is the first reading, set previous_altitude to current
        if previous_altitude is None:
            previous_altitude = current_altitude

        # Check if the altitude has been stable (within tolerance of previous altitude)
        if abs(current_altitude - previous_altitude) < tolerance:
            if current_altitude >= (target_altitude / 2.2):
                stable_readings += 1
        else:
            stable_readings = 0  # Reset counter if the altitude fluctuates
        previous_altitude = current_altitude

        # If we've had 10 consecutive stable readings, consider the altitude stable
        if stable_readings >= stability_count:
            print(f"Altitude has been stable for {stability_count} consecutive readings.")
            return True

        # Wait a bit before the next telemetry check to avoid tight looping
        await asyncio.sleep(1)

    return False


# Return to the start position using simple location commands
async def return_to_start_position():
    global drone, starting_position

    if starting_position is None:
        print("Starting position not recorded.")
        return

    print("Returning to starting position...")
    await drone.action.goto_location(
        starting_position.latitude_deg,
        starting_position.longitude_deg,
        starting_position.absolute_altitude_m,
        0  # Assuming you want to face north upon return
    )

    # Wait for the drone to reach the target location by monitoring telemetry data
    await timeout_check(60, lambda: asyncio.ensure_future(reached_target_position(
        starting_position.latitude_deg, starting_position.longitude_deg)))

# Check if the drone has reached the desired position
async def reached_target_position(target_latitude, target_longitude):
    stable_readings = 0  # Counter for how many consecutive stable readings we get
    previous_altitude = None
    async for position in drone.telemetry.position():
        distance = calculate_distance(position.latitude_deg, position.longitude_deg, target_latitude, target_longitude)
        print(distance)
        if distance < 0.00001:  # Set a threshold for how close it should be
            print("Reached the target position.")
            return True
    return False

# Move drone by setting GPS coordinates based on current position and direction
async def move_in_direction(distance: int, direction: str):
    global drone

    async for position in drone.telemetry.position():
        current_lat = position.latitude_deg
        current_lon = position.longitude_deg
        break

    heading_deg = await get_heading()
    heading_rad = math.radians(heading_deg)

    # Calculate new position based on direction and current heading
    if direction == "forward":
        new_lat = current_lat + (distance * math.cos(heading_rad)) / 111320  # Convert meters to degrees
        new_lon = current_lon + (distance * math.sin(heading_rad)) / (111320 * math.cos(math.radians(current_lat)))
    elif direction == "backward":
        new_lat = current_lat - (distance * math.cos(heading_rad)) / 111320
        new_lon = current_lon - (distance * math.sin(heading_rad)) / (111320 * math.cos(math.radians(current_lat)))
    elif direction == "left":
        new_lat = current_lat + (distance * math.cos(heading_rad + math.pi / 2)) / 111320
        new_lon = current_lon + (distance * math.sin(heading_rad + math.pi / 2)) / (111320 * math.cos(math.radians(current_lat)))
    elif direction == "right":
        new_lat = current_lat + (distance * math.cos(heading_rad - math.pi / 2)) / 111320
        new_lon = current_lon + (distance * math.sin(heading_rad - math.pi / 2)) / (111320 * math.cos(math.radians(current_lat)))
    else:
        raise ValueError(f"Unknown direction: {direction}")

    print(f"Moving {direction} by {distance} meters...")
    await drone.action.goto_location(new_lat, new_lon, position.absolute_altitude_m, heading_deg)

    # Wait for the drone to reach the new position
    await timeout_check(2000, lambda: asyncio.ensure_future(reached_target_position(new_lat, new_lon)))

# Get the current heading
async def get_heading():
    global drone
    async for position in drone.telemetry.heading():
        print(f"Current Heading: {position.heading_deg} degrees")
        return position.heading_deg

# Turn the drone by adjusting yaw
async def turn_cw(degrees: float):
    current_heading = await get_heading()
    new_heading = (current_heading + degrees) % 360
    print(f"Turning clockwise to {new_heading} degrees")
    await drone.action.goto_location(0, 0, 0, new_heading)

async def turn_ccw(degrees: float):
    current_heading = await get_heading()
    new_heading = (current_heading - degrees) % 360
    print(f"Turning counterclockwise to {new_heading} degrees")
    await drone.action.goto_location(0, 0, 0, new_heading)

# Function to check if the object is visible
def is_visible(object_name: str) -> bool:
    object_name = object_name.lower()
    scene_description = get_latest_detection()
    for item in scene_description:
        if object_name in item["name"].lower():
            return True
    return False

# Get object X position based on detection
def object_x(object: str) -> float:
    scene_description = get_latest_detection()
    for item in scene_description:
        if object in item["name"]:
            return ((item["xmax"] + item["xmin"]) / 2) / 1920
    return -1

# Get latest detection (from YOLO server)
def get_latest_detection():
    yolo_server_url = "http://localhost:5000/detections/latest"
    try:
        response = requests.get(yolo_server_url)
        if response.status_code == 200:
            detection_result = response.json()
            return detection_result
        else:
            print(f"Error: Received status code {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")
        return None

# Function to land the drone
async def land_drone():
    global drone
    print("Landing the drone...")
    await drone.action.land()

