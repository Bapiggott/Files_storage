# functions.py
import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import requests
from mavsdk.offboard import VelocityNedYaw

async def moving_drone(distance: int, direction: str) -> None:
    heading_deg = asyncio.run(get_heading())
    heading_rad = math.radians(heading_deg)

    # Calculate NED movement based on direction and heading
    if direction == "forward":
        north = distance * math.cos(heading_rad)
        east = distance * math.sin(heading_rad)
    elif direction == "backward":
        north = -distance * math.cos(heading_rad)
        east = -distance * math.sin(heading_rad)
    elif direction == "left":
        north = distance * math.cos(heading_rad + math.pi / 2)
        east = distance * math.sin(heading_rad + math.pi / 2)
    elif direction == "right":
        north = distance * math.cos(heading_rad - math.pi / 2)
        east = distance * math.sin(heading_rad - math.pi / 2)
    else:
        raise ValueError(f"Unknown direction: {direction}")
    drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            break
    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                        with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Move the drone based on calculated NED position
    print(f"Moving {direction} by {distance} meters (Heading: {heading_deg} degrees)...")
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, 0.0, heading_deg))  # Keep altitude and yaw stable
    await asyncio.sleep(10)

    try:
        await drone.offboard.start()
        await asyncio.sleep(5)  # Allow the drone to move
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Offboard mode failed with error: {error._result.result}")

    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, 0.0, 180.0))
    await asyncio.sleep(10)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed \
                        with error code: {error._result.result}")


async def get_heading():
    # Connect to the drone
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            break

    # Subscribe to telemetry heading data
    async for position in drone.telemetry.heading():
        print(f"Current Heading: {position.heading_deg} degrees")
        # break  # Exit after one reading, or remove this to continuously get updates
        return position.heading_deg

def move_forward(distance: int) -> None:
    moving_drone(distance, "forward")

def move_backward(distance: int) -> None:
    moving_drone(distance, "backward")


async def set_heading_with_velocity(yaw_deg):
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            break

    # Arm the drone
    print("Arming drone...")
    await drone.action.arm()

    # Set the offboard mode
    print("Setting offboard mode...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, yaw_deg))

    try:
        await drone.offboard.start()
        print(f"Rotating to {yaw_deg} degrees heading...")
        await asyncio.sleep(5)
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Offboard mode failed with error: {error._result.result}")


# asyncio.run(set_heading_with_velocity(145))  # Example: Set heading to 90 degrees
def move_left(distance: int) -> None:
    moving_drone(distance, "left")

def move_right(distance: int) -> None:
    moving_drone(distance, "right")


async def changing_elevation(distance: int) -> None:
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            break

    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed \
                        with error code: {error._result.result}")
        print("-- Disarming")
        await drone.action.disarm()
        return

    # Move the drone based on calculated NED position
    if distance <= 0:
        print(f"Moving up by {-distance} meters ...")
    else:
        print(f"Moving down by {distance} meters ...")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -distance, 0.0))  # Keep altitude and yaw stable
    await asyncio.sleep(10)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: \
                  {error._result.result}")

def move_up(distance: int) -> None:
    asyncio.run(changing_elevation(-distance))

def move_down(distance: int) -> None:
    asyncio.run(changing_elevation(distance))

def turn_cw(degrees: int) -> None:
    heading_deg = asyncio.run(get_heading())
    while degrees >= 360:
        degrees = degrees - 360
    asyncio.run(set_heading_with_velocity(heading_deg + degrees))

def turn_ccw(degrees: int) -> None:
    heading_deg = asyncio.run(get_heading())
    while degrees >= 360:
        degrees = degrees - 360
    asyncio.run(set_heading_with_velocity(heading_deg - degrees))

def move_in_circle(cw: bool) -> None:
    heading_deg = asyncio.run(get_heading())
    for degrees in range(10):
        asyncio.run(set_heading_with_velocity(heading_deg + (36*degrees)))

async def delay(seconds: int) -> None:
    await asyncio.sleep(seconds)


def get_latest_detection():
    """
    Retrieves the latest detection result from the YOLO server.
    """
    yolo_server_url = "http://localhost:5000/detections/latest"
    try:
        response = requests.get(yolo_server_url)
        if response.status_code == 200:
            detection_result = response.json()
            """print("Latest Detection:")
            print(detection_result)"""
            return detection_result
        elif response.status_code == 404:
            print("No detection results available.")
            return None
        else:
            print(f"Error: Received status code {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")
        return None

def scan(object_name: str) -> bool:
    pass  # Implement your function here

def scan_abstract(question: str) -> bool:
    pass  # Implement your function here> None:

def goto(object_name: str) -> None:
    pass  # Implement your function here

def is_visible(object_name: str) -> bool:
    pass  # Implement your function here

def objectx(object: str) -> float:
    for x in range(0,4):
        scene_description = get_latest_detection()
        for item in scene_description:
                #print(item)
                if object in item["name"]:
                    objectx = ((item["xmax"] + item["xmin"]) / 2) / 1920
                    return objectx
    return  -1

def objecty(object: str) -> float:
    for x in range(0,4):
        scene_description = get_latest_detection()
        for item in scene_description:
                if object in item["name"]:
                    objecty = ((item["ymax"] + item["ymin"]) / 2) / 1200
                    return objecty
    return -1

def object_width(object: str) -> float:
    for x in range(0,4):
        scene_description = get_latest_detection()
        for item in scene_description:
                if object in item["name"]:
                    object_wid = (item["xmax"] - item["xmin"]) / 1920
                    return object_wid
    return  -1

def object_height(object: str) -> float:
    for x in range(0,4):
        scene_description = get_latest_detection()
        for item in scene_description:
                if object in item["name"]:
                    object_height = (item["ymax"] - item["ymin"]) / 1200
                    return object_height
    return  -1

def object_dis(object_name: str) -> float:
    pass  # Implement your function here

def probe(question: str) -> str:
    pass  # Implement your function here

def log(text: str) -> None:
    pass  # Implement your function here

def take_picture() -> None:
    pass  # Implement your function here

def re_plan() -> None:
    pass  # Implement your function here

def orienting(object_name: str) -> bool:
    pass  # Implement your function here

def approach() -> None:
    pass  # Implement your function here

