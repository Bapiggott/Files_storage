# functions.py
import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import requests
from mavsdk.offboard import VelocityNedYaw
# from mavsdk import ActionError
import datetime

drone = System()
starting_position = None

async def connect_drone():
    global drone
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            break


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
    # Wait for the drone to reach the position
    await asyncio.sleep(60)
    print("Returned to starting position.")


async def ensure_armed_and_taken_off():
    global drone, starting_position

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
        print("Setting maximum ascent rate...")
        await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", 9.0)  # Set ascent rate to 3.0 m/s

        print("Taking off...")
        try:
            await drone.action.set_takeoff_altitude(2.0)
            await drone.action.takeoff()
            await asyncio.sleep(100)  # Adjust sleep time based on ascent speed
            print("Drone has taken off")
        except Exception as e:
            print(f"Failed to take off: {e}")
            return
    else:
        print("Drone is already in the air")

    """async for position in drone.telemetry.position():
        starting_position = position
        print(
            f"Starting position recorded: Lat {starting_position.latitude_deg}, Lon {starting_position.longitude_deg}, Alt {starting_position.relative_altitude_m} meters"
        )
        break"""

"""async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return"""


async def ensure_armed_and_taken_off1():
    global drone, starting_position

    # Get the current armed status
    is_armed = False  # Default value
    async for armed in drone.telemetry.armed():
        is_armed = armed
        break  # Exit after getting the first value

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

    # Get the current in_air status
    in_air = False  # Default value
    async for air in drone.telemetry.in_air():
        in_air = air
        break

    if not in_air:
        print("Setting takeoff altitude to 2.5 meter...")
        try:
            await drone.action.set_takeoff_altitude(1.5)  # Set takeoff altitude to 1.0 meter
            print("Takeoff altitude set")
        except Exception as e:
            print(f"Failed to set takeoff altitude: {e}")
            return

        print("Taking off...")
        try:
            await drone.action.takeoff()
            await asyncio.sleep(40)  # Wait for the drone to take off
            print("Drone has taken off")
        except Exception as e:
            print(f"Failed to take off: {e}")
            return
    else:
        print("Drone is already in the air")
    await asyncio.sleep(40)
    # Record the starting position
    """async for position in drone.telemetry.position():
        starting_position = position
        print(
            f"Starting position recorded: Lat {starting_position.latitude_deg}, Lon {starting_position.longitude_deg}, Alt {starting_position.relative_altitude_m}"
        )
        break"""


async def moving_drone(distance: int, direction: str) -> None:

    global drone
    heading_deg = await get_heading()
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
    """drone = System()
    await drone.connect(system_address="udp://:14540")
    print("Waiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print("Drone connected")
            break"""
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
    global drone

    # Subscribe to telemetry heading data
    async for position in drone.telemetry.heading():
        print(f"Current Heading: {position.heading_deg} degrees")
        # break  # Exit after one reading, or remove this to continuously get updates
        return position.heading_deg


async def move_forward(distance: int) -> None:
    await moving_drone(distance, "forward")


async def move_backward(distance: int) -> None:
    await moving_drone(distance, "backward")


async def set_heading_with_velocity(yaw_deg):
    global drone

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
async def move_left(distance: int) -> None:
    await moving_drone(distance, "left")


async def move_right(distance: int) -> None:
    await moving_drone(distance, "right")


async def changing_elevation(distance: int) -> None:
    global drone

    # Ensure the drone is armed and has taken off
    # await ensure_armed_and_taken_off()
    heading_deg = await get_heading()

    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, heading_deg))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        await drone.action.disarm()
        return

    # Move the drone based on the distance
    if distance <= 0:
        print(f"Moving up by {-distance} meters ...")
    else:
        print(f"Moving down by {distance} meters ...")

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -distance, heading_deg))
    await asyncio.sleep(10)

    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
        print("Offboard mode stopped successfully")
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")
    print("end")


async def move_up(distance: float) -> None:
    await changing_elevation(-distance)


async def move_down(distance: float) -> None:
    await changing_elevation(distance)


async def turn_cw(degrees: float) -> None:
    heading_deg = await get_heading()
    while degrees >= 360:
        degrees = degrees - 360
    await set_heading_with_velocity(heading_deg + degrees)

async def turn_ccw(degrees: float) -> None:
    heading_deg = await get_heading()
    while degrees >= 360:
        degrees = degrees - 360
    await set_heading_with_velocity(heading_deg - degrees)

async def move_in_circle(cw: bool) -> None:
    heading_deg = await get_heading()
    for degrees in range(10):
        await set_heading_with_velocity(heading_deg + (36*degrees))

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


def object_x(object: str) -> float:
    for x in range(0,4):
        scene_description = get_latest_detection()
        for item in scene_description:
                #print(item)
                if object in item["name"]:
                    objectx = ((item["xmax"] + item["xmin"]) / 2) / 1920
                    return objectx
    return  -1


def object_y(object: str) -> float:
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


def get_latest_image():
    """
    Retrieves the latest image from the YOLO server and saves it with a timestamp.
    """
    yolo_server_url = "http://localhost:5000/image/latest"
    try:
        response = requests.get(yolo_server_url)
        if response.status_code == 200:
            # Get the current timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"latest_image_{timestamp}.jpg"

            # Save the image to the current directory
            with open(filename, 'wb') as f:
                f.write(response.content)

            print(f"Image saved as {filename}")
            return filename
        else:
            print(f"Failed to get image: Status code {response.status_code}")
            return None
    except requests.exceptions.RequestException as e:
        print(f"Request failed: {e}")
        return None


def display_image(filename):
    """
    Displays the image in the terminal using ASCII characters (optional).
    Note: This requires the 'PIL' and 'numpy' libraries.
    """
    try:
        from PIL import Image
        import numpy as np

        img = Image.open(filename)
        img.thumbnail((80, 40))  # Resize for terminal display
        img_array = np.array(img)

        # Convert to grayscale
        img_gray = img.convert('L')
        img_array = np.array(img_gray)

        # Define ASCII characters
        ascii_chars = [' ', '.', ':', '-', '=', '+', '*', '#', '%', '@']
        ascii_str = ''
        for pixel_row in img_array:
            for pixel in pixel_row:
                ascii_str += ascii_chars[pixel // 25]
            ascii_str += '\n'

        print(ascii_str)
    except ImportError:
        print("PIL and numpy are required for displaying the image in the terminal.")
        print("You can install them using 'pip install Pillow numpy'.")


def take_picture() -> None:
    filename = get_latest_image()
    if filename:
        # Optionally display the image in the terminal
        display_image(filename)


def object_dis(object_name: str) -> float:
    # need to look how typefly got this
    pass

def probe(question: str) -> bool:
    False

def log(text) -> None:
    # no need for def
    pass


def re_plan() -> None:
    # no need for def
    pass

async def orienting(object_name: str) -> bool:
    print("Orienting...")
    for _ in range(4):
        _1 =  object_x(object_name)
        if _1 > 0.6:
            await turn_cw(15)
        if _1 < 0.4:
            await turn_ccw(15)
        _2 = object_x(object_name)
        if 0.4 < _2 < 0.6:
            return True
    return False


async def approach() -> None:
    await move_forward(1)


async def scan(object_name: str) -> bool:
    for _ in range(8):
        if is_visible(object_name):
            return True
        await turn_cw(45)
    return False


async def scan_abstract(question: str) -> bool:
    for _ in range(8):
        _1 = probe(question)
        if _1 != False:
            return _1
        await turn_cw(45)
    return False


async def goto(object_name: str) -> None:
    await orienting(object_name)
    await approach()


def is_visible(object_name: str) -> bool:
    object_name = object_name.lower()
    for x in range(0, 4):
        scene_description = get_latest_detection()
        for item in scene_description:
            if object_name in item["name"].lower():
                return True
    return False
