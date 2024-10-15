# functions.py
import asyncio
import math
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw
import requests
from mavsdk.offboard import VelocityNedYaw
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
        await drone.param.set_param_float("MPC_Z_VEL_MAX_UP", 9.0)  # Set ascent rate to 9.0 m/s

        print("Taking off...")
        try:
            await drone.action.set_takeoff_altitude(2.0)
            await drone.action.takeoff()
            await asyncio.sleep(10)  # Adjust sleep time based on ascent speed
            print("Drone has taken off")
        except Exception as e:
            print(f"Failed to take off: {e}")
            return
    else:
        print("Drone is already in the air")

    # Record the starting position
    """async for position in drone.telemetry.position():
        starting_position = position
        print(
            f"Starting position recorded: Lat {starting_position.latitude_deg}, Lon {starting_position.longitude_deg}, Alt {starting_position.relative_altitude_m} meters"
        )
        break
    """
    # Start offboard mode here
    print("-- Setting initial setpoint")
    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))

    print("-- Starting offboard")
    try:
        await drone.offboard.start()
        print("Offboard mode started")
    except OffboardError as error:
        print(f"Starting offboard mode failed with error code: {error._result.result}")
        await drone.action.disarm()
        return

async def print_status_text(drone):
    try:
        async for status_text in drone.telemetry.status_text():
            print(f"Status: {status_text.type}: {status_text.text}")
    except asyncio.CancelledError:
        return

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

    # Move the drone based on calculated NED position
    print(f"Moving {direction} by {distance} meters (Heading: {heading_deg} degrees)...")
    await drone.offboard.set_position_ned(PositionNedYaw(north, east, 0.0, heading_deg))
    await asyncio.sleep(10)  # Allow the drone to move

async def get_heading():
    global drone

    # Subscribe to telemetry heading data
    async for position in drone.telemetry.heading():
        print(f"Current Heading: {position.heading_deg} degrees")
        return position.heading_deg

async def move_forward(distance: int) -> None:
    await moving_drone(distance, "forward")

async def move_backward(distance: int) -> None:
    await moving_drone(distance, "backward")


from mavsdk.offboard import VelocityBodyYawspeed


async def set_heading_with_velocity(target_yaw_deg):
    global drone

    current_heading = await get_heading()
    yaw_diff = (target_yaw_deg - current_heading + 360) % 360
    if yaw_diff > 180:
        yaw_diff -= 360

    # Set a yaw rate (degrees per second)
    yaw_rate_deg_s = 30.0  # Adjust as needed

    # Determine rotation direction
    if yaw_diff < 0:
        yaw_rate_deg_s = -yaw_rate_deg_s

    rotate_time = abs(yaw_diff) / abs(yaw_rate_deg_s)

    print(f"Rotating from {current_heading:.2f} degrees to {target_yaw_deg:.2f} degrees at {yaw_rate_deg_s} deg/s")

    # Start rotation
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, yaw_rate_deg_s))

    await asyncio.sleep(rotate_time)

    # Stop rotation
    await drone.offboard.set_velocity_body(VelocityBodyYawspeed(0.0, 0.0, 0.0, 0.0))

    # Verify final heading
    final_heading = await get_heading()
    print(f"Final Heading: {final_heading:.2f} degrees")


"""async def set_heading_with_velocity(yaw_deg):
    global drone

    print(f"Setting heading to {yaw_deg} degrees...")
    await drone.offboard.set_velocity_ned(VelocityNedYaw(0, 0, 0, yaw_deg))
    await asyncio.sleep(5)"""

async def move_left(distance: int) -> None:
    await moving_drone(distance, "left")

async def move_right(distance: int) -> None:
    await moving_drone(distance, "right")

async def changing_elevation(distance: int) -> None:
    global drone

    heading_deg = await get_heading()

    # Move the drone based on the distance
    if distance <= 0:
        print(f"Moving up by {-distance} meters ...")
    else:
        print(f"Moving down by {distance} meters ...")

    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, -distance, heading_deg))
    await asyncio.sleep(20)

async def move_up(distance: float) -> None:
    await changing_elevation(-distance)

async def move_down(distance: float) -> None:
    await changing_elevation(distance)

async def turn_cw(degrees: float) -> None:
    heading_deg = await get_heading()
    new_heading = (heading_deg + degrees) % 360
    await set_heading_with_velocity(new_heading)

async def turn_ccw(degrees: float) -> None:
    heading_deg = await get_heading()
    new_heading = (heading_deg - degrees + 360) % 360
    await set_heading_with_velocity(new_heading)

"""async def turn_cw(degrees: float) -> None:
    heading_deg = await get_heading()
    new_heading = (heading_deg + degrees) % 360
    await set_heading_with_velocity(new_heading)

async def turn_ccw(degrees: float) -> None:
    heading_deg = await get_heading()
    new_heading = (heading_deg - degrees) % 360
    await set_heading_with_velocity(new_heading)"""

async def move_in_circle(cw: bool) -> None:
    heading_deg = await get_heading()
    for degrees in range(0, 360, 36):
        new_heading = (heading_deg + degrees) % 360 if cw else (heading_deg - degrees) % 360
        await set_heading_with_velocity(new_heading)
        await asyncio.sleep(1)

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

def object_x(object_name: str) -> float:
    for _ in range(4):
        scene_description = get_latest_detection()
        if scene_description:
            for item in scene_description:
                if object_name.lower() in item["name"].lower():
                    objectx = ((item["xmax"] + item["xmin"]) / 2) / 1920
                    return objectx
    return -1

def object_y(object_name: str) -> float:
    for _ in range(4):
        scene_description = get_latest_detection()
        if scene_description:
            for item in scene_description:
                if object_name.lower() in item["name"].lower():
                    objecty = ((item["ymax"] + item["ymin"]) / 2) / 1200
                    return objecty
    return -1

def object_width(object_name: str) -> float:
    for _ in range(4):
        scene_description = get_latest_detection()
        if scene_description:
            for item in scene_description:
                if object_name.lower() in item["name"].lower():
                    object_wid = (item["xmax"] - item["xmin"]) / 1920
                    return object_wid
    return -1

def object_height(object_name: str) -> float:
    for _ in range(4):
        scene_description = get_latest_detection()
        if scene_description:
            for item in scene_description:
                if object_name.lower() in item["name"].lower():
                    object_height = (item["ymax"] - item["ymin"]) / 1200
                    return object_height
    return -1

def get_latest_image():
    """
    Retrieves the latest image from the YOLO server and saves it with a timestamp.
    """
    yolo_server_url = "http://localhost:5000/image/latest"
    try:
        response = requests.get(yolo_server_url)
        if response.status_code == 200:
            # Get the current timestamp
            timestamp = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
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
    # Placeholder function; implement as needed
    pass

def probe(question: str) -> bool:
    # Placeholder function; implement as needed
    return False

def log(text) -> None:
    print(f"LOG: {text}")

def re_plan() -> None:
    # Placeholder function; implement as needed
    pass

"""async def orienting(object_name: str) -> bool:
    for _ in range(4):
        _1 = object_x(object_name)
        if _1 == -1:
            print(f"{object_name} not found in current frame.")
            return False
        if _1 > 0.6:
            await turn_cw(15)
        elif _1 < 0.4:
            await turn_ccw(15)
        else:
            return True
    return False"""

async def orienting(object_name: str) -> bool:
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
    success = await orienting(object_name)
    if success:
        await approach()
    else:
        print(f"Could not orient towards {object_name}.")

def is_visible(object_name: str) -> bool:
    object_name = object_name.lower()
    scene_description = get_latest_detection()
    if scene_description:
        for item in scene_description:
            if object_name in item["name"].lower():
                return True
    return False

# Ensure offboard mode is stopped at the end of your main function or mission
async def stop_offboard_mode():
    global drone
    print("-- Stopping offboard")
    try:
        await drone.offboard.stop()
        print("Offboard mode stopped")
    except OffboardError as error:
        print(f"Stopping offboard mode failed with error code: {error._result.result}")
